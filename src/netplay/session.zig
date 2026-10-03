const std = @import("std");
const iroh = @import("iroh");
const env = @import("../env.zig");
const protocol = @import("protocol.zig");
const types = @import("types.zig");
const Ref = @import("../utils/types.zig").Ref;

const EndpointRef = Ref(iroh.Endpoint);
const ConnectionRef = Ref(iroh.Connection);
const BiStreamRef = Ref(iroh.BiStream);
const SendStreamRef = Ref(iroh.SendStream);
const RecvStreamRef = Ref(iroh.RecvStream);
pub const MessageRef = types.MessageRef;
pub const PreviewRef = types.PreviewRef;
pub const BytesRef = types.BytesRef;

pub const Role = types.Role;
pub const State = types.State;
pub const Event = types.Event;
pub const EventRef = types.EventRef;
pub const ConnectionStats = iroh.ConnectionStats;
pub const ConnectionRoute = iroh.ConnectionRoute;

const max_queue_items = 128;

/// An encoded message waiting for the worker. The tag lets the worker tell
/// handshake responses apart without decoding its own output.
const Outgoing = struct {
    tag: protocol.MessageTag,
    bytes: []u8,
};

/// Owns the complete blocking iroh lifecycle. UI and emulation code interact
/// through queues and synchronized value snapshots, never borrowed FFI
/// handles.
pub const SessionManager = struct {
    alloc: std.mem.Allocator,
    io: std.Io,
    mutex: std.Io.Mutex = .init,
    wake: std.Io.Condition = .init,
    role: Role = .none,
    state: State = .idle,
    cancelled: bool = false,
    graceful_shutdown: bool = false,
    worker: ?std.Thread = null,
    shutdown_worker: ?std.Thread = null,
    timeout_worker: ?std.Thread = null,
    // The session worker owns these handles on its stack. The manager borrows
    // them only so shutdown can interrupt blocking iroh operations.
    endpoint: ?EndpointRef.Borrowed = null,
    connection: ?ConnectionRef.Borrowed = null,
    outgoing: std.ArrayList(Outgoing) = .empty,
    events: std.ArrayList(EventRef.Owned) = .empty,
    preset: iroh.Preset = .n0,

    const Self = @This();

    pub fn init(alloc: std.mem.Allocator, io: std.Io) Self {
        std.log.debug("netplay: initializing session manager", .{});
        return .{ .alloc = alloc, .io = io };
    }

    pub fn deinit(self: *Self) void {
        std.log.debug("netplay: deinitializing session manager", .{});

        self.cancel();
        if (self.worker) |thread| thread.join();
        if (self.shutdown_worker) |thread| thread.join();
        if (self.timeout_worker) |thread| thread.join();

        self.mutex.lockUncancelable(self.io);
        self.clear();
        self.outgoing.deinit(self.alloc);
        self.events.deinit(self.alloc);
        self.mutex.unlock(self.io);
    }

    pub fn getRole(self: *Self) Role {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);
        return self.role;
    }

    pub fn getState(self: *Self) State {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);
        return self.state;
    }

    /// Copies current transport telemetry while holding the lock that protects
    /// the worker-owned connection handle.
    pub fn getConnectionStats(self: *Self) !?ConnectionStats {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        const connection = self.connection orelse return null;
        return try connection.get().stats();
    }

    pub fn isActive(self: *Self) bool {
        return self.getRole() != .none;
    }

    pub fn startHost(self: *Self, preview: PreviewRef.Owned) !void {
        try self.startHostWithPreset(preview, .n0);
    }

    /// Takes ownership of the preview payload only when startup succeeds.
    pub fn startHostWithPreset(self: *Self, preview: PreviewRef.Owned, preset: iroh.Preset) !void {
        std.log.info("netplay: host requested for ROM '{s}' ({d} bytes), endpoint preset={s}", .{
            preview.value.name,
            preview.value.rom_size,
            @tagName(preset),
        });

        self.reapFinished();
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.role != .none) return error.SessionAlreadyActive;
        try verifyAbi();
        self.clear();

        self.role = .host;
        self.state = .creating;
        self.cancelled = false;
        self.graceful_shutdown = false;
        self.preset = preset;
        errdefer {
            self.role = .none;
            self.state = .idle;
        }

        self.worker = try std.Thread.spawn(.{}, hostMain, .{ self, preview });
        self.pushEvent(.init(.{ .state = .creating }));

        std.log.debug("netplay: host worker started", .{});
    }

    pub fn connect(self: *Self, code: []const u8) !void {
        try self.connectWithPreset(code, .n0);
    }

    pub fn connectWithPreset(self: *Self, code: []const u8, preset: iroh.Preset) !void {
        const ticket = protocol.parseSessionCode(code) catch |err| {
            std.log.err("netplay: rejected session code (length={d}): {s}", .{
                std.mem.trim(u8, code, " \t\r\n").len,
                @errorName(err),
            });
            return err;
        };

        std.log.info("netplay: client connection requested (ticket_length={d}, endpoint preset={s})", .{
            ticket.len,
            @tagName(preset),
        });

        self.reapFinished();
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.role != .none) return error.SessionAlreadyActive;
        try verifyAbi();
        self.clear();

        const owned_ticket = try self.alloc.dupe(u8, ticket);
        errdefer self.alloc.free(owned_ticket);

        self.role = .client;
        self.state = .connecting;
        self.cancelled = false;
        self.graceful_shutdown = false;
        self.preset = preset;
        errdefer {
            self.role = .none;
            self.state = .idle;
        }

        self.worker = try std.Thread.spawn(.{}, clientMain, .{ self, BytesRef.Owned.init(owned_ticket) });
        self.pushEvent(.init(.{ .state = .connecting }));
        self.timeout_worker = std.Thread.spawn(.{}, timeoutMain, .{ self, State.connecting }) catch |err| blk: {
            std.log.err("netplay: failed to start setup-timeout worker: {s}", .{@errorName(err)});
            break :blk null;
        };

        std.log.debug("netplay: client and setup-timeout workers started", .{});
    }

    /// Fails when the session ended or moved on before the user confirmed.
    pub fn acceptPreview(self: *Self) !void {
        if (self.getState() != .preview) return error.InvalidSessionState;

        std.log.info("netplay: client accepted preview and requested to join", .{});
        self.send(.init(&.{ .join = {} }));

        self.setState(.joining);
        self.restartTimeout(.joining);
    }

    /// Queues `message` for the peer. Messages for a closed session are
    /// dropped. A full queue means the peer stopped reading, which fails the
    /// session.
    pub fn send(self: *Self, message_ref: MessageRef.Borrowed) void {
        const message = message_ref.get().*;
        const encoded = protocol.encode(self.alloc, message) catch @panic("OOM");

        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.cancelled or self.role == .none) {
            std.log.debug("netplay: dropping {s} message for a closed session", .{@tagName(message)});
            self.alloc.free(encoded);
            return;
        }

        if (self.outgoing.items.len >= max_queue_items) {
            std.log.err("netplay: outgoing queue full; cannot queue {s} message (role={s}, capacity={d})", .{
                @tagName(message),
                @tagName(self.role),
                max_queue_items,
            });
            self.alloc.free(encoded);
            self.abortLocked("The peer stopped receiving messages");
            return;
        }

        self.outgoing.append(self.alloc, .{ .tag = message, .bytes = encoded }) catch @panic("OOM");
        logMessage(.queued, self.role, message, self.outgoing.items.len, encoded.len);
        self.wake.signal(self.io);
    }

    /// Payload events of a session the application already left are dropped.
    pub fn pollEvent(self: *Self) ?EventRef.Owned {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        while (self.events.items.len != 0) {
            var event = self.events.orderedRemove(0);
            if (self.cancelled and event.value.isPayload()) {
                std.log.debug("netplay: discarding {s} event of a closed session", .{@tagName(event.value)});
                event.value.deinit(self.alloc);
                continue;
            }
            return event;
        }
        return null;
    }

    pub fn cancel(self: *Self) void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.role == .none or self.cancelled) return;

        std.log.info("netplay: cancelling {s} session from state {s}", .{ @tagName(self.role), @tagName(self.state) });
        self.cancelled = true;

        if (self.state != .failed) {
            self.state = .disconnecting;
            self.pushEvent(.init(.{ .state = .disconnecting }));
        }

        self.wake.broadcast(self.io);
        self.startShutdown();
    }

    pub fn disconnect(self: *Self) void {
        std.log.info("netplay: graceful disconnect requested", .{});
        self.send(.init(&.{ .disconnect = "peer left the session" }));

        self.mutex.lockUncancelable(self.io);
        self.graceful_shutdown = true;

        // During preview the client worker is waiting on the outgoing queue
        // and owns the stream. Let it send the decline before closing the
        // transport; finishWorker will complete the normal cleanup.
        if (!self.cancelled and self.role == .client and self.state == .preview) {
            std.log.info("netplay: client declining session preview", .{});
            self.state = .disconnecting;
            self.pushEvent(.init(.{ .state = .disconnecting }));
            self.wake.broadcast(self.io);
            self.mutex.unlock(self.io);
            return;
        }

        self.mutex.unlock(self.io);
        self.cancel();
    }

    pub fn markResyncing(self: *Self) void {
        if (self.getState() == .connected) {
            std.log.info("netplay: entering resynchronization", .{});
            self.setState(.resyncing);
        }
    }

    pub fn markConnected(self: *Self) void {
        if (self.getState() == .resyncing) {
            std.log.info("netplay: resynchronization completed", .{});
            self.setState(.connected);
        }
    }

    fn hostMain(self: *Self, preview: PreviewRef.Owned) void {
        std.log.debug("netplay: host worker entered", .{});
        defer {
            self.alloc.free(preview.value.name);
            self.alloc.free(preview.value.framebuffer);
        }

        self.runHost(preview.value) catch |err| self.reportFailure(err);
        self.finishWorker();
    }

    fn runHost(self: *Self, preview: protocol.Preview) !void {
        std.log.info("netplay: host binding endpoint (preset={s}, ALPN={s})", .{ @tagName(self.preset), protocol.alpn });
        var endpoint = EndpointRef.Owned.init(
            try iroh.Endpoint.bind(self.alloc, .{ .preset = self.preset, .alpns = &.{protocol.alpn} }),
        );
        defer closeAndFreeEndpoint(&endpoint.value);
        std.log.info("netplay: host endpoint bound", .{});

        try self.publishEndpoint(endpoint.borrow());
        defer self.joinShutdown();

        const ticket = try endpoint.value.ticket(self.alloc);
        defer self.alloc.free(ticket);

        const code = try protocol.makeSessionCode(self.alloc, ticket);
        std.log.info("netplay: session code generated (length={d}); waiting for one client", .{code.len});
        self.pushEventLocked(.init(.{ .session_code = code }));

        const encoded_preview = try protocol.encode(self.alloc, .{ .preview = preview });
        defer self.alloc.free(encoded_preview);

        std.log.info("netplay: host sending preview for '{s}' (rom={d} bytes, framebuffer={d} bytes, framed={d} bytes)", .{
            preview.name,
            preview.rom_size,
            protocol.framebuffer_size,
            encoded_preview.len,
        });

        while (!self.isCancelled()) {
            // A peer that leaves before accepting the preview has not joined
            // the session. Keep the endpoint and ticket alive so another peer
            // can connect with the same session code.
            self.setState(.waiting);

            var connection = ConnectionRef.Owned.init(try endpoint.value.accept());
            defer connection.value.deinit();
            std.log.info("netplay: host accepted incoming connection", .{});

            try self.publishConnection(connection.borrow());
            defer self.clearPublishedConnection();

            var peer_joined = false;
            const completed = self.runHostConnection(connection.borrow(), encoded_preview, &peer_joined) catch |err| {
                if (self.isCancelled() or peer_joined) return err;

                std.log.info("netplay: prospective client left before joining: {s}; waiting for another client", .{@errorName(err)});
                continue;
            };

            if (completed) return;
            std.log.info("netplay: prospective client declined the session; waiting for another client", .{});
        }

        return error.SessionClosed;
    }

    /// Returns false when the prospective client declines joining the session. Once a
    /// join request is received, ending this connection ends the host session.
    fn runHostConnection(
        self: *Self,
        connection: ConnectionRef.Borrowed,
        encoded_preview: []const u8,
        peer_joined: *bool,
    ) !bool {
        const remote = try connection.get().remoteId();
        self.pushEventLocked(.init(.{ .peer = remote.bytes }));
        std.log.info("netplay: host connected to peer {x}", .{remote.bytes[0..8]});

        std.log.debug("netplay: host opening bidirectional stream", .{});
        var stream = BiStreamRef.Owned.init(try connection.get().openBi());
        std.log.debug("netplay: host opened bidirectional stream", .{});
        defer stream.value.deinit();
        defer self.joinShutdown();

        // The BiStream remains owned by this worker until both the connected
        // loop and its sender thread have finished.
        const send_stream = SendStreamRef.Borrowed.init(&stream.value.send);
        const recv_stream = RecvStreamRef.Borrowed.init(&stream.value.recv);

        try send_stream.get().writeAll(encoded_preview);

        var join = try recvMessage(self.alloc, recv_stream);
        defer join.value.deinit(self.alloc);

        switch (join.value) {
            .join => peer_joined.* = true,
            .disconnect => |reason| {
                std.log.info("netplay: client declined session preview: {s}", .{reason});
                return false;
            },
            else => return error.UnexpectedHandshakeMessage,
        }

        std.log.info("netplay: host received join request", .{});
        self.pushEventLocked(.init(.join_requested));
        self.setState(.joining);

        const join_data = try self.waitOutgoing();
        defer self.alloc.free(join_data.bytes);
        std.debug.assert(join_data.tag == .join_data);

        std.log.info("netplay: host sending ROM and snapshot transfer ({d} framed bytes)", .{join_data.bytes.len});
        try send_stream.get().writeAll(join_data.bytes);

        var ready = try recvMessage(self.alloc, recv_stream);
        if (ready.value != .ready) {
            ready.value.deinit(self.alloc);
            return error.UnexpectedHandshakeMessage;
        }

        std.log.info(
            "netplay: host received client ready (epoch={d}, frame={d})",
            .{ ready.value.ready.epoch, ready.value.ready.frame },
        );

        self.pushEventLocked(.init(.{ .message = ready.value }));
        self.setState(.connected);

        try self.runConnected(send_stream, recv_stream);
        return true;
    }

    fn clientMain(self: *Self, ticket: BytesRef.Owned) void {
        std.log.debug("netplay: client worker entered", .{});
        defer self.alloc.free(ticket.value);

        self.runClient(ticket.value) catch |err| self.reportFailure(err);
        self.finishWorker();
    }

    fn runClient(self: *Self, ticket: []const u8) !void {
        std.log.info("netplay: client binding local endpoint (preset={s})", .{@tagName(self.preset)});
        var endpoint = EndpointRef.Owned.init(
            try iroh.Endpoint.bind(self.alloc, .{ .preset = self.preset }),
        );
        defer closeAndFreeEndpoint(&endpoint.value);
        std.log.info("netplay: client endpoint bound", .{});

        try self.publishEndpoint(endpoint.borrow());
        defer self.joinShutdown();

        std.log.info("netplay: client connecting to host (ticket_length={d}, ALPN={s})", .{
            ticket.len,
            protocol.alpn,
        });

        var connection = ConnectionRef.Owned.init(try endpoint.value.connect(ticket, protocol.alpn));
        defer connection.value.deinit();
        std.log.info("netplay: client connection established", .{});

        try self.publishConnection(connection.borrow());
        const remote = try connection.value.remoteId();
        std.log.info("netplay: client connected to peer {x}", .{remote.bytes[0..8]});

        self.pushEventLocked(.init(.{ .peer = remote.bytes }));
        std.log.debug("netplay: client waiting for bidirectional stream", .{});

        var stream = BiStreamRef.Owned.init(try connection.value.acceptBi());
        std.log.debug("netplay: client accepted bidirectional stream", .{});
        defer stream.value.deinit();
        defer self.joinShutdown();

        // The BiStream remains owned by this worker until both the connected
        // loop and its sender thread have finished.
        const send_stream = SendStreamRef.Borrowed.init(&stream.value.send);
        const recv_stream = RecvStreamRef.Borrowed.init(&stream.value.recv);

        var preview_message = try recvMessage(self.alloc, recv_stream);
        if (preview_message.value != .preview) {
            preview_message.value.deinit(self.alloc);
            return error.UnexpectedHandshakeMessage;
        }

        const preview = preview_message.value.preview;

        std.log.info("netplay: client received preview for '{s}' ({d} bytes)", .{ preview.name, preview.rom_size });
        self.pushEventLocked(.init(.{ .preview = preview }));
        self.setState(.preview);

        // `acceptPreview` and `disconnect` are the only messages sent during preview.
        const response = try self.waitOutgoing();
        defer self.alloc.free(response.bytes);

        switch (response.tag) {
            .join => {
                std.log.debug("netplay: client sending join request", .{});
                try send_stream.get().writeAll(response.bytes);
            },
            .disconnect => {
                std.log.info("netplay: client sending preview decline", .{});
                send_stream.get().writeAll(response.bytes) catch |err| {
                    std.log.debug("netplay: preview decline send interrupted by shutdown: {s}", .{@errorName(err)});
                };
                return;
            },
            else => unreachable,
        }

        var join_data = try recvMessage(self.alloc, recv_stream);
        if (join_data.value != .join_data) {
            join_data.value.deinit(self.alloc);
            return error.UnexpectedHandshakeMessage;
        }

        std.log.info("netplay: client received ROM and snapshot (rom={d} bytes, snapshot={d} bytes, epoch={d}, frame={d})", .{
            join_data.value.join_data.rom.len,
            join_data.value.join_data.snapshot.len,
            join_data.value.join_data.epoch,
            join_data.value.join_data.frame,
        });

        self.pushEventLocked(.init(.{ .message = join_data.value }));

        const ready = try self.waitOutgoing();
        defer self.alloc.free(ready.bytes);
        std.debug.assert(ready.tag == .ready);

        std.log.debug("netplay: client sending ready acknowledgement", .{});
        try send_stream.get().writeAll(ready.bytes);

        self.setState(.connected);
        try self.runConnected(send_stream, recv_stream);
    }

    fn runConnected(self: *Self, send_stream: SendStreamRef.Borrowed, recv_stream: RecvStreamRef.Borrowed) !void {
        const role = self.getRole();
        std.log.info("netplay: connected message loop started for {s}", .{@tagName(role)});

        const sender = try std.Thread.spawn(.{}, sendMain, .{ self, send_stream });
        defer {
            self.mutex.lockUncancelable(self.io);
            self.cancelled = true;
            self.wake.broadcast(self.io);
            self.mutex.unlock(self.io);

            sender.join();
        }

        while (!self.isCancelled()) {
            var message = recvMessage(self.alloc, recv_stream) catch |err| {
                if (self.isCancelled()) {
                    std.log.debug("netplay: receive unblocked during shutdown: {s}", .{@errorName(err)});
                    return;
                }

                std.log.err("netplay: stream receive failed: {s}", .{@errorName(err)});
                return err;
            };

            if (message.value == .disconnect) {
                std.log.info("netplay: peer requested disconnect: {s}", .{message.value.disconnect});
                message.value.deinit(self.alloc);
                self.pushEventLocked(.init(.peer_disconnected));
                return;
            }

            if (!isExpectedFromPeer(role, message.value)) {
                std.log.err("netplay: unexpected {s} message from peer (role={s})", .{
                    @tagName(message.value),
                    @tagName(role),
                });
                message.value.deinit(self.alloc);
                return error.UnexpectedSessionMessage;
            }

            logMessage(.received, role, message.value, null, null);
            self.pushEventLocked(.init(.{ .message = message.value }));
        }
    }

    fn sendMain(self: *Self, stream: SendStreamRef.Borrowed) void {
        std.log.debug("netplay: sender worker started", .{});

        while (self.waitOutgoing()) |outgoing| {
            defer self.alloc.free(outgoing.bytes);

            stream.get().writeAll(outgoing.bytes) catch |err| {
                std.log.err("netplay: stream write failed: {s}", .{@errorName(err)});
                self.reportFailure(err);
                self.cancel();
                return;
            };
        } else |err| {
            std.log.debug("netplay: sender worker stopped: {s}", .{@errorName(err)});
        }
    }

    fn waitOutgoing(self: *Self) !Outgoing {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        while (self.outgoing.items.len == 0 and !self.cancelled) self.wake.waitUncancelable(self.io, &self.mutex);

        if (self.outgoing.items.len != 0) return self.outgoing.orderedRemove(0);
        return error.SessionClosed;
    }

    fn setState(self: *Self, state: State) void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.cancelled and state != .failed and state != .idle) return;

        const previous = self.state;
        self.state = state;

        if (previous != state) std.log.info("netplay: state {s} -> {s} (role={s})", .{
            @tagName(previous),
            @tagName(state),
            @tagName(self.role),
        });

        self.pushEvent(.init(.{ .state = state }));
    }

    fn pushEventLocked(self: *Self, event: EventRef.Owned) void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        self.pushEvent(event);
    }

    /// Takes ownership of `event`.
    fn pushEvent(self: *Self, event: EventRef.Owned) void {
        self.events.append(self.alloc, event) catch @panic("OOM");
    }

    /// Reports a worker error. The worker cleans up the transport itself.
    fn reportFailure(self: *Self, err: anyerror) void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.cancelled) return;

        const detail = iroh.lastErrorMessage();
        const message = if (detail.len != 0)
            std.fmt.allocPrint(self.alloc, "{s}: {s}", .{ @errorName(err), detail }) catch @panic("OOM")
        else
            self.alloc.dupe(u8, @errorName(err)) catch @panic("OOM");

        std.log.err("netplay: session failed: {s}", .{message});
        self.failLocked(message);
    }

    /// Fails a running session from outside its worker and interrupts the transport.
    fn abortLocked(self: *Self, detail: []const u8) void {
        self.cancelled = true;
        self.failLocked(self.alloc.dupe(u8, detail) catch @panic("OOM"));
        self.wake.broadcast(self.io);
        self.startShutdown();
    }

    /// Takes ownership of `message`.
    fn failLocked(self: *Self, message: []u8) void {
        self.state = .failed;
        self.pushEvent(.init(.{ .state = .failed }));
        self.pushEvent(.init(.{ .failed = message }));
    }

    fn finishWorker(self: *Self) void {
        self.mutex.lockUncancelable(self.io);

        self.endpoint = null;
        self.connection = null;
        self.cancelled = true;
        self.wake.broadcast(self.io);

        const failed = self.state == .failed;
        const finished_role = self.role;

        self.role = .none;
        if (!failed) self.state = .idle;

        self.pushEvent(.init(.disconnected));
        if (!failed) self.pushEvent(.init(.{ .state = .idle }));

        self.mutex.unlock(self.io);

        std.log.info("netplay: {s} worker finished (failed={any})", .{ @tagName(finished_role), failed });
    }

    fn shutdownMain(self: *Self) void {
        self.mutex.lockUncancelable(self.io);
        const endpoint = self.endpoint;
        const connection = self.connection;
        const graceful = self.graceful_shutdown;
        self.mutex.unlock(self.io);

        std.log.debug("netplay: shutdown worker closing connection and endpoint (graceful={any})", .{graceful});
        if (graceful) sleep(self.io, 20 * std.time.ns_per_ms);

        if (connection) |value| value.get().close(0, "session shutdown") catch |err| {
            std.log.warn("netplay: connection close failed: {s}", .{@errorName(err)});
        };

        if (endpoint) |value| value.get().close() catch |err| {
            std.log.warn("netplay: endpoint close failed: {s}", .{@errorName(err)});
        };

        std.log.debug("netplay: shutdown worker finished", .{});
    }

    fn timeoutMain(self: *Self, armed_state: State) void {
        std.log.debug("netplay: setup timeout armed for state {s} (30 seconds)", .{@tagName(armed_state)});

        const deadline = milliTimestamp(self.io) + 30_000;
        while (milliTimestamp(self.io) < deadline) {
            if (!self.isTimeoutPending(armed_state)) return;
            sleep(self.io, 10 * std.time.ns_per_ms);
        }

        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        // The session may have progressed while the lock was released.
        if (self.role != .client or self.state != armed_state or self.cancelled) return;

        std.log.err("netplay: client setup timed out in state {s} after 30 seconds", .{@tagName(armed_state)});
        self.abortLocked("Connection/setup timed out after 30 seconds");
    }

    fn isTimeoutPending(self: *Self, armed_state: State) bool {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        return self.role == .client and self.state == armed_state and !self.cancelled;
    }

    fn startShutdown(self: *Self) void {
        if (self.shutdown_worker != null) return;

        self.shutdown_worker = std.Thread.spawn(.{}, shutdownMain, .{self}) catch |err| blk: {
            std.log.err("netplay: failed to start shutdown worker: {s}", .{@errorName(err)});
            break :blk null;
        };
    }

    fn joinShutdown(self: *Self) void {
        self.mutex.lockUncancelable(self.io);
        const thread = self.shutdown_worker;
        self.shutdown_worker = null;
        self.mutex.unlock(self.io);

        if (thread) |value| value.join();
    }

    fn restartTimeout(self: *Self, state: State) void {
        self.mutex.lockUncancelable(self.io);
        const old = self.timeout_worker;
        self.timeout_worker = null;
        self.mutex.unlock(self.io);

        if (old) |thread| thread.join();

        self.mutex.lockUncancelable(self.io);
        if (!self.cancelled and self.role == .client and self.state == state) {
            self.timeout_worker = std.Thread.spawn(.{}, timeoutMain, .{ self, state }) catch |err| blk: {
                std.log.err("netplay: failed to restart setup-timeout worker: {s}", .{@errorName(err)});
                break :blk null;
            };
        }
        self.mutex.unlock(self.io);
    }

    fn reapFinished(self: *Self) void {
        self.mutex.lockUncancelable(self.io);
        if (self.role != .none) {
            self.mutex.unlock(self.io);
            return;
        }

        const worker = self.worker;
        const shutdown = self.shutdown_worker;
        const timeout = self.timeout_worker;

        self.worker = null;
        self.shutdown_worker = null;
        self.timeout_worker = null;
        self.mutex.unlock(self.io);

        if (worker) |thread| thread.join();
        if (shutdown) |thread| thread.join();
        if (timeout) |thread| thread.join();
    }

    fn publishEndpoint(self: *Self, endpoint: EndpointRef.Borrowed) !void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.cancelled) return error.SessionClosed;
        self.endpoint = endpoint;
    }

    fn publishConnection(self: *Self, connection: ConnectionRef.Borrowed) !void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        if (self.cancelled) return error.SessionClosed;
        self.connection = connection;
    }

    fn clearPublishedConnection(self: *Self) void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        self.connection = null;
    }

    fn isCancelled(self: *Self) bool {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        return self.cancelled;
    }

    fn clear(self: *Self) void {
        for (self.outgoing.items) |outgoing| self.alloc.free(outgoing.bytes);
        self.outgoing.clearRetainingCapacity();

        for (self.events.items) |*event| event.value.deinit(self.alloc);
        self.events.clearRetainingCapacity();
    }
};

/// After the handshake the host only hears acknowledgements (and `ready` after
/// a rebase); the client only hears authoritative frames, controls and rebases.
fn isExpectedFromPeer(role: Role, message: protocol.Message) bool {
    return switch (role) {
        .host => message == .ready or message == .ack,
        .client => message == .frame or message == .control or message == .rebase,
        .none => unreachable,
    };
}

/// iroh expects `close` before an endpoint is dropped; otherwise its background
/// tasks keep running on the shared runtime. The shutdown worker may already
/// have closed it, but it never sees an endpoint bound after the session was
/// cancelled.
fn closeAndFreeEndpoint(endpoint: *iroh.Endpoint) void {
    if (!endpoint.isClosed()) endpoint.close() catch |err| {
        std.log.warn("netplay: endpoint close failed: {s}", .{@errorName(err)});
    };
    endpoint.deinit();
}

fn verifyAbi() !void {
    const actual = iroh.runtimeAbiVersion();
    if (actual != iroh.abi_version) {
        std.log.err("netplay: incompatible iroh ABI (runtime={d}, bindings={d})", .{ actual, iroh.abi_version });
        return error.IncompatibleIrohAbi;
    }

    std.log.debug("netplay: verified iroh ABI version {d}", .{actual});
}

const MessageDirection = enum { queued, received };

fn logMessage(
    direction: MessageDirection,
    role: Role,
    message: protocol.Message,
    queue_len: ?usize,
    encoded_len: ?usize,
) void {
    if (queue_len) |len| if (len >= max_queue_items * 3 / 4) {
        std.log.warn("netplay: outgoing queue pressure is high ({d}/{d}, role={s})", .{
            len,
            max_queue_items,
            @tagName(role),
        });
    };

    const action = @tagName(direction);
    switch (message) {
        .preview => |value| std.log.debug("netplay: {s} preview (role={s}, name='{s}', rom_size={d})", .{
            action, @tagName(role), value.name, value.rom_size,
        }),
        .join => std.log.debug("netplay: {s} join request (role={s})", .{ action, @tagName(role) }),
        .join_data => |value| std.log.info("netplay: {s} join transfer (role={s}, rom={d} bytes, snapshot={d} bytes, framed={?d} bytes, epoch={d}, frame={d})", .{
            action, @tagName(role), value.rom.len, value.snapshot.len, encoded_len, value.epoch, value.frame,
        }),
        .ready => |value| std.log.info("netplay: {s} ready (role={s}, epoch={d}, frame={d})", .{
            action, @tagName(role), value.epoch, value.frame,
        }),
        .frame => |value| if (value.digest != null) {
            std.log.debug("netplay: {s} checkpoint frame (epoch={d}, frame={d}, queue={?d})", .{
                action, value.epoch, value.frame, queue_len,
            });
        },
        .ack => |value| if (value.digest != null) {
            std.log.debug("netplay: {s} checkpoint acknowledgement (epoch={d}, frame={d}, queue={?d})", .{
                action, value.epoch, value.frame, queue_len,
            });
        },
        .control => |value| switch (value) {
            .paused => |paused| std.log.info("netplay: {s} pause control (paused={any})", .{ action, paused }),
            .speed => |speed| std.log.info("netplay: {s} speed control (value={d})", .{ action, speed }),
        },
        .rebase => |value| std.log.info("netplay: {s} rebase (epoch={d}, frame={d}, snapshot={d} bytes)", .{
            action, value.epoch, value.frame, value.snapshot.len,
        }),
        .disconnect => |reason| std.log.info("netplay: {s} disconnect notice: {s}", .{ action, reason }),
    }
}

fn recvMessage(alloc: std.mem.Allocator, stream: RecvStreamRef.Borrowed) !MessageRef.Owned {
    const header = try stream.get().readExact(alloc, 4);
    defer alloc.free(header);

    const len = std.mem.readInt(u32, header[0..4], .little);
    if (len == 0) return error.TruncatedMessage;
    if (len > protocol.max_message_size) return error.MessageTooLarge;

    const body = try stream.get().readExact(alloc, len);
    defer alloc.free(body);

    return MessageRef.Owned.init(try protocol.decodePayload(alloc, body));
}

fn sleep(io: std.Io, nanoseconds: u64) void {
    std.Io.sleep(io, .fromNanoseconds(@intCast(nanoseconds)), .awake) catch {};
}

fn milliTimestamp(io: std.Io) i64 {
    return std.Io.Timestamp.now(io, .real).toMilliseconds();
}

// Testing

fn checkIfLoopbackTestIsEnabled() !void {
    const enabled = env.get("NESKWIK_NETPLAY_LOOPBACK_TEST") orelse return error.SkipZigTest;

    if (!std.mem.eql(u8, enabled, "1")) return error.SkipZigTest;
}

fn startLoopbackHost(host: *SessionManager, alloc: std.mem.Allocator, name: []const u8) !void {
    const owned_name = try alloc.dupe(u8, name);
    errdefer alloc.free(owned_name);

    const framebuffer = try alloc.alloc(u8, protocol.framebuffer_size);
    errdefer alloc.free(framebuffer);
    @memset(framebuffer, 0x22);

    try host.startHostWithPreset(.init(.{
        .name = owned_name,
        .rom_size = 3,
        .rom_hash = [_]u8{0x11} ** 32,
        .framebuffer = framebuffer,
    }), .minimal);
}

fn waitForSessionCode(host: *SessionManager, deadline: i64) ![]u8 {
    while (milliTimestamp(std.testing.io) < deadline) {
        if (host.pollEvent()) |event_value| {
            var event = event_value;

            if (event.value == .session_code) return event.value.takeSessionCode().value;
            event.value.deinit(host.alloc);
        } else sleep(std.testing.io, std.time.ns_per_ms);
    }

    return error.SessionCodeTimeout;
}

test "session state starts idle and validates codes synchronously" {
    var manager = SessionManager.init(std.testing.allocator, std.testing.io);
    defer manager.deinit();

    try std.testing.expectEqual(Role.none, manager.getRole());
    try std.testing.expectEqual(State.idle, manager.getState());
    try std.testing.expectError(error.InvalidSessionCode, manager.connect("bad-code"));
}

test "payload events of a cancelled session are discarded" {
    const alloc = std.testing.allocator;
    var manager = SessionManager.init(alloc, std.testing.io);
    defer manager.deinit();

    manager.pushEventLocked(.init(.{ .session_code = try alloc.dupe(u8, "neskwik:ticket") }));
    manager.pushEventLocked(.init(.join_requested));
    manager.pushEventLocked(.init(.disconnected));
    manager.cancelled = true;

    var event = manager.pollEvent().?;
    defer event.value.deinit(alloc);
    try std.testing.expect(event.value == .disconnected);
    try std.testing.expect(manager.pollEvent() == null);
}

test "connected peers only accept their role's messages" {
    const hash: protocol.Digest = [_]u8{0} ** 32;
    const framebuffer = [_]u8{0} ** 4;
    const messages = [_]protocol.Message{
        .{ .preview = .{ .name = "x", .rom_size = 0, .rom_hash = hash, .framebuffer = &framebuffer } },
        .{ .join = {} },
        .{ .join_data = .{ .name = "x", .rom = "", .snapshot = "", .speed = 0, .epoch = 0, .frame = 0 } },
        .{ .ready = .{ .epoch = 0, .frame = 0, .player2 = 0 } },
        .{ .frame = .{ .epoch = 0, .frame = 0, .player1 = 0, .player2 = 0 } },
        .{ .ack = .{ .epoch = 0, .frame = 0, .player2 = 0 } },
        .{ .control = .{ .paused = true } },
        .{ .rebase = .{ .epoch = 0, .frame = 0, .snapshot = "" } },
    };
    for (messages) |message| {
        const host_expected = message == .ready or message == .ack;
        const client_expected = message == .frame or message == .control or message == .rebase;
        try std.testing.expectEqual(host_expected, isExpectedFromPeer(.host, message));
        try std.testing.expectEqual(client_expected, isExpectedFromPeer(.client, message));
    }
}

test "local loopback session" {
    const alloc = std.testing.allocator;
    try checkIfLoopbackTestIsEnabled();

    var host = SessionManager.init(alloc, std.testing.io);
    defer host.deinit();
    var client = SessionManager.init(alloc, std.testing.io);
    defer client.deinit();
    try startLoopbackHost(&host, alloc, "loopback.nes");

    const deadline = milliTimestamp(std.testing.io) + 15_000;
    const code = try waitForSessionCode(&host, deadline);
    defer alloc.free(code);
    try client.connectWithPreset(code, .minimal);

    var host_connected = false;
    var client_connected = false;
    var frame_received = false;
    var ack_received = false;

    while (!ack_received and milliTimestamp(std.testing.io) < deadline) {
        while (host.pollEvent()) |event_value| {
            var event = event_value;
            defer event.value.deinit(alloc);

            switch (event.value) {
                .join_requested => host.send(.init(&.{ .join_data = .{
                    .name = "loopback.nes",
                    .rom = "rom",
                    .snapshot = "state",
                    .speed = 1,
                    .epoch = 1,
                    .frame = 0,
                } })),
                .state => |state| if (state == .connected) {
                    host_connected = true;
                },
                .message => |message| if (message == .ack) {
                    try std.testing.expectEqual(@as(u8, 7), message.ack.player2);
                    ack_received = true;
                },
                else => {},
            }
        }

        while (client.pollEvent()) |event_value| {
            var event = event_value;
            defer event.value.deinit(alloc);

            switch (event.value) {
                .preview => try client.acceptPreview(),
                .message => |message| switch (message) {
                    .join_data => |data| client.send(.init(&.{ .ready = .{
                        .epoch = data.epoch,
                        .frame = data.frame,
                        .player2 = 7,
                    } })),
                    .frame => |frame| {
                        frame_received = true;
                        client.send(.init(&.{ .ack = .{
                            .epoch = frame.epoch,
                            .frame = frame.frame,
                            .player2 = 7,
                        } }));
                    },
                    else => {},
                },
                .state => |state| if (state == .connected) {
                    client_connected = true;
                },
                else => {},
            }
        }

        if (host_connected and client_connected and !frame_received) {
            host.send(.init(&.{ .frame = .{
                .epoch = 1,
                .frame = 1,
                .player1 = 1,
                .player2 = 7,
            } }));
        }

        sleep(std.testing.io, std.time.ns_per_ms);
    }

    try std.testing.expect(host_connected and client_connected and frame_received and ack_received);

    const host_stats = (try host.getConnectionStats()).?;
    try std.testing.expect(host_stats.has_selected_path);
    try std.testing.expect(host_stats.route != .unknown);
    try std.testing.expect(host_stats.udp_tx_bytes != 0);
    try std.testing.expect(host_stats.udp_rx_bytes != 0);

    const client_stats = (try client.getConnectionStats()).?;
    try std.testing.expect(client_stats.has_selected_path);
    try std.testing.expect(client_stats.route != .unknown);
    try std.testing.expect(client_stats.udp_tx_bytes != 0);
    try std.testing.expect(client_stats.udp_rx_bytes != 0);

    host.disconnect();
}

test "local loopback session preview decline" {
    const alloc = std.testing.allocator;
    try checkIfLoopbackTestIsEnabled();

    var host = SessionManager.init(alloc, std.testing.io);
    defer host.deinit();
    var declining_client = SessionManager.init(alloc, std.testing.io);
    defer declining_client.deinit();
    try startLoopbackHost(&host, alloc, "declined.nes");

    const deadline = milliTimestamp(std.testing.io) + 15_000;
    const code = try waitForSessionCode(&host, deadline);
    defer alloc.free(code);
    try declining_client.connectWithPreset(code, .minimal);

    var preview_received = false;
    var client_ended = false;
    var host_ended = false;
    var peer_disconnected = false;
    var failure_received = false;

    while (!client_ended and milliTimestamp(std.testing.io) < deadline) {
        while (host.pollEvent()) |event_value| {
            var event = event_value;
            defer event.value.deinit(alloc);

            switch (event.value) {
                .failed => failure_received = true,
                .peer_disconnected => peer_disconnected = true,
                .disconnected => host_ended = true,
                else => {},
            }
        }

        while (declining_client.pollEvent()) |event_value| {
            var event = event_value;
            defer event.value.deinit(alloc);

            switch (event.value) {
                .preview => {
                    preview_received = true;
                    declining_client.disconnect();
                },
                .failed => failure_received = true,
                .disconnected => client_ended = true,
                else => {},
            }
        }

        sleep(std.testing.io, std.time.ns_per_ms);
    }

    try std.testing.expect(preview_received);
    try std.testing.expect(client_ended);
    try std.testing.expect(!host_ended);
    try std.testing.expect(!peer_disconnected);
    try std.testing.expect(!failure_received);
    try std.testing.expectEqual(Role.host, host.getRole());
    try std.testing.expectEqual(State.waiting, host.getState());

    // The same session code remains usable after the first client declines.
    var joined_client = SessionManager.init(alloc, std.testing.io);
    defer joined_client.deinit();
    try joined_client.connectWithPreset(code, .minimal);

    var host_connected = false;
    var client_connected = false;
    var disconnect_requested = false;
    client_ended = false;
    const reconnect_deadline = milliTimestamp(std.testing.io) + 15_000;

    while ((!host_ended or !client_ended) and milliTimestamp(std.testing.io) < reconnect_deadline) {
        while (host.pollEvent()) |event_value| {
            var event = event_value;
            defer event.value.deinit(alloc);

            switch (event.value) {
                .join_requested => host.send(.init(&.{ .join_data = .{
                    .name = "declined.nes",
                    .rom = "rom",
                    .snapshot = "state",
                    .speed = 1,
                    .epoch = 1,
                    .frame = 0,
                } })),
                .state => |state| if (state == .connected) {
                    host_connected = true;
                },
                .failed => failure_received = true,
                .peer_disconnected => peer_disconnected = true,
                .disconnected => host_ended = true,
                else => {},
            }
        }

        while (joined_client.pollEvent()) |event_value| {
            var event = event_value;
            defer event.value.deinit(alloc);

            switch (event.value) {
                .preview => try joined_client.acceptPreview(),
                .message => |message| if (message == .join_data) {
                    joined_client.send(.init(&.{ .ready = .{
                        .epoch = message.join_data.epoch,
                        .frame = message.join_data.frame,
                        .player2 = 0,
                    } }));
                },
                .state => |state| if (state == .connected) {
                    client_connected = true;
                },
                .failed => failure_received = true,
                .disconnected => client_ended = true,
                else => {},
            }
        }

        if (host_connected and client_connected and !disconnect_requested) {
            disconnect_requested = true;
            joined_client.disconnect();
        }

        sleep(std.testing.io, std.time.ns_per_ms);
    }

    try std.testing.expect(host_connected and client_connected);
    try std.testing.expect(host_ended and client_ended);
    try std.testing.expect(peer_disconnected);
    try std.testing.expect(!failure_received);
}

test "immediate cancellation does not strand host or client workers" {
    const alloc = std.testing.allocator;
    try checkIfLoopbackTestIsEnabled();

    for (0..8) |_| {
        var cancelled_host = SessionManager.init(alloc, std.testing.io);
        try startLoopbackHost(&cancelled_host, alloc, "cancelled.nes");
        cancelled_host.cancel();
        cancelled_host.deinit();

        var host = SessionManager.init(alloc, std.testing.io);
        defer host.deinit();
        try startLoopbackHost(&host, alloc, "client-cancel.nes");

        const deadline = milliTimestamp(std.testing.io) + 15_000;
        const code = try waitForSessionCode(&host, deadline);
        defer alloc.free(code);

        var client = SessionManager.init(alloc, std.testing.io);
        try client.connectWithPreset(code, .minimal);
        client.cancel();
        client.deinit();
        host.cancel();
    }
}
