const std = @import("std");
const iroh = @import("iroh");
const protocol = @import("protocol.zig");

pub const Role = enum { none, host, client };
pub const State = enum {
    idle,
    creating,
    waiting,
    connecting,
    preview,
    joining,
    connected,
    resyncing,
    disconnecting,
    failed,
};

pub const Event = union(enum) {
    state: State,
    session_code: []u8,
    preview: protocol.Preview,
    peer: [32]u8,
    message: protocol.Message,
    join_requested: void,
    disconnected: void,
    failed: []u8,

    pub fn deinit(self: *Event, alloc: std.mem.Allocator) void {
        switch (self.*) {
            .session_code, .failed => |value| alloc.free(value),
            .preview => |*value| alloc.free(value.name),
            .message => |*value| value.deinit(alloc),
            else => {},
        }
        self.* = undefined;
    }
};

const max_queue_items = 128;

/// Owns the complete blocking iroh lifecycle. UI and emulation code interact
/// only through bounded queues and never call the FFI directly.
pub const SessionManager = struct {
    alloc: std.mem.Allocator,
    mutex: std.Thread.Mutex = .{},
    wake: std.Thread.Condition = .{},
    role: Role = .none,
    state: State = .idle,
    cancelled: bool = false,
    graceful_shutdown: bool = false,
    worker: ?std.Thread = null,
    shutdown_worker: ?std.Thread = null,
    timeout_worker: ?std.Thread = null,
    timeout_state: State = .connecting,
    endpoint: ?iroh.Endpoint = null,
    connection: ?iroh.Connection = null,
    outgoing: std.ArrayList([]u8) = .empty,
    events: std.ArrayList(Event) = .empty,
    host_preview: ?protocol.Preview = null,
    connect_ticket: ?[]u8 = null,
    preset: iroh.Preset = .n0,

    const Self = @This();

    pub fn init(alloc: std.mem.Allocator) Self {
        std.log.debug("netplay: initializing session manager", .{});
        return .{ .alloc = alloc };
    }

    pub fn deinit(self: *Self) void {
        std.log.debug("netplay: deinitializing session manager", .{});
        self.cancel();
        if (self.worker) |thread| thread.join();
        if (self.shutdown_worker) |thread| thread.join();
        if (self.timeout_worker) |thread| thread.join();
        self.mutex.lock();
        self.clearOwnedLocked();
        self.outgoing.deinit(self.alloc);
        self.events.deinit(self.alloc);
        self.mutex.unlock();
        self.* = undefined;
    }

    pub fn getRole(self: *Self) Role {
        self.mutex.lock();
        defer self.mutex.unlock();
        return self.role;
    }

    pub fn getState(self: *Self) State {
        self.mutex.lock();
        defer self.mutex.unlock();
        return self.state;
    }

    pub fn isActive(self: *Self) bool {
        return self.getRole() != .none;
    }

    pub fn startHost(self: *Self, preview: protocol.Preview) !void {
        try self.startHostWithPreset(preview, .n0);
    }

    pub fn startHostWithPreset(self: *Self, preview: protocol.Preview, preset: iroh.Preset) !void {
        std.log.info("netplay: host requested for ROM '{s}' ({d} bytes), endpoint preset={s}", .{
            preview.name,
            preview.rom_size,
            @tagName(preset),
        });
        self.reapFinished();
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.role != .none) return error.SessionAlreadyActive;
        try verifyAbi();
        self.clearOwnedLocked();
        self.role = .host;
        self.state = .creating;
        self.cancelled = false;
        self.graceful_shutdown = false;
        self.preset = preset;
        self.host_preview = try clonePreview(self.alloc, preview);
        errdefer {
            self.alloc.free(self.host_preview.?.name);
            self.host_preview = null;
            self.role = .none;
            self.state = .idle;
        }
        try self.pushEventLocked(.{ .state = .creating });
        self.worker = try std.Thread.spawn(.{}, hostMain, .{self});
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
        std.log.info("netplay: client connection requested (code_length={d}, ticket_length={d}, endpoint preset={s})", .{
            std.mem.trim(u8, code, " \t\r\n").len,
            ticket.len,
            @tagName(preset),
        });
        self.reapFinished();
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.role != .none) return error.SessionAlreadyActive;
        try verifyAbi();
        self.clearOwnedLocked();
        self.role = .client;
        self.state = .connecting;
        self.cancelled = false;
        self.graceful_shutdown = false;
        self.preset = preset;
        self.connect_ticket = try self.alloc.dupe(u8, ticket);
        errdefer {
            self.alloc.free(self.connect_ticket.?);
            self.connect_ticket = null;
            self.role = .none;
            self.state = .idle;
        }
        try self.pushEventLocked(.{ .state = .connecting });
        self.worker = try std.Thread.spawn(.{}, clientMain, .{self});
        self.timeout_state = .connecting;
        self.timeout_worker = std.Thread.spawn(.{}, timeoutMain, .{self}) catch |err| blk: {
            std.log.err("netplay: failed to start setup-timeout worker: {s}", .{@errorName(err)});
            break :blk null;
        };
        std.log.debug("netplay: client and setup-timeout workers started", .{});
    }

    pub fn acceptPreview(self: *Self) !void {
        if (self.getState() != .preview) return error.InvalidSessionState;
        std.log.info("netplay: client accepted preview and requested to join", .{});
        try self.send(.{ .join = {} });
        self.setState(.joining);
        self.restartTimeout(.joining);
    }

    pub fn send(self: *Self, message: protocol.Message) !void {
        const encoded = protocol.encode(self.alloc, message) catch |err| {
            std.log.err("netplay: failed to encode outgoing {s} message: {s}", .{ @tagName(message), @errorName(err) });
            return err;
        };
        errdefer self.alloc.free(encoded);
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.cancelled or self.role == .none) return error.SessionClosed;
        if (self.outgoing.items.len >= max_queue_items) {
            std.log.err("netplay: outgoing queue full; cannot queue {s} message (role={s}, capacity={d})", .{
                @tagName(message),
                @tagName(self.role),
                max_queue_items,
            });
            return error.OutgoingQueueFull;
        }
        try self.outgoing.append(self.alloc, encoded);
        logQueuedMessage(self.role, message, self.outgoing.items.len, encoded.len);
        self.wake.signal();
    }

    pub fn pollEvent(self: *Self) ?Event {
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.events.items.len == 0) return null;
        return self.events.orderedRemove(0);
    }

    pub fn cancel(self: *Self) void {
        self.mutex.lock();
        if (self.role == .none or self.cancelled) {
            self.mutex.unlock();
            return;
        }
        std.log.info("netplay: cancelling {s} session from state {s}", .{ @tagName(self.role), @tagName(self.state) });
        self.cancelled = true;
        if (self.state != .failed) {
            self.state = .disconnecting;
            self.pushEventLocked(.{ .state = .disconnecting }) catch {};
        }
        self.wake.broadcast();
        if (self.shutdown_worker == null) {
            self.shutdown_worker = std.Thread.spawn(.{}, shutdownMain, .{self}) catch |err| blk: {
                std.log.err("netplay: failed to start shutdown worker: {s}", .{@errorName(err)});
                break :blk null;
            };
        }
        self.mutex.unlock();
    }

    pub fn disconnect(self: *Self) void {
        std.log.info("netplay: graceful disconnect requested", .{});
        var notice_queued = true;
        self.send(.{ .disconnect = @constCast("peer left the session") }) catch |err| {
            notice_queued = false;
            std.log.warn("netplay: could not queue disconnect notice: {s}", .{@errorName(err)});
        };
        self.mutex.lock();
        self.graceful_shutdown = true;
        // During preview the client worker is waiting on the outgoing queue
        // and owns the stream. Let it send the decline before closing the
        // transport; finishWorker will complete the normal cleanup.
        if (notice_queued and self.role == .client and self.state == .preview and !self.cancelled) {
            std.log.info("netplay: client declining session preview", .{});
            self.state = .disconnecting;
            self.pushEventLocked(.{ .state = .disconnecting }) catch {};
            self.wake.broadcast();
            self.mutex.unlock();
            return;
        }
        self.mutex.unlock();
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

    fn hostMain(self: *Self) void {
        std.log.debug("netplay: host worker entered", .{});
        self.runHost() catch |err| self.reportFailure(err);
        self.finishWorker();
    }

    fn runHost(self: *Self) !void {
        std.log.info("netplay: host binding endpoint (preset={s}, ALPN={s})", .{ @tagName(self.preset), protocol.alpn });
        var endpoint = try iroh.Endpoint.bind(self.alloc, .{ .preset = self.preset, .alpns = &.{protocol.alpn} });
        defer endpoint.deinit();
        std.log.info("netplay: host endpoint bound", .{});
        try self.publishEndpoint(endpoint);
        defer self.joinShutdown();
        const ticket = try endpoint.ticket(self.alloc);
        defer self.alloc.free(ticket);
        const code = try protocol.makeSessionCode(self.alloc, ticket);
        std.log.info("netplay: session code generated (length={d}); waiting for one client", .{code.len});
        try self.pushEvent(.{ .session_code = code });
        self.setState(.waiting);

        var connection = try endpoint.accept();
        defer connection.deinit();
        std.log.info("netplay: host accepted incoming connection", .{});
        try self.publishConnection(connection);
        const remote = try connection.remoteId();
        std.log.info("netplay: host connected to peer {x}", .{remote.bytes[0..8]});
        try self.pushEvent(.{ .peer = remote.bytes });
        std.log.debug("netplay: host opening bidirectional stream", .{});
        var stream = try connection.openBi();
        std.log.debug("netplay: host opened bidirectional stream", .{});
        defer stream.deinit();
        defer self.joinShutdown();

        const preview = self.takeHostPreview() orelse return error.MissingPreview;
        defer self.alloc.free(preview.name);
        const encoded_preview = try protocol.encode(self.alloc, .{ .preview = preview });
        defer self.alloc.free(encoded_preview);
        std.log.info("netplay: host sending preview for '{s}' (rom={d} bytes, framebuffer={d} bytes, framed={d} bytes)", .{
            preview.name,
            preview.rom_size,
            protocol.framebuffer_size,
            encoded_preview.len,
        });
        try stream.send.writeAll(encoded_preview);
        var join = try recvMessage(self.alloc, stream.recv);
        defer join.deinit(self.alloc);
        switch (join) {
            .join => {},
            .disconnect => |reason| {
                std.log.info("netplay: client declined session preview: {s}", .{reason});
                return;
            },
            else => return error.UnexpectedHandshakeMessage,
        }
        std.log.info("netplay: host received join request", .{});
        try self.pushEvent(.{ .join_requested = {} });
        self.setState(.joining);

        const join_data = try self.waitOutgoing();
        defer self.alloc.free(join_data);
        std.log.info("netplay: host sending ROM and snapshot transfer ({d} framed bytes)", .{join_data.len});
        try stream.send.writeAll(join_data);
        var ready = try recvMessage(self.alloc, stream.recv);
        var ready_owned = true;
        defer if (ready_owned) ready.deinit(self.alloc);
        if (ready != .ready) return error.UnexpectedHandshakeMessage;
        std.log.info("netplay: host received client ready (epoch={d}, frame={d})", .{ ready.ready.epoch, ready.ready.frame });
        try self.pushEvent(.{ .message = ready });
        ready_owned = false;
        self.setState(.connected);
        try self.runConnected(stream.send, stream.recv);
    }

    fn clientMain(self: *Self) void {
        std.log.debug("netplay: client worker entered", .{});
        self.runClient() catch |err| self.reportFailure(err);
        self.finishWorker();
    }

    fn runClient(self: *Self) !void {
        std.log.info("netplay: client binding local endpoint (preset={s})", .{@tagName(self.preset)});
        var endpoint = try iroh.Endpoint.bind(self.alloc, .{ .preset = self.preset });
        defer endpoint.deinit();
        std.log.info("netplay: client endpoint bound", .{});
        try self.publishEndpoint(endpoint);
        defer self.joinShutdown();
        const ticket = self.takeConnectTicket() orelse return error.MissingTicket;
        defer self.alloc.free(ticket);
        std.log.info("netplay: client connecting to host (ticket_length={d}, ALPN={s})", .{ ticket.len, protocol.alpn });
        var connection = try endpoint.connect(ticket, protocol.alpn);
        defer connection.deinit();
        std.log.info("netplay: client connection established", .{});
        try self.publishConnection(connection);
        const remote = try connection.remoteId();
        std.log.info("netplay: client connected to peer {x}", .{remote.bytes[0..8]});
        try self.pushEvent(.{ .peer = remote.bytes });
        std.log.debug("netplay: client waiting for bidirectional stream", .{});
        var stream = try connection.acceptBi();
        std.log.debug("netplay: client accepted bidirectional stream", .{});
        defer stream.deinit();
        defer self.joinShutdown();

        var preview_message = try recvMessage(self.alloc, stream.recv);
        if (preview_message != .preview) {
            preview_message.deinit(self.alloc);
            return error.UnexpectedHandshakeMessage;
        }
        const preview = preview_message.preview;
        preview_message = undefined;
        std.log.info("netplay: client received preview for '{s}' ({d} bytes)", .{ preview.name, preview.rom_size });
        try self.pushEvent(.{ .preview = preview });
        self.setState(.preview);

        const response_bytes = try self.waitOutgoing();
        defer self.alloc.free(response_bytes);
        var response = try protocol.decode(self.alloc, response_bytes);
        defer response.deinit(self.alloc);
        switch (response) {
            .join => {
                std.log.debug("netplay: client sending join request", .{});
                try stream.send.writeAll(response_bytes);
            },
            .disconnect => |reason| {
                std.log.info("netplay: client sending preview decline: {s}", .{reason});
                stream.send.writeAll(response_bytes) catch |err| {
                    std.log.debug("netplay: preview decline send interrupted by shutdown: {s}", .{@errorName(err)});
                };
                return;
            },
            else => return error.UnexpectedHandshakeMessage,
        }
        self.setState(.joining);
        var join_data = try recvMessage(self.alloc, stream.recv);
        if (join_data != .join_data) {
            join_data.deinit(self.alloc);
            return error.UnexpectedHandshakeMessage;
        }
        std.log.info("netplay: client received ROM and snapshot (rom={d} bytes, snapshot={d} bytes, epoch={d}, frame={d})", .{
            join_data.join_data.rom.len,
            join_data.join_data.snapshot.len,
            join_data.join_data.epoch,
            join_data.join_data.frame,
        });
        try self.pushEvent(.{ .message = join_data });
        join_data = undefined;

        const ready = try self.waitOutgoing();
        defer self.alloc.free(ready);
        std.log.debug("netplay: client sending ready acknowledgement", .{});
        try stream.send.writeAll(ready);
        self.setState(.connected);
        try self.runConnected(stream.send, stream.recv);
    }

    fn runConnected(self: *Self, send_stream: iroh.SendStream, recv_stream: iroh.RecvStream) !void {
        std.log.info("netplay: connected message loop started for {s}", .{@tagName(self.getRole())});
        const sender = try std.Thread.spawn(.{}, sendMain, .{ self, send_stream });
        defer {
            self.mutex.lock();
            self.cancelled = true;
            self.wake.broadcast();
            self.mutex.unlock();
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
            if (message == .disconnect) {
                std.log.info("netplay: peer requested disconnect: {s}", .{message.disconnect});
                message.deinit(self.alloc);
                return;
            }
            logReceivedMessage(self.getRole(), message);
            self.pushEvent(.{ .message = message }) catch {
                message.deinit(self.alloc);
                return error.EventQueueFull;
            };
        }
    }

    fn sendMain(self: *Self, stream: iroh.SendStream) void {
        std.log.debug("netplay: sender worker started", .{});
        while (self.waitOutgoing()) |encoded| {
            defer self.alloc.free(encoded);
            stream.writeAll(encoded) catch |err| {
                std.log.err("netplay: stream write failed: {s}", .{@errorName(err)});
                self.reportFailure(err);
                self.cancel();
                return;
            };
        } else |err| {
            std.log.debug("netplay: sender worker stopped: {s}", .{@errorName(err)});
        }
    }

    fn waitOutgoing(self: *Self) ![]u8 {
        self.mutex.lock();
        defer self.mutex.unlock();
        while (self.outgoing.items.len == 0 and !self.cancelled) self.wake.wait(&self.mutex);
        if (self.outgoing.items.len != 0) return self.outgoing.orderedRemove(0);
        return error.SessionClosed;
    }

    fn setState(self: *Self, state: State) void {
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.cancelled and state != .failed and state != .idle) return;
        const previous = self.state;
        self.state = state;
        if (previous != state) std.log.info("netplay: state {s} -> {s} (role={s})", .{
            @tagName(previous),
            @tagName(state),
            @tagName(self.role),
        });
        self.pushEventLocked(.{ .state = state }) catch {};
    }

    fn pushEvent(self: *Self, event: Event) !void {
        self.mutex.lock();
        defer self.mutex.unlock();
        try self.pushEventLocked(event);
    }

    fn pushEventLocked(self: *Self, event: Event) !void {
        if (self.events.items.len >= max_queue_items) {
            std.log.err("netplay: application event queue full; cannot queue {s} event (capacity={d})", .{
                @tagName(event),
                max_queue_items,
            });
            return error.EventQueueFull;
        }
        self.events.append(self.alloc, event) catch |err| {
            std.log.err("netplay: failed to queue {s} application event: {s}", .{ @tagName(event), @errorName(err) });
            return err;
        };
    }

    fn reportFailure(self: *Self, err: anyerror) void {
        if (self.isCancelled()) return;
        const detail = iroh.lastErrorMessage();
        if (detail.len != 0) {
            std.log.err("netplay: session failed: {s}; iroh: {s}", .{ @errorName(err), detail });
        } else {
            std.log.err("netplay: session failed: {s}", .{@errorName(err)});
        }
        const message = if (detail.len != 0)
            std.fmt.allocPrint(self.alloc, "{s}: {s}", .{ @errorName(err), detail })
        else
            self.alloc.dupe(u8, @errorName(err));
        const owned = message catch |alloc_err| {
            std.log.err("netplay: failed to allocate failure detail: {s}", .{@errorName(alloc_err)});
            return;
        };
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.cancelled) {
            self.alloc.free(owned);
            return;
        }
        self.state = .failed;
        self.pushEventLocked(.{ .state = .failed }) catch {};
        self.pushEventLocked(.{ .failed = owned }) catch self.alloc.free(owned);
    }

    fn finishWorker(self: *Self) void {
        self.mutex.lock();
        self.endpoint = null;
        self.connection = null;
        self.cancelled = true;
        self.wake.broadcast();
        const failed = self.state == .failed;
        const finished_role = self.role;
        self.role = .none;
        if (!failed) self.state = .idle;
        self.pushEventLocked(.{ .disconnected = {} }) catch {};
        if (!failed) self.pushEventLocked(.{ .state = .idle }) catch {};
        self.mutex.unlock();
        std.log.info("netplay: {s} worker finished (failed={any})", .{ @tagName(finished_role), failed });
    }

    fn shutdownMain(self: *Self) void {
        self.mutex.lock();
        const endpoint = self.endpoint;
        const connection = self.connection;
        const graceful = self.graceful_shutdown;
        self.mutex.unlock();
        std.log.debug("netplay: shutdown worker closing connection and endpoint (graceful={any})", .{graceful});
        if (graceful) std.Thread.sleep(20 * std.time.ns_per_ms);
        if (connection) |value| value.close(0, "session shutdown") catch |err| {
            std.log.warn("netplay: connection close failed: {s}", .{@errorName(err)});
        };
        if (endpoint) |value| value.close() catch |err| {
            std.log.warn("netplay: endpoint close failed: {s}", .{@errorName(err)});
        };
        std.log.debug("netplay: shutdown worker finished", .{});
    }

    fn timeoutMain(self: *Self) void {
        self.mutex.lock();
        const armed_state = self.timeout_state;
        self.mutex.unlock();
        std.log.debug("netplay: setup timeout armed for state {s} (30 seconds)", .{@tagName(armed_state)});
        const deadline = std.time.milliTimestamp() + 30_000;
        while (std.time.milliTimestamp() < deadline) {
            self.mutex.lock();
            const pending = self.role == .client and self.state == self.timeout_state and !self.cancelled;
            self.mutex.unlock();
            if (!pending) return;
            std.Thread.sleep(10 * std.time.ns_per_ms);
        }
        self.mutex.lock();
        if (self.role != .client or self.state != self.timeout_state or self.cancelled) {
            self.mutex.unlock();
            return;
        }
        self.cancelled = true;
        self.state = .failed;
        std.log.err("netplay: client setup timed out in state {s} after 30 seconds", .{@tagName(self.timeout_state)});
        self.pushEventLocked(.{ .state = .failed }) catch {};
        const message = self.alloc.dupe(u8, "Connection/setup timed out after 30 seconds") catch |err| blk: {
            std.log.err("netplay: failed to allocate timeout error detail: {s}", .{@errorName(err)});
            break :blk null;
        };
        if (message) |value| self.pushEventLocked(.{ .failed = value }) catch self.alloc.free(value);
        self.wake.broadcast();
        if (self.shutdown_worker == null) self.shutdown_worker = std.Thread.spawn(.{}, shutdownMain, .{self}) catch |err| blk: {
            std.log.err("netplay: failed to start timeout shutdown worker: {s}", .{@errorName(err)});
            break :blk null;
        };
        self.mutex.unlock();
    }

    fn joinShutdown(self: *Self) void {
        self.mutex.lock();
        const thread = self.shutdown_worker;
        self.shutdown_worker = null;
        self.mutex.unlock();
        if (thread) |value| value.join();
    }

    fn restartTimeout(self: *Self, state: State) void {
        self.mutex.lock();
        const old = self.timeout_worker;
        self.timeout_worker = null;
        self.mutex.unlock();
        if (old) |thread| thread.join();
        self.mutex.lock();
        if (!self.cancelled and self.role == .client and self.state == state) {
            self.timeout_state = state;
            self.timeout_worker = std.Thread.spawn(.{}, timeoutMain, .{self}) catch |err| blk: {
                std.log.err("netplay: failed to restart setup-timeout worker: {s}", .{@errorName(err)});
                break :blk null;
            };
        }
        self.mutex.unlock();
    }

    fn reapFinished(self: *Self) void {
        self.mutex.lock();
        if (self.role != .none) {
            self.mutex.unlock();
            return;
        }
        const worker = self.worker;
        const shutdown = self.shutdown_worker;
        const timeout = self.timeout_worker;
        self.worker = null;
        self.shutdown_worker = null;
        self.timeout_worker = null;
        self.mutex.unlock();
        if (worker) |thread| thread.join();
        if (shutdown) |thread| thread.join();
        if (timeout) |thread| thread.join();
    }

    fn publishEndpoint(self: *Self, endpoint: iroh.Endpoint) !void {
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.cancelled) return error.SessionClosed;
        self.endpoint = endpoint;
    }

    fn publishConnection(self: *Self, connection: iroh.Connection) !void {
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.cancelled) return error.SessionClosed;
        self.connection = connection;
    }

    fn isCancelled(self: *Self) bool {
        self.mutex.lock();
        defer self.mutex.unlock();
        return self.cancelled;
    }

    fn takeHostPreview(self: *Self) ?protocol.Preview {
        self.mutex.lock();
        defer self.mutex.unlock();
        const result = self.host_preview;
        self.host_preview = null;
        return result;
    }

    fn takeConnectTicket(self: *Self) ?[]u8 {
        self.mutex.lock();
        defer self.mutex.unlock();
        const result = self.connect_ticket;
        self.connect_ticket = null;
        return result;
    }

    fn clearOwnedLocked(self: *Self) void {
        if (self.host_preview) |preview| self.alloc.free(preview.name);
        self.host_preview = null;
        if (self.connect_ticket) |ticket| self.alloc.free(ticket);
        self.connect_ticket = null;
        for (self.outgoing.items) |encoded| self.alloc.free(encoded);
        self.outgoing.clearRetainingCapacity();
        for (self.events.items) |*event| event.deinit(self.alloc);
        self.events.clearRetainingCapacity();
    }
};

fn clonePreview(alloc: std.mem.Allocator, value: protocol.Preview) !protocol.Preview {
    return .{
        .name = try alloc.dupe(u8, value.name),
        .rom_size = value.rom_size,
        .rom_hash = value.rom_hash,
        .framebuffer = value.framebuffer,
    };
}

fn verifyAbi() !void {
    const actual = iroh.runtimeAbiVersion();
    if (actual != iroh.abi_version) {
        std.log.err("netplay: incompatible iroh ABI (runtime={d}, bindings={d})", .{ actual, iroh.abi_version });
        return error.IncompatibleIrohAbi;
    }
    std.log.debug("netplay: verified iroh ABI version {d}", .{actual});
}

fn logQueuedMessage(role: Role, message: protocol.Message, queue_len: usize, encoded_len: usize) void {
    if (queue_len >= max_queue_items * 3 / 4) {
        std.log.warn("netplay: outgoing queue pressure is high ({d}/{d}, role={s})", .{
            queue_len,
            max_queue_items,
            @tagName(role),
        });
    }
    switch (message) {
        .preview => |value| std.log.debug("netplay: queued preview (role={s}, name='{s}', rom_size={d})", .{
            @tagName(role), value.name, value.rom_size,
        }),
        .join => std.log.debug("netplay: queued join request (role={s})", .{@tagName(role)}),
        .join_data => |value| std.log.info("netplay: queued join transfer (role={s}, rom={d} bytes, snapshot={d} bytes, framed={d} bytes, epoch={d}, frame={d})", .{
            @tagName(role), value.rom.len, value.snapshot.len, encoded_len, value.epoch, value.frame,
        }),
        .ready => |value| std.log.info("netplay: queued ready (role={s}, epoch={d}, frame={d})", .{
            @tagName(role), value.epoch, value.frame,
        }),
        .frame => |value| if (value.digest != null) {
            std.log.debug("netplay: queued checkpoint frame (epoch={d}, frame={d}, queue={d})", .{
                value.epoch, value.frame, queue_len,
            });
        },
        .ack => |value| if (value.digest != null) {
            std.log.debug("netplay: queued checkpoint acknowledgement (epoch={d}, frame={d}, queue={d})", .{
                value.epoch, value.frame, queue_len,
            });
        },
        .control => |value| switch (value) {
            .paused => |paused| std.log.info("netplay: queued pause control (paused={any})", .{paused}),
            .speed => |speed| std.log.info("netplay: queued speed control (value={d})", .{speed}),
        },
        .rebase => |value| std.log.info("netplay: queued rebase (epoch={d}, frame={d}, snapshot={d} bytes)", .{
            value.epoch, value.frame, value.snapshot.len,
        }),
        .disconnect => |reason| std.log.info("netplay: queued disconnect notice: {s}", .{reason}),
    }
}

fn logReceivedMessage(role: Role, message: protocol.Message) void {
    switch (message) {
        .ready => |value| std.log.info("netplay: received ready (role={s}, epoch={d}, frame={d})", .{
            @tagName(role), value.epoch, value.frame,
        }),
        .frame => |value| if (value.digest != null) {
            std.log.debug("netplay: received checkpoint frame (epoch={d}, frame={d})", .{ value.epoch, value.frame });
        },
        .ack => |value| if (value.digest != null) {
            std.log.debug("netplay: received checkpoint acknowledgement (epoch={d}, frame={d})", .{ value.epoch, value.frame });
        },
        .control => |value| switch (value) {
            .paused => |paused| std.log.info("netplay: received pause control (paused={any})", .{paused}),
            .speed => |speed| std.log.info("netplay: received speed control (value={d})", .{speed}),
        },
        .rebase => |value| std.log.info("netplay: received rebase (epoch={d}, frame={d}, snapshot={d} bytes)", .{
            value.epoch, value.frame, value.snapshot.len,
        }),
        else => std.log.debug("netplay: received {s} message (role={s})", .{ @tagName(message), @tagName(role) }),
    }
}

fn recvMessage(alloc: std.mem.Allocator, stream: iroh.RecvStream) !protocol.Message {
    const header = try stream.readExact(alloc, 4);
    defer alloc.free(header);
    const len = std.mem.readInt(u32, header[0..4], .little);
    if (len == 0 or len > protocol.max_message_size) return error.MessageTooLarge;
    const body = try stream.readExact(alloc, len);
    defer alloc.free(body);
    const framed = try alloc.alloc(u8, 4 + @as(usize, len));
    defer alloc.free(framed);
    @memcpy(framed[0..4], header);
    @memcpy(framed[4..], body);
    return protocol.decode(alloc, framed);
}

test "session state starts idle and validates codes synchronously" {
    var manager = SessionManager.init(std.testing.allocator);
    defer manager.deinit();
    try std.testing.expectEqual(Role.none, manager.getRole());
    try std.testing.expectEqual(State.idle, manager.getState());
    try std.testing.expectError(error.InvalidSessionCode, manager.connect("bad-code"));
}

test "local loopback session" {
    const alloc = std.testing.allocator;
    const enabled = std.process.getEnvVarOwned(alloc, "NESKWIK_NETPLAY_LOOPBACK_TEST") catch
        return error.SkipZigTest;
    defer alloc.free(enabled);
    if (!std.mem.eql(u8, enabled, "1")) return error.SkipZigTest;

    var host = SessionManager.init(alloc);
    defer host.deinit();
    var client = SessionManager.init(alloc);
    defer client.deinit();
    const framebuffer = [_]u8{0x22} ** protocol.framebuffer_size;
    try host.startHostWithPreset(.{
        .name = @constCast("loopback.nes"),
        .rom_size = 3,
        .rom_hash = [_]u8{0x11} ** 32,
        .framebuffer = framebuffer,
    }, .minimal);

    var code: ?[]u8 = null;
    const deadline = std.time.milliTimestamp() + 15_000;
    while (code == null and std.time.milliTimestamp() < deadline) {
        if (host.pollEvent()) |event_value| {
            var event = event_value;
            defer event.deinit(alloc);
            if (event == .session_code) code = try alloc.dupe(u8, event.session_code);
        } else std.Thread.sleep(std.time.ns_per_ms);
    }
    defer if (code) |value| alloc.free(value);
    try std.testing.expect(code != null);
    try client.connectWithPreset(code.?, .minimal);

    var host_connected = false;
    var client_connected = false;
    var frame_received = false;
    var ack_received = false;
    while (!ack_received and std.time.milliTimestamp() < deadline) {
        while (host.pollEvent()) |event_value| {
            var event = event_value;
            defer event.deinit(alloc);
            switch (event) {
                .join_requested => try host.send(.{ .join_data = .{
                    .name = @constCast("loopback.nes"),
                    .rom = @constCast("rom"),
                    .rom_hash = [_]u8{0x11} ** 32,
                    .snapshot = @constCast("state"),
                    .speed = 1,
                    .epoch = 1,
                    .frame = 0,
                } }),
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
            defer event.deinit(alloc);
            switch (event) {
                .preview => try client.acceptPreview(),
                .message => |message| switch (message) {
                    .join_data => |data| try client.send(.{ .ready = .{ .epoch = data.epoch, .frame = data.frame, .player2 = 7 } }),
                    .frame => |frame| {
                        frame_received = true;
                        try client.send(.{ .ack = .{ .epoch = frame.epoch, .frame = frame.frame, .player2 = 7 } });
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
            try host.send(.{ .frame = .{ .epoch = 1, .frame = 1, .player1 = 1, .player2 = 7 } });
        }
        std.Thread.sleep(std.time.ns_per_ms);
    }
    try std.testing.expect(host_connected and client_connected and frame_received and ack_received);
    host.disconnect();
}

test "local loopback session preview decline" {
    const alloc = std.testing.allocator;
    const enabled = std.process.getEnvVarOwned(alloc, "NESKWIK_NETPLAY_LOOPBACK_TEST") catch
        return error.SkipZigTest;
    defer alloc.free(enabled);
    if (!std.mem.eql(u8, enabled, "1")) return error.SkipZigTest;

    var host = SessionManager.init(alloc);
    defer host.deinit();
    var client = SessionManager.init(alloc);
    defer client.deinit();
    const framebuffer = [_]u8{0x22} ** protocol.framebuffer_size;
    try host.startHostWithPreset(.{
        .name = @constCast("declined.nes"),
        .rom_size = 3,
        .rom_hash = [_]u8{0x11} ** 32,
        .framebuffer = framebuffer,
    }, .minimal);

    var code: ?[]u8 = null;
    const deadline = std.time.milliTimestamp() + 15_000;
    while (code == null and std.time.milliTimestamp() < deadline) {
        if (host.pollEvent()) |event_value| {
            var event = event_value;
            defer event.deinit(alloc);
            if (event == .session_code) code = try alloc.dupe(u8, event.session_code);
        } else std.Thread.sleep(std.time.ns_per_ms);
    }
    defer if (code) |value| alloc.free(value);
    try std.testing.expect(code != null);
    try client.connectWithPreset(code.?, .minimal);

    var preview_received = false;
    var host_ended = false;
    var client_ended = false;
    var failure_received = false;
    while ((!host_ended or !client_ended) and std.time.milliTimestamp() < deadline) {
        while (host.pollEvent()) |event_value| {
            var event = event_value;
            defer event.deinit(alloc);
            switch (event) {
                .failed => failure_received = true,
                .disconnected => host_ended = true,
                else => {},
            }
        }
        while (client.pollEvent()) |event_value| {
            var event = event_value;
            defer event.deinit(alloc);
            switch (event) {
                .preview => {
                    preview_received = true;
                    client.disconnect();
                },
                .failed => failure_received = true,
                .disconnected => client_ended = true,
                else => {},
            }
        }
        std.Thread.sleep(std.time.ns_per_ms);
    }

    try std.testing.expect(preview_received);
    try std.testing.expect(host_ended and client_ended);
    try std.testing.expect(!failure_received);
}

test "immediate cancellation does not strand host or client workers" {
    const alloc = std.testing.allocator;
    const enabled = std.process.getEnvVarOwned(alloc, "NESKWIK_NETPLAY_LOOPBACK_TEST") catch
        return error.SkipZigTest;
    defer alloc.free(enabled);
    if (!std.mem.eql(u8, enabled, "1")) return error.SkipZigTest;

    const framebuffer = [_]u8{0x22} ** protocol.framebuffer_size;
    for (0..8) |_| {
        var cancelled_host = SessionManager.init(alloc);
        try cancelled_host.startHostWithPreset(.{
            .name = @constCast("cancelled.nes"),
            .rom_size = 3,
            .rom_hash = [_]u8{0x11} ** 32,
            .framebuffer = framebuffer,
        }, .minimal);
        cancelled_host.cancel();
        cancelled_host.deinit();

        var host = SessionManager.init(alloc);
        defer host.deinit();
        try host.startHostWithPreset(.{
            .name = @constCast("client-cancel.nes"),
            .rom_size = 3,
            .rom_hash = [_]u8{0x11} ** 32,
            .framebuffer = framebuffer,
        }, .minimal);

        var code: ?[]u8 = null;
        const deadline = std.time.milliTimestamp() + 15_000;
        while (code == null and std.time.milliTimestamp() < deadline) {
            if (host.pollEvent()) |event_value| {
                var event = event_value;
                defer event.deinit(alloc);
                if (event == .session_code) code = try alloc.dupe(u8, event.session_code);
            } else std.Thread.sleep(std.time.ns_per_ms);
        }
        defer if (code) |value| alloc.free(value);
        try std.testing.expect(code != null);

        var client = SessionManager.init(alloc);
        try client.connectWithPreset(code.?, .minimal);
        client.cancel();
        client.deinit();
        host.cancel();
    }
}
