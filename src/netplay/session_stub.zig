const std = @import("std");
const protocol = @import("protocol.zig");
const Ref = @import("../utils/types.zig").Ref;

pub const MessageRef = Ref(protocol.Message);
pub const PreviewRef = Ref(protocol.Preview);
pub const BytesRef = Ref([]u8);

pub const Role = enum { none, host, client };
pub const ConnectionRoute = enum(u32) { unknown, direct, relay, custom };
pub const ConnectionStats = struct {
    has_selected_path: bool = false,
    route: ConnectionRoute = .unknown,
    rtt_ms: u64 = 0,
    udp_tx_datagrams: u64 = 0,
    udp_tx_bytes: u64 = 0,
    udp_rx_datagrams: u64 = 0,
    udp_rx_bytes: u64 = 0,
    lost_packets: u64 = 0,
    lost_bytes: u64 = 0,
};

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
    join_requested,
    peer_disconnected,
    disconnected,
    failed: []u8,

    pub fn deinit(self: *Event, alloc: std.mem.Allocator) void {
        switch (self.*) {
            .session_code, .failed => |value| alloc.free(value),
            .preview => |*value| {
                alloc.free(value.name);
                alloc.free(value.framebuffer);
            },
            .message => |*value| value.deinit(alloc),
            else => {},
        }
    }

    pub fn takeSessionCode(self: *Event) BytesRef.Owned {
        const value = self.session_code;
        self.* = .{ .state = .idle };
        return .init(value);
    }

    pub fn takePreview(self: *Event) PreviewRef.Owned {
        const value = self.preview;
        self.* = .{ .state = .idle };
        return .init(value);
    }
};
pub const EventRef = Ref(Event);

/// Compile-time-compatible transport used when netplay is disabled. It owns no
/// threads, sockets, queues, or external runtime state.
pub const SessionManager = struct {
    pub fn init(_: std.mem.Allocator, _: std.Io) @This() {
        return .{};
    }

    pub fn deinit(_: *@This()) void {}
    pub fn getRole(_: *@This()) Role {
        return .none;
    }
    pub fn getState(_: *@This()) State {
        return .idle;
    }
    pub fn getConnectionStats(_: *@This()) !?ConnectionStats {
        return null;
    }
    pub fn isActive(_: *@This()) bool {
        return false;
    }

    pub fn startHost(_: *@This(), preview: PreviewRef.Owned) !void {
        _ = preview;
        return error.NetplayDisabled;
    }

    pub fn connect(_: *@This(), _: []const u8) !void {
        return error.NetplayDisabled;
    }
    pub fn acceptPreview(_: *@This()) !void {
        return error.NetplayDisabled;
    }
    pub fn send(_: *@This(), _: MessageRef.Borrowed) !void {
        return error.NetplayDisabled;
    }
    pub fn pollEvent(_: *@This()) ?EventRef.Owned {
        return null;
    }
    pub fn cancel(_: *@This()) void {}
    pub fn disconnect(_: *@This()) void {}
    pub fn markResyncing(_: *@This()) void {}
    pub fn markConnected(_: *@This()) void {}
};
