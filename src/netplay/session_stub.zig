const std = @import("std");
const types = @import("types.zig");

pub const MessageRef = types.MessageRef;
pub const PreviewRef = types.PreviewRef;
pub const BytesRef = types.BytesRef;
pub const Role = types.Role;
pub const State = types.State;
pub const Event = types.Event;
pub const EventRef = types.EventRef;

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
    pub fn send(_: *@This(), _: MessageRef.Borrowed) void {}
    pub fn pollEvent(_: *@This()) ?EventRef.Owned {
        return null;
    }
    pub fn cancel(_: *@This()) void {}
    pub fn disconnect(_: *@This()) void {}
    pub fn markResyncing(_: *@This()) void {}
    pub fn markConnected(_: *@This()) void {}
};
