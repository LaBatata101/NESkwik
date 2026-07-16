const std = @import("std");

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

pub const SessionManager = struct {
    pub fn init(_: std.mem.Allocator) @This() {
        return .{};
    }
    pub fn deinit(_: *@This()) void {}
    pub fn cancel(_: *@This()) void {}
    pub fn disconnect(_: *@This()) void {}
    pub fn isActive(_: *@This()) bool {
        return false;
    }
    pub fn getRole(_: *@This()) Role {
        return .none;
    }
    pub fn getState(_: *@This()) State {
        return .idle;
    }
    pub fn markResyncing(_: *@This()) void {}
    pub fn markConnected(_: *@This()) void {}
};
