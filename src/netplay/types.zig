//! Session types shared by the iroh-backed session manager and the stub used
//! when netplay is disabled.

const std = @import("std");
const protocol = @import("protocol.zig");
const Ref = @import("../utils/types.zig").Ref;

pub const MessageRef = Ref(protocol.Message);
pub const PreviewRef = Ref(protocol.Preview);
pub const BytesRef = Ref([]u8);

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

    /// Payload events describe an ongoing session. Once the application has
    /// left the session only the lifecycle events still matter.
    pub fn isPayload(self: Event) bool {
        return switch (self) {
            .session_code, .preview, .peer, .message, .join_requested => true,
            .state, .peer_disconnected, .disconnected, .failed => false,
        };
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
