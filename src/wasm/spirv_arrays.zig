//! Rewrites stores of array-of-arrays values into per-element stores.
//!
//! GLSL ES 3.00 has no arrays of arrays. SPIRV-Cross flattens their
//! declarations and accesses (`FLATTEN_MULTIDIMENSIONAL_ARRAYS`), but refuses
//! values that construct a whole array of arrays, which shaders produce with
//! initializers like `vec3 mask[3][4] = { ... }` (crt-geom, crt-hyllian, ...).
//! Storing the elements one at a time leaves no such value behind.
const std = @import("std");

const magic: u32 = 0x07230203;
const header_len = 5;

const Op = struct {
    const name = 5;
    const type_int = 21;
    const type_array = 28;
    const type_pointer = 32;
    const constant = 43;
    const constant_composite = 44;
    const spec_constant_composite = 51;
    const function = 54;
    const function_parameter = 55;
    const function_call = 57;
    const variable = 59;
    const store = 62;
    const access_chain = 65;
    const in_bounds_access_chain = 66;
    const decorate = 71;
    const composite_construct = 80;
    const composite_extract = 81;
    const composite_insert = 82;
    const copy_object = 83;
    const select = 169;
    const phi = 245;
    const return_value = 254;
};

const Composite = struct {
    type: u32,
    /// Index of the defining instruction.
    inst: usize,
};

const Module = struct {
    alloc: std.mem.Allocator,
    words: []const u32,
    /// Word offset of every instruction.
    insts: std.ArrayList(usize) = .empty,
    bound: u32,
    first_function: ?usize = null,

    array_elem: std.AutoHashMapUnmanaged(u32, u32) = .empty,
    pointers: std.AutoHashMapUnmanaged(u32, [2]u32) = .empty,
    pointer_ids: std.AutoHashMapUnmanaged([2]u32, u32) = .empty,
    value_types: std.AutoHashMapUnmanaged(u32, u32) = .empty,
    composites: std.AutoHashMapUnmanaged(u32, Composite) = .empty,
    int_type: ?u32 = null,
    int_constants: std.AutoHashMapUnmanaged(u32, u32) = .empty,

    /// Declarations added before the first function.
    globals: std.ArrayList(u32) = .empty,

    fn deinit(self: *Module) void {
        self.insts.deinit(self.alloc);
        self.array_elem.deinit(self.alloc);
        self.pointers.deinit(self.alloc);
        self.pointer_ids.deinit(self.alloc);
        self.value_types.deinit(self.alloc);
        self.composites.deinit(self.alloc);
        self.int_constants.deinit(self.alloc);
        self.globals.deinit(self.alloc);
    }

    fn inst(self: *const Module, index: usize) []const u32 {
        const start = self.insts.items[index];
        return self.words[start..][0 .. self.words[start] >> 16];
    }

    fn isArrayOfArrays(self: *const Module, type_id: u32) bool {
        const elem = self.array_elem.get(type_id) orelse return false;
        return self.array_elem.contains(elem);
    }

    fn newId(self: *Module) u32 {
        defer self.bound += 1;
        return self.bound;
    }

    fn intType(self: *Module) !u32 {
        if (self.int_type) |id| return id;
        const id = self.newId();
        try self.globals.appendSlice(self.alloc, &.{ (4 << 16) | Op.type_int, id, 32, 1 });
        self.int_type = id;
        return id;
    }

    /// Indices are signed: SPIRV-Cross flattens `a[i][j]` to `a[i * n + j]`
    /// with an `int` stride, and GLSL ES does not mix `int` and `uint`.
    fn intConstant(self: *Module, value: u32) !u32 {
        if (self.int_constants.get(value)) |id| return id;
        const type_id = try self.intType();
        const id = self.newId();
        try self.globals.appendSlice(self.alloc, &.{ (4 << 16) | Op.constant, type_id, id, value });
        try self.int_constants.put(self.alloc, value, id);
        return id;
    }

    fn pointerType(self: *Module, storage: u32, pointee: u32) !u32 {
        if (self.pointer_ids.get(.{ storage, pointee })) |id| return id;
        const id = self.newId();
        try self.globals.appendSlice(self.alloc, &.{ (4 << 16) | Op.type_pointer, id, storage, pointee });
        try self.pointer_ids.put(self.alloc, .{ storage, pointee }, id);
        return id;
    }

    fn scan(self: *Module) !void {
        var offset: usize = header_len;
        while (offset < self.words.len) {
            const len = self.words[offset] >> 16;
            if (len == 0 or offset + len > self.words.len) return error.InvalidSpirv;
            const w = self.words[offset..][0..len];
            const index = self.insts.items.len;
            try self.insts.append(self.alloc, offset);

            switch (w[0] & 0xFFFF) {
                Op.type_int => if (len == 4 and w[2] == 32 and w[3] == 1 and self.int_type == null) {
                    self.int_type = w[1];
                },
                Op.type_array => if (len == 4) try self.array_elem.put(self.alloc, w[1], w[2]),
                Op.type_pointer => if (len == 4) {
                    try self.pointers.put(self.alloc, w[1], .{ w[2], w[3] });
                    try self.pointer_ids.put(self.alloc, .{ w[2], w[3] }, w[1]);
                },
                Op.constant => if (len == 4 and self.int_type != null and w[1] == self.int_type.?) {
                    try self.int_constants.put(self.alloc, w[3], w[2]);
                },
                Op.constant_composite, Op.composite_construct => if (len >= 3) {
                    try self.composites.put(self.alloc, w[2], .{ .type = w[1], .inst = index });
                    try self.value_types.put(self.alloc, w[2], w[1]);
                },
                Op.variable, Op.function_parameter, Op.access_chain, Op.in_bounds_access_chain => if (len >= 3) {
                    try self.value_types.put(self.alloc, w[2], w[1]);
                },
                Op.function => if (self.first_function == null) {
                    self.first_function = index;
                },
                else => {},
            }
            offset += len;
        }
    }

    /// Append stores of every element of `value` (of `type_id`) to `out`.
    /// `path` holds the indices from the stored variable down to `value`.
    fn storeElements(
        self: *Module,
        out: *std.ArrayList(u32),
        pointer: u32,
        storage: u32,
        value: u32,
        type_id: u32,
        path: *std.ArrayList(u32),
    ) !void {
        const elem_type = self.array_elem.get(type_id) orelse {
            // Leaf: store it through an access chain into the variable.
            const ptr_type = try self.pointerType(storage, type_id);
            const chain = self.newId();
            try out.appendSlice(self.alloc, &.{
                @as(u32, @intCast(4 + path.items.len)) << 16 | Op.access_chain,
                ptr_type,
                chain,
                pointer,
            });
            for (path.items) |index| try out.append(self.alloc, try self.intConstant(index));
            try out.appendSlice(self.alloc, &.{ (3 << 16) | Op.store, chain, value });
            return;
        };

        if (self.composites.get(value)) |composite| {
            const w = self.inst(composite.inst);
            for (w[3..], 0..) |element, i| {
                try path.append(self.alloc, @intCast(i));
                defer _ = path.pop();
                try self.storeElements(out, pointer, storage, element, elem_type, path);
            }
            return;
        }

        // Not a known composite (e.g. a loaded row): extract its elements.
        const length = try self.arrayLength(type_id);
        for (0..length) |i| {
            const element = self.newId();
            try out.appendSlice(self.alloc, &.{ (5 << 16) | Op.composite_extract, elem_type, element, value, @intCast(i) });
            try path.append(self.alloc, @intCast(i));
            defer _ = path.pop();
            try self.storeElements(out, pointer, storage, element, elem_type, path);
        }
    }

    fn arrayLength(self: *const Module, type_id: u32) !u32 {
        for (self.insts.items, 0..) |_, index| {
            const w = self.inst(index);
            if ((w[0] & 0xFFFF) != Op.type_array or w[1] != type_id) continue;
            const length_id = w[3];
            for (self.insts.items, 0..) |_, j| {
                const c = self.inst(j);
                if ((c[0] & 0xFFFF) == Op.constant and c.len == 4 and c[2] == length_id) return c[3];
            }
        }
        return error.UnknownArrayLength;
    }

    /// Whether the array value `id` is still used. Only the operands that can
    /// hold an array value are checked: other words may be literals (or
    /// embedded source text) that merely equal `id`.
    fn isUsed(self: *const Module, id: u32, skip: []const bool) bool {
        for (self.insts.items, 0..) |_, index| {
            if (skip[index]) continue;
            const w = self.inst(index);
            const operands: []const u32 = switch (w[0] & 0xFFFF) {
                Op.store => w[2..@min(w.len, 3)],
                Op.composite_extract, Op.copy_object => w[3..@min(w.len, 4)],
                Op.composite_insert => w[3..@min(w.len, 5)],
                Op.composite_construct, Op.constant_composite, Op.spec_constant_composite => w[3..],
                Op.function_call => w[4..],
                Op.return_value => w[1..@min(w.len, 2)],
                Op.select => w[4..@min(w.len, 6)],
                Op.variable => w[@min(w.len, 4)..],
                Op.phi => {
                    // (value, parent block) pairs
                    var i: usize = 3;
                    while (i < w.len) : (i += 2) {
                        if (w[i] == id) return true;
                    }
                    continue;
                },
                else => continue,
            };
            if (std.mem.indexOfScalar(u32, operands, id) != null) return true;
        }
        return false;
    }
};

/// Returns the rewritten module, or null when it has no stores to rewrite.
pub fn expandArrayOfArrayStores(alloc: std.mem.Allocator, words: []const u32) !?[]u32 {
    if (words.len < header_len or words[0] != magic) return error.InvalidSpirv;

    var module: Module = .{ .alloc = alloc, .words = words, .bound = words[3] };
    defer module.deinit();
    try module.scan();

    // Instruction index -> replacement words.
    var replacements: std.AutoHashMapUnmanaged(usize, []u32) = .empty;
    defer {
        var it = replacements.valueIterator();
        while (it.next()) |r| alloc.free(r.*);
        replacements.deinit(alloc);
    }

    var path: std.ArrayList(u32) = .empty;
    defer path.deinit(alloc);
    for (module.insts.items, 0..) |_, index| {
        const w = module.inst(index);
        if ((w[0] & 0xFFFF) != Op.store or w.len < 3) continue;
        const pointer_type = module.value_types.get(w[1]) orelse continue;
        const pointer = module.pointers.get(pointer_type) orelse continue;
        if (!module.isArrayOfArrays(pointer[1])) continue;

        var out: std.ArrayList(u32) = .empty;
        errdefer out.deinit(alloc);
        try module.storeElements(&out, w[1], pointer[0], w[2], pointer[1], &path);
        try replacements.put(alloc, index, try out.toOwnedSlice(alloc));
    }
    if (replacements.count() == 0) return null;

    // Drop the array-of-arrays values that only the rewritten stores used.
    const skip = try alloc.alloc(bool, module.insts.items.len);
    defer alloc.free(skip);
    @memset(skip, false);
    var it = replacements.keyIterator();
    while (it.next()) |index| skip[index.*] = true;

    var removed: std.AutoHashMapUnmanaged(u32, void) = .empty;
    defer removed.deinit(alloc);
    var changed = true;
    while (changed) {
        changed = false;
        var composite_it = module.composites.iterator();
        while (composite_it.next()) |entry| {
            const composite = entry.value_ptr.*;
            if (skip[composite.inst] or !module.isArrayOfArrays(composite.type)) continue;
            if (module.isUsed(entry.key_ptr.*, skip)) continue;
            skip[composite.inst] = true;
            try removed.put(alloc, entry.key_ptr.*, {});
            changed = true;
        }
    }

    var result: std.ArrayList(u32) = .empty;
    errdefer result.deinit(alloc);
    try result.appendSlice(alloc, words[0..header_len]);
    for (module.insts.items, 0..) |_, index| {
        if (module.first_function == index) try result.appendSlice(alloc, module.globals.items);

        const w = module.inst(index);
        if (replacements.get(index)) |replacement| {
            try result.appendSlice(alloc, replacement);
            continue;
        }
        if (skip[index]) continue;
        switch (w[0] & 0xFFFF) {
            Op.name, Op.decorate => if (w.len >= 2 and removed.contains(w[1])) continue,
            else => {},
        }
        try result.appendSlice(alloc, w);
    }
    if (module.first_function == null) try result.appendSlice(alloc, module.globals.items);
    result.items[3] = module.bound;

    return try result.toOwnedSlice(alloc);
}
