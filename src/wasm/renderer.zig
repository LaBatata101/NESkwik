const std = @import("std");
const raster = @import("font_raster");
const c = @import("../root.zig").c;
const clay = @import("../ui/core/clay.zig");
const sdlError = @import("../utils/sdl.zig").sdlError;

pub const Texture = struct {
    id: c.GLuint,
    width: u32,
    height: u32,

    pub fn init(alloc: std.mem.Allocator, id: c.GLuint, width: u32, height: u32) !*Texture {
        const self = try alloc.create(Texture);
        self.* = .{ .id = id, .width = width, .height = height };
        return self;
    }

    pub fn deinit(self: *Texture, alloc: std.mem.Allocator) void {
        c.glDeleteTextures(1, &self.id);
        alloc.destroy(self);
    }
};

const Vertex = extern struct {
    position: c.SDL_FPoint,
    color: c.SDL_FColor,
    tex_coord: c.SDL_FPoint,
    rect: c.SDL_FRect,
    corner_radius: c.SDL_FColor,
    overlay_color: c.SDL_FColor,

    const empty_rect = c.SDL_FRect{ .x = 0, .y = 0, .w = 0, .h = 0 };
    const no_radius = c.SDL_FColor{ .r = 0, .g = 0, .b = 0, .a = 0 };

    fn init(position: c.SDL_FPoint, color: c.SDL_FColor, tex_coord: c.SDL_FPoint, overlay_color: c.SDL_FColor) Vertex {
        return .{
            .position = position,
            .color = color,
            .tex_coord = tex_coord,
            .rect = empty_rect,
            .corner_radius = no_radius,
            .overlay_color = overlay_color,
        };
    }

    fn rounded(
        position: c.SDL_FPoint,
        color: c.SDL_FColor,
        tex_coord: c.SDL_FPoint,
        rect: c.SDL_FRect,
        corner_radius: clay.CornerRadius,
        overlay_color: c.SDL_FColor,
    ) Vertex {
        return .{
            .position = position,
            .color = color,
            .tex_coord = tex_coord,
            .rect = rect,
            .corner_radius = .{
                .r = corner_radius.top_left,
                .g = corner_radius.top_right,
                .b = corner_radius.bottom_right,
                .a = corner_radius.bottom_left,
            },
            .overlay_color = overlay_color,
        };
    }
};

pub const RendererWeb = struct {
    alloc: std.mem.Allocator,
    window: ?*c.SDL_Window,
    context: c.SDL_GLContext,
    program: c.GLuint,
    vertex_buffer: c.GLuint,
    index_buffer: c.GLuint,
    white_texture: *Texture,
    font_texture: ?*Texture = null,
    font_size: u32 = 0,
    font_generation: usize = 0,
    current_texture: *Texture,
    display_scale: f32,
    clip_rect: ?c.SDL_Rect = null,
    clip_stack: std.ArrayList(?c.SDL_Rect) = .empty,
    current_overlay_color: c.SDL_FColor = .{ .r = 0, .g = 0, .b = 0, .a = 0 },
    overlay_stack: std.ArrayList(c.SDL_FColor) = .empty,

    const Self = @This();
    const ROUNDED_RECT_AA_PAD: f32 = 1.0;

    const vertex_shader_source =
        \\#version 300 es
        \\precision highp float;
        \\in vec2 position;
        \\in vec4 color;
        \\in vec2 tex_coord;
        \\in vec4 rect;
        \\in vec4 corner_radius;
        \\in vec4 overlay_color;
        \\uniform vec2 viewport_size;
        \\out vec4 frag_color;
        \\out vec2 frag_tex_coord;
        \\out vec2 frag_position;
        \\out vec4 frag_rect;
        \\out vec4 frag_corner_radius;
        \\out vec4 frag_overlay_color;
        \\void main() {
        \\    vec2 clip = position * vec2(2.0, -2.0) / viewport_size + vec2(-1.0, 1.0);
        \\    gl_Position = vec4(clip, 0.0, 1.0);
        \\    frag_color = color;
        \\    frag_tex_coord = tex_coord;
        \\    frag_position = position;
        \\    frag_rect = rect;
        \\    frag_corner_radius = corner_radius;
        \\    frag_overlay_color = overlay_color;
        \\}
    ;

    const fragment_shader_source =
        \\#version 300 es
        \\precision highp float;
        \\uniform sampler2D image;
        \\in vec4 frag_color;
        \\in vec2 frag_tex_coord;
        \\in vec2 frag_position;
        \\in vec4 frag_rect;
        \\in vec4 frag_corner_radius;
        \\in vec4 frag_overlay_color;
        \\out vec4 output_color;
        \\float roundedRectDistance(vec2 p, vec2 half_size, vec4 radii) {
        \\    bool right = p.x > 0.0;
        \\    bool bottom = p.y > 0.0;
        \\    float radius = bottom ? (right ? radii.z : radii.w) : (right ? radii.y : radii.x);
        \\    radius = min(radius, min(half_size.x, half_size.y));
        \\    vec2 q = abs(p) - (half_size - vec2(radius));
        \\    return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - radius;
        \\}
        \\void main() {
        \\    vec4 result = texture(image, frag_tex_coord) * frag_color;
        \\    if (dot(frag_corner_radius, vec4(1.0)) > 0.0 && frag_rect.z > 0.0 && frag_rect.w > 0.0) {
        \\        vec2 half_size = frag_rect.zw * 0.5;
        \\        vec2 center = frag_rect.xy + half_size;
        \\        float dist = roundedRectDistance(frag_position - center, half_size, frag_corner_radius);
        \\        float aa = max(fwidth(dist), 0.0001);
        \\        result.a *= clamp(0.5 - dist / aa, 0.0, 1.0);
        \\    }
        \\    result.rgb = mix(result.rgb, frag_overlay_color.rgb, frag_overlay_color.a);
        \\    output_color = result;
        \\}
    ;

    pub fn init(alloc: std.mem.Allocator, _: std.Io, _: ?*c.SDL_GPUDevice, window: ?*c.SDL_Window, _: c_uint) !*Self {
        const context = sdlError(c.SDL_GL_CreateContext(window));
        errdefer _ = c.SDL_GL_DestroyContext(context);
        sdlError(c.SDL_GL_MakeCurrent(window, context));

        const vertex_shader = try compileShader(c.GL_VERTEX_SHADER, vertex_shader_source);
        defer c.glDeleteShader(vertex_shader);
        const fragment_shader = try compileShader(c.GL_FRAGMENT_SHADER, fragment_shader_source);
        defer c.glDeleteShader(fragment_shader);

        const program = c.glCreateProgram();
        if (program == 0) return error.GLProgramCreateFailed;
        errdefer c.glDeleteProgram(program);

        c.glAttachShader(program, vertex_shader);
        c.glAttachShader(program, fragment_shader);

        c.glBindAttribLocation(program, 0, "position");
        c.glBindAttribLocation(program, 1, "color");
        c.glBindAttribLocation(program, 2, "tex_coord");
        c.glBindAttribLocation(program, 3, "rect");
        c.glBindAttribLocation(program, 4, "corner_radius");
        c.glBindAttribLocation(program, 5, "overlay_color");

        c.glLinkProgram(program);

        var linked: c.GLint = 0;
        c.glGetProgramiv(program, c.GL_LINK_STATUS, &linked);
        if (linked == 0) return error.GLProgramLinkFailed;

        var buffers: [2]c.GLuint = undefined;
        c.glGenBuffers(buffers.len, &buffers);

        const self = try alloc.create(Self);
        errdefer alloc.destroy(self);
        self.* = .{
            .alloc = alloc,
            .window = window,
            .context = context,
            .program = program,
            .vertex_buffer = buffers[0],
            .index_buffer = buffers[1],
            .white_texture = undefined,
            .current_texture = undefined,
            .display_scale = c.SDL_GetWindowDisplayScale(window),
        };

        const white_pixel = [_]u8{ 255, 255, 255, 255 };
        self.white_texture = try self.createTexture(1, 1, &white_pixel, false);
        self.current_texture = self.white_texture;

        c.glUseProgram(program);
        c.glUniform1i(c.glGetUniformLocation(program, "image"), 0);
        c.glEnable(c.GL_BLEND);
        c.glBlendFunc(c.GL_SRC_ALPHA, c.GL_ONE_MINUS_SRC_ALPHA);
        c.glDisable(c.GL_DEPTH_TEST);
        return self;
    }

    fn compileShader(shader_type: c.GLenum, source: []const u8) !c.GLuint {
        const shader = c.glCreateShader(shader_type);
        if (shader == 0) return error.GLShaderCreateFailed;
        errdefer c.glDeleteShader(shader);

        var source_ptr: [*c]const c.GLchar = @ptrCast(source.ptr);
        var source_len: c.GLint = @intCast(source.len);

        c.glShaderSource(shader, 1, &source_ptr, &source_len);
        c.glCompileShader(shader);

        var compiled: c.GLint = 0;
        c.glGetShaderiv(shader, c.GL_COMPILE_STATUS, &compiled);
        if (compiled == 0) {
            var log: [2048]u8 = undefined;
            var length: c.GLsizei = 0;
            c.glGetShaderInfoLog(shader, log.len, &length, @ptrCast(&log));
            std.log.err("UI GLSL compile failed: {s}", .{log[0..@intCast(@max(0, length))]});
            return error.GLShaderCompileFailed;
        }

        return shader;
    }

    pub fn setDisplayScale(self: *Self, display_scale: f32) void {
        self.display_scale = display_scale;
    }

    pub fn deinit(self: *Self) void {
        sdlError(c.SDL_GL_MakeCurrent(self.window, self.context));
        self.clip_stack.deinit(self.alloc);
        self.overlay_stack.deinit(self.alloc);
        if (self.font_texture) |texture| texture.deinit(self.alloc);
        self.white_texture.deinit(self.alloc);
        c.glDeleteBuffers(1, &self.vertex_buffer);
        c.glDeleteBuffers(1, &self.index_buffer);
        c.glDeleteProgram(self.program);
        _ = c.SDL_GL_DestroyContext(self.context);
        self.alloc.destroy(self);
    }

    pub fn reset(self: *Self) void {
        sdlError(c.SDL_GL_MakeCurrent(self.window, self.context));
        self.clip_stack.clearRetainingCapacity();
        self.overlay_stack.clearRetainingCapacity();
        self.clip_rect = null;
        self.current_texture = self.white_texture;
        self.current_overlay_color = .{ .r = 0, .g = 0, .b = 0, .a = 0 };

        var width: c_int = 0;
        var height: c_int = 0;
        sdlError(c.SDL_GetWindowSizeInPixels(self.window, &width, &height));
        c.glViewport(0, 0, width, height);
        c.glDisable(c.GL_SCISSOR_TEST);
        c.glClearColor(0, 0, 0, 1);
        c.glClear(c.GL_COLOR_BUFFER_BIT);
        c.glUseProgram(self.program);
        c.glUniform2f(
            c.glGetUniformLocation(self.program, "viewport_size"),
            @as(f32, @floatFromInt(width)) / self.display_scale,
            @as(f32, @floatFromInt(height)) / self.display_scale,
        );
    }

    pub fn present(self: *Self) void {
        sdlError(c.SDL_GL_SwapWindow(self.window));
    }

    pub fn resumeRendering(self: *Self) void {
        sdlError(c.SDL_GL_MakeCurrent(self.window, self.context));
        c.glBindFramebuffer(c.GL_FRAMEBUFFER, 0);
        var width: c_int = 0;
        var height: c_int = 0;
        sdlError(c.SDL_GetWindowSizeInPixels(self.window, &width, &height));
        c.glViewport(0, 0, width, height);
        c.glDisable(c.GL_DEPTH_TEST);
        c.glEnable(c.GL_BLEND);
        c.glBlendFunc(c.GL_SRC_ALPHA, c.GL_ONE_MINUS_SRC_ALPHA);
        c.glUseProgram(self.program);
        self.applyClipRect();
    }

    pub fn pushClipRect(self: *Self, clip: c.SDL_Rect) void {
        self.clip_stack.append(self.alloc, self.clip_rect) catch @panic("OOM");
        self.clip_rect = if (self.clip_rect) |current| intersectClipRects(current, clip) else clip;
        self.applyClipRect();
    }

    pub fn popClipRect(self: *Self) void {
        self.clip_rect = self.clip_stack.pop() orelse null;
        self.applyClipRect();
    }

    fn applyClipRect(self: *Self) void {
        const clip = self.clip_rect orelse {
            c.glDisable(c.GL_SCISSOR_TEST);
            return;
        };
        var pixel_width: c_int = 0;
        var pixel_height: c_int = 0;
        sdlError(c.SDL_GetWindowSizeInPixels(self.window, &pixel_width, &pixel_height));
        const scale = self.display_scale;
        const x: c_int = @intFromFloat(@floor(@as(f32, @floatFromInt(clip.x)) * scale));
        const y: c_int = @intFromFloat(@floor(@as(f32, @floatFromInt(clip.y)) * scale));
        const right: c_int = @intFromFloat(@ceil(@as(f32, @floatFromInt(clip.x + clip.w)) * scale));
        const bottom: c_int = @intFromFloat(@ceil(@as(f32, @floatFromInt(clip.y + clip.h)) * scale));
        c.glEnable(c.GL_SCISSOR_TEST);
        c.glScissor(x, pixel_height - bottom, @max(0, right - x), @max(0, bottom - y));
    }

    pub fn pushOverlayColor(self: *Self, color_: c.SDL_FColor) void {
        self.overlay_stack.append(self.alloc, self.current_overlay_color) catch @panic("OOM");
        self.current_overlay_color = color_;
    }

    pub fn popOverlayColor(self: *Self) void {
        self.current_overlay_color = self.overlay_stack.pop() orelse .{ .r = 0, .g = 0, .b = 0, .a = 0 };
    }

    fn intersectClipRects(a: c.SDL_Rect, b: c.SDL_Rect) c.SDL_Rect {
        const left = @max(a.x, b.x);
        const top = @max(a.y, b.y);
        const right = @min(a.x + a.w, b.x + b.w);
        const bottom = @min(a.y + a.h, b.y + b.h);
        return .{ .x = left, .y = top, .w = @max(0, right - left), .h = @max(0, bottom - top) };
    }

    pub fn releaseMemory(self: *Self) void {
        self.clip_stack.clearAndFree(self.alloc);
        self.overlay_stack.clearAndFree(self.alloc);
    }

    pub fn createTexture(self: *Self, width: u32, height: u32, pixels: ?[]const u8, linear: bool) !*Texture {
        var id: c.GLuint = 0;
        c.glGenTextures(1, &id);
        if (id == 0) return error.GLTextureCreateFailed;
        errdefer c.glDeleteTextures(1, &id);
        c.glBindTexture(c.GL_TEXTURE_2D, id);

        const filter: c.GLint = if (linear) c.GL_LINEAR else c.GL_NEAREST;
        c.glTexParameteri(c.GL_TEXTURE_2D, c.GL_TEXTURE_MIN_FILTER, filter);
        c.glTexParameteri(c.GL_TEXTURE_2D, c.GL_TEXTURE_MAG_FILTER, filter);
        c.glTexParameteri(c.GL_TEXTURE_2D, c.GL_TEXTURE_WRAP_S, c.GL_CLAMP_TO_EDGE);
        c.glTexParameteri(c.GL_TEXTURE_2D, c.GL_TEXTURE_WRAP_T, c.GL_CLAMP_TO_EDGE);
        c.glPixelStorei(c.GL_UNPACK_ALIGNMENT, 1);
        c.glTexImage2D(
            c.GL_TEXTURE_2D,
            0,
            c.GL_RGBA,
            @intCast(width),
            @intCast(height),
            0,
            c.GL_RGBA,
            c.GL_UNSIGNED_BYTE,
            if (pixels) |data| data.ptr else null,
        );

        return try Texture.init(self.alloc, id, width, height);
    }

    pub fn pushTexture(_: *Self, texture_upload: struct {
        texture: ?*Texture,
        pixels: []const u8,
        width: u32,
        height: u32,
        pixel_format: c.SDL_PixelFormat,
    }) void {
        const texture = texture_upload.texture orelse return;
        c.glBindTexture(c.GL_TEXTURE_2D, texture.id);
        c.glPixelStorei(c.GL_UNPACK_ALIGNMENT, 1);
        c.glTexSubImage2D(
            c.GL_TEXTURE_2D,
            0,
            0,
            0,
            @intCast(texture_upload.width),
            @intCast(texture_upload.height),
            c.GL_RGBA,
            c.GL_UNSIGNED_BYTE,
            texture_upload.pixels.ptr,
        );
    }

    pub fn flush(_: *Self) void {}

    pub fn setTexture(self: *Self, texture: ?*Texture) void {
        self.current_texture = texture orelse self.white_texture;
    }

    pub fn setTextTexture(self: *Self, texture: ?*Texture) void {
        self.current_texture = texture orelse self.white_texture;
    }

    pub fn pushRect(self: *Self, rect_: c.SDL_FRect, color_: c.SDL_FColor, uv: ?c.SDL_FRect) void {
        self.rect(self.current_texture, rect_, color_, uv);
    }

    pub fn pushTextRect(self: *Self, rect_: c.SDL_FRect, color_: c.SDL_FColor, uv: c.SDL_FRect) void {
        self.rectWithOverlay(self.current_texture, rect_, color_, uv, .{ .r = 0, .g = 0, .b = 0, .a = 0 });
    }

    pub fn pushRoundedTexturedRect(self: *Self, rect_: c.SDL_FRect, color_: c.SDL_FColor, corner_radius: clay.CornerRadius, uv: ?c.SDL_FRect) void {
        self.roundedRect(self.current_texture, rect_, color_, corner_radius, uv);
    }

    pub fn pushRoundedRect(self: *Self, rect_: c.SDL_FRect, color_: c.SDL_FColor, corner_radius: clay.CornerRadius) void {
        self.roundedRect(self.current_texture, rect_, color_, corner_radius, null);
    }

    pub fn pushTriangle(self: *Self, p1: clay.Vector2, p2: clay.Vector2, p3: clay.Vector2, color_: c.SDL_FColor) void {
        const vertices = [_]Vertex{
            Vertex.init(.{ .x = p1.x, .y = p1.y }, color_, .{ .x = 0, .y = 0 }, self.current_overlay_color),
            Vertex.init(.{ .x = p2.x, .y = p2.y }, color_, .{ .x = 0, .y = 0 }, self.current_overlay_color),
            Vertex.init(.{ .x = p3.x, .y = p3.y }, color_, .{ .x = 0, .y = 0 }, self.current_overlay_color),
        };
        const indices = [_]u16{ 0, 1, 2 };
        self.geometry(self.current_texture, &vertices, &indices);
    }

    fn geometry(self: *Self, texture: *Texture, vertices: []const Vertex, indices: []const u16) void {
        if (vertices.len == 0 or indices.len == 0) return;
        c.glActiveTexture(c.GL_TEXTURE0);
        c.glBindTexture(c.GL_TEXTURE_2D, texture.id);
        c.glBindBuffer(c.GL_ARRAY_BUFFER, self.vertex_buffer);
        c.glBufferData(c.GL_ARRAY_BUFFER, @intCast(vertices.len * @sizeOf(Vertex)), vertices.ptr, c.GL_STREAM_DRAW);
        c.glBindBuffer(c.GL_ELEMENT_ARRAY_BUFFER, self.index_buffer);
        c.glBufferData(c.GL_ELEMENT_ARRAY_BUFFER, @intCast(indices.len * @sizeOf(u16)), indices.ptr, c.GL_STREAM_DRAW);

        inline for (0..6) |attribute| c.glEnableVertexAttribArray(attribute);
        c.glVertexAttribPointer(0, 2, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "position")));
        c.glVertexAttribPointer(1, 4, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "color")));
        c.glVertexAttribPointer(2, 2, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "tex_coord")));
        c.glVertexAttribPointer(3, 4, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "rect")));
        c.glVertexAttribPointer(4, 4, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "corner_radius")));
        c.glVertexAttribPointer(5, 4, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "overlay_color")));
        c.glDrawElements(c.GL_TRIANGLES, @intCast(indices.len), c.GL_UNSIGNED_SHORT, null);
    }

    fn rect(self: *Self, texture: *Texture, rect_: c.SDL_FRect, color_: c.SDL_FColor, uv_: ?c.SDL_FRect) void {
        self.rectWithOverlay(texture, rect_, color_, uv_, self.current_overlay_color);
    }

    fn rectWithOverlay(
        self: *Self,
        texture: *Texture,
        rect_: c.SDL_FRect,
        tint: c.SDL_FColor,
        uv_: ?c.SDL_FRect,
        overlay: c.SDL_FColor,
    ) void {
        const uv = uv_ orelse c.SDL_FRect{ .x = 0, .y = 0, .w = 1, .h = 1 };
        const vertices = [_]Vertex{
            Vertex.init(.{ .x = rect_.x, .y = rect_.y }, tint, .{ .x = uv.x, .y = uv.y }, overlay),
            Vertex.init(.{ .x = rect_.x + rect_.w, .y = rect_.y }, tint, .{ .x = uv.x + uv.w, .y = uv.y }, overlay),
            Vertex.init(.{ .x = rect_.x + rect_.w, .y = rect_.y + rect_.h }, tint, .{ .x = uv.x + uv.w, .y = uv.y + uv.h }, overlay),
            Vertex.init(.{ .x = rect_.x, .y = rect_.y + rect_.h }, tint, .{ .x = uv.x, .y = uv.y + uv.h }, overlay),
        };
        const indices = [_]u16{ 0, 1, 2, 0, 2, 3 };
        self.geometry(texture, &vertices, &indices);
    }

    fn roundedRect(self: *Self, texture: *Texture, rect_: c.SDL_FRect, color_: c.SDL_FColor, radius_: clay.CornerRadius, uv_: ?c.SDL_FRect) void {
        if (!hasCornerRadius(radius_)) {
            self.rect(texture, rect_, color_, uv_);
            return;
        }

        const uv = uv_ orelse c.SDL_FRect{ .x = 0, .y = 0, .w = 1, .h = 1 };
        var radius = radius_;
        const max_radius = @min(rect_.w, rect_.h) / 2.0;
        radius.top_left = @min(@max(radius.top_left, 0.0), max_radius);
        radius.top_right = @min(@max(radius.top_right, 0.0), max_radius);
        radius.bottom_left = @min(@max(radius.bottom_left, 0.0), max_radius);
        radius.bottom_right = @min(@max(radius.bottom_right, 0.0), max_radius);

        const ScalePair = struct {
            fn apply(a: *f32, b: *f32, max_sum: f32) void {
                const sum = a.* + b.*;
                if (sum > max_sum and sum > 0.0) {
                    const scale = max_sum / sum;
                    a.* *= scale;
                    b.* *= scale;
                }
            }
        };
        ScalePair.apply(&radius.top_left, &radius.top_right, rect_.w);
        ScalePair.apply(&radius.bottom_left, &radius.bottom_right, rect_.w);
        ScalePair.apply(&radius.top_left, &radius.bottom_left, rect_.h);
        ScalePair.apply(&radius.top_right, &radius.bottom_right, rect_.h);

        const pad = @min(ROUNDED_RECT_AA_PAD, @min(rect_.w, rect_.h) * 0.5);
        const x = rect_.x - pad;
        const y = rect_.y - pad;
        const w = rect_.w + pad * 2.0;
        const h = rect_.h + pad * 2.0;
        const vertices = [_]Vertex{
            Vertex.rounded(.{ .x = x, .y = y }, color_, texCoordForPoint(rect_, uv, x, y), rect_, radius, self.current_overlay_color),
            Vertex.rounded(.{ .x = x + w, .y = y }, color_, texCoordForPoint(rect_, uv, x + w, y), rect_, radius, self.current_overlay_color),
            Vertex.rounded(.{ .x = x + w, .y = y + h }, color_, texCoordForPoint(rect_, uv, x + w, y + h), rect_, radius, self.current_overlay_color),
            Vertex.rounded(.{ .x = x, .y = y + h }, color_, texCoordForPoint(rect_, uv, x, y + h), rect_, radius, self.current_overlay_color),
        };
        const indices = [_]u16{ 0, 1, 2, 0, 2, 3 };
        self.geometry(texture, &vertices, &indices);
    }

    fn hasCornerRadius(radius: clay.CornerRadius) bool {
        return radius.top_left > 0.0 or radius.top_right > 0.0 or radius.bottom_left > 0.0 or radius.bottom_right > 0.0;
    }

    fn texCoordForPoint(rect_: c.SDL_FRect, uv: c.SDL_FRect, x: f32, y: f32) c.SDL_FPoint {
        return .{
            .x = uv.x + uv.w * ((x - rect_.x) / rect_.w),
            .y = uv.y + uv.h * ((y - rect_.y) / rect_.h),
        };
    }

    pub fn pushRoundedBorder(self: *Self, box: clay.BoundingBox, border: clay.BorderRenderData) void {
        const widths = border.width;
        if (widths.left == 0 and widths.right == 0 and widths.top == 0 and widths.bottom == 0) return;

        const color_ = c.SDL_FColor{
            .r = border.color[0] / 255.0,
            .g = border.color[1] / 255.0,
            .b = border.color[2] / 255.0,
            .a = border.color[3] / 255.0,
        };
        const outer = c.SDL_FRect{ .x = box.x, .y = box.y, .w = box.width, .h = box.height };
        const inner = c.SDL_FRect{
            .x = box.x + @as(f32, @floatFromInt(widths.left)),
            .y = box.y + @as(f32, @floatFromInt(widths.top)),
            .w = @max(0, box.width - @as(f32, @floatFromInt(widths.left + widths.right))),
            .h = @max(0, box.height - @as(f32, @floatFromInt(widths.top + widths.bottom))),
        };
        const radius_outer = @min(
            @max(border.corner_radius.top_left, border.corner_radius.top_right, border.corner_radius.bottom_right, border.corner_radius.bottom_left),
            @min(outer.w, outer.h) / 2,
        );
        const average_width = @as(f32, @floatFromInt(widths.left + widths.right + widths.top + widths.bottom)) / 4;
        const radius_inner = @max(0, radius_outer - average_width);
        const centers_outer = [_]clay.Vector2{
            .{ .x = outer.x + radius_outer, .y = outer.y + radius_outer },
            .{ .x = outer.x + outer.w - radius_outer, .y = outer.y + radius_outer },
            .{ .x = outer.x + outer.w - radius_outer, .y = outer.y + outer.h - radius_outer },
            .{ .x = outer.x + radius_outer, .y = outer.y + outer.h - radius_outer },
        };
        const centers_inner = [_]clay.Vector2{
            .{ .x = inner.x + radius_inner, .y = inner.y + radius_inner },
            .{ .x = inner.x + inner.w - radius_inner, .y = inner.y + radius_inner },
            .{ .x = inner.x + inner.w - radius_inner, .y = inner.y + inner.h - radius_inner },
            .{ .x = inner.x + radius_inner, .y = inner.y + inner.h - radius_inner },
        };

        const segments: usize = 8;
        var vertices: [4 * (segments + 1) * 2]Vertex = undefined;
        var indices: [(4 * (segments + 1) - 1) * 6 + 6]u16 = undefined;
        const half_pi: f32 = std.math.pi / 2.0;
        const start_angles = [_]f32{ std.math.pi, -half_pi, 0, half_pi };
        var vertex_count: usize = 0;
        var index_count: usize = 0;

        for (0..4) |corner| {
            for (0..segments + 1) |segment| {
                const angle = start_angles[corner] + @as(f32, @floatFromInt(segment)) * half_pi / @as(f32, @floatFromInt(segments));
                const cos_angle = @cos(angle);
                const sin_angle = @sin(angle);
                vertices[vertex_count] = Vertex.init(
                    .{
                        .x = centers_outer[corner].x + cos_angle * radius_outer,
                        .y = centers_outer[corner].y + sin_angle * radius_outer,
                    },
                    color_,
                    .{ .x = 0, .y = 0 },
                    self.current_overlay_color,
                );
                vertices[vertex_count + 1] = Vertex.init(
                    .{
                        .x = centers_inner[corner].x + cos_angle * radius_inner,
                        .y = centers_inner[corner].y + sin_angle * radius_inner,
                    },
                    color_,
                    .{ .x = 0, .y = 0 },
                    self.current_overlay_color,
                );
                if (vertex_count > 0) {
                    indices[index_count..][0..6].* = .{
                        @intCast(vertex_count - 2), @intCast(vertex_count), @intCast(vertex_count - 1),
                        @intCast(vertex_count - 1), @intCast(vertex_count), @intCast(vertex_count + 1),
                    };
                    index_count += 6;
                }
                vertex_count += 2;
            }
        }
        indices[index_count..][0..6].* = .{
            @intCast(vertex_count - 2), 0, @intCast(vertex_count - 1),
            @intCast(vertex_count - 1), 0, 1,
        };
        index_count += 6;
        self.geometry(self.current_texture, vertices[0..vertex_count], indices[0..index_count]);
    }

    pub fn updateFont(self: *Self, atlas: *const raster.Atlas) !void {
        const generation = atlas.modified.load(.monotonic);
        if (self.font_texture != null and self.font_size == atlas.size and self.font_generation == generation) return;
        if (self.font_texture == null or self.font_size != atlas.size) {
            if (self.font_texture) |texture| texture.deinit(self.alloc);
            self.font_texture = null;
            self.font_texture = try self.createTexture(atlas.size, atlas.size, null, false);
            self.font_size = atlas.size;
        }
        const rgba = try self.alloc.alloc(u8, atlas.data.len * 4);
        defer self.alloc.free(rgba);
        for (atlas.data, 0..) |alpha, i| rgba[i * 4 ..][0..4].* = .{ 255, 255, 255, alpha };
        self.pushTexture(.{
            .texture = self.font_texture,
            .pixels = rgba,
            .width = atlas.size,
            .height = atlas.size,
            .pixel_format = c.SDL_PIXELFORMAT_RGBA32,
        });
        self.font_generation = generation;
    }
};
