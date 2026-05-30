const c = @import("../root.zig").c;

var cached_version: ?c_uint = null;

pub fn detect_vulkan_version() c_uint {
    if (cached_version) |version| return version;

    const version = detect_vulkan_version_impl();
    cached_version = version;
    return version;
}

fn detect_vulkan_version_impl() c_uint {
    if (!c.SDL_Vulkan_LoadLibrary(null)) {
        return c.VK_MAKE_VERSION(1, 0, 0);
    }

    const raw_get_proc_addr = c.SDL_Vulkan_GetVkGetInstanceProcAddr() orelse {
        return c.VK_MAKE_VERSION(1, 0, 0);
    };
    const VkGetInstanceProcAddr = *const fn (c.VkInstance, [*c]const u8) callconv(.c) c.PFN_vkVoidFunction;
    const vkGetInstanceProcAddr: VkGetInstanceProcAddr = @ptrCast(raw_get_proc_addr);

    const raw_enumerate_instance_version = vkGetInstanceProcAddr(null, "vkEnumerateInstanceVersion") orelse {
        return c.VK_MAKE_VERSION(1, 0, 0);
    };
    const VkEnumerateInstanceVersion = *const fn ([*c]u32) callconv(.c) c.VkResult;
    const vkEnumerateInstanceVersion: VkEnumerateInstanceVersion = @ptrCast(raw_enumerate_instance_version);

    var version: u32 = 0;
    if (vkEnumerateInstanceVersion(&version) != c.VK_SUCCESS) {
        return c.VK_MAKE_VERSION(1, 0, 0);
    }
    return @intCast(version);
}

pub fn vk_to_glslang_version(vk_version: usize) c_uint {
    return switch (c.VK_VERSION_MINOR(vk_version)) {
        1 => c.GLSLANG_TARGET_VULKAN_1_1,
        2 => c.GLSLANG_TARGET_VULKAN_1_2,
        3 => c.GLSLANG_TARGET_VULKAN_1_3,
        4 => c.GLSLANG_TARGET_VULKAN_1_4,
        else => c.GLSLANG_TARGET_VULKAN_1_0,
    };
}
