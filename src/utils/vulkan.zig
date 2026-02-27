const c = @import("../root.zig").c;

pub fn detect_vulkan_version() c_uint {
    const vkProcAddr = c.vkGetInstanceProcAddr(null, "vkEnumerateInstanceVersion");
    if (vkProcAddr == null) {
        // Vulkan 1.0 only
        return c.GLSLANG_TARGET_VULKAN_1_0;
    }
    const vkEnumerateInstanceVersion: *const fn (*u32) callconv(.c) c.VkResult = @ptrCast(vkProcAddr.?);

    var version: u32 = 0;
    if (vkEnumerateInstanceVersion(&version) != c.VK_SUCCESS) {
        @panic("Failed to get Vulkan version");
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
