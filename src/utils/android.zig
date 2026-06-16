const std = @import("std");
const builtin = @import("builtin");

const c = @import("../root.zig").c;
const jni = if (builtin.abi.isAndroid()) @cImport({
    @cInclude("jni.h");
}) else struct {};

pub const ScreenOrientation = enum(i32) {
    unknown = 0,
    landscape = 1,
    landscape_flipped = 2,
    portrait = 3,
    portrait_flipped = 4,

    fn fromInt(value: i32) ScreenOrientation {
        return switch (value) {
            1 => .landscape,
            2 => .landscape_flipped,
            3 => .portrait,
            4 => .portrait_flipped,
            else => .unknown,
        };
    }
};

pub fn displayName(alloc: std.mem.Allocator, uri: []const u8) !?[]u8 {
    if (!builtin.abi.isAndroid()) @compileError("Function only available for Android");

    const uri_z = try alloc.dupeZ(u8, uri);
    defer alloc.free(uri_z);

    const env_raw = c.SDL_GetAndroidJNIEnv() orelse return null;
    const activity_raw = c.SDL_GetAndroidActivity() orelse return null;

    const env: [*c]jni.JNIEnv = @ptrCast(@alignCast(env_raw));
    const activity: jni.jobject = @ptrCast(activity_raw);
    const f = &env.*[0];
    defer deleteLocalRef(env, f, activity);

    const activity_class = f.GetObjectClass.?(env, activity) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, activity_class);

    const method = f.GetMethodID.?(
        env,
        activity_class,
        "getDisplayNameForUri",
        "(Ljava/lang/String;)Ljava/lang/String;",
    ) orelse {
        clearException(env, f);
        return null;
    };

    const uri_string = f.NewStringUTF.?(env, uri_z.ptr) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, uri_string);

    var args = [_]jni.jvalue{.{ .l = uri_string }};
    const name_object = f.CallObjectMethodA.?(env, activity, method, &args) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, name_object);

    if (hasException(env, f)) {
        f.ExceptionClear.?(env);
        return null;
    }

    const name_string: jni.jstring = @ptrCast(name_object);
    const utf = f.GetStringUTFChars.?(env, name_string, null) orelse {
        clearException(env, f);
        return null;
    };
    defer f.ReleaseStringUTFChars.?(env, name_string, utf);

    const name = std.mem.span(utf);
    if (name.len == 0) return null;

    return try alloc.dupe(u8, std.fs.path.stem(name));
}

pub fn currentScreenOrientation() ?ScreenOrientation {
    if (!builtin.abi.isAndroid()) @compileError("Function only available for Android");

    const env_raw = c.SDL_GetAndroidJNIEnv() orelse return null;
    const activity_raw = c.SDL_GetAndroidActivity() orelse return null;

    const env: [*c]jni.JNIEnv = @ptrCast(@alignCast(env_raw));
    const activity: jni.jobject = @ptrCast(activity_raw);
    const f = &env.*[0];
    defer deleteLocalRef(env, f, activity);

    const activity_class = f.GetObjectClass.?(env, activity) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, activity_class);

    const method = f.GetMethodID.?(
        env,
        activity_class,
        "getCurrentScreenOrientation",
        "()I",
    ) orelse {
        clearException(env, f);
        return null;
    };

    const orientation = f.CallIntMethodA.?(env, activity, method, null);
    if (hasException(env, f)) {
        f.ExceptionClear.?(env);
        return null;
    }

    return ScreenOrientation.fromInt(orientation);
}

pub fn getExternalCacheDir(alloc: std.mem.Allocator) !?[]u8 {
    if (!builtin.abi.isAndroid()) @compileError("Function only available for Android");

    const env_raw = c.SDL_GetAndroidJNIEnv() orelse return null;
    const activity_raw = c.SDL_GetAndroidActivity() orelse return null;

    const env: [*c]jni.JNIEnv = @ptrCast(@alignCast(env_raw));
    const activity: jni.jobject = @ptrCast(activity_raw);
    const f = &env.*[0];
    defer deleteLocalRef(env, f, activity);

    const activity_class = f.GetObjectClass.?(env, activity) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, activity_class);

    const get_external_cache_dir = f.GetMethodID.?(
        env,
        activity_class,
        "getExternalCacheDir",
        "()Ljava/io/File;",
    ) orelse {
        clearException(env, f);
        return null;
    };

    const file_object = f.CallObjectMethodA.?(env, activity, get_external_cache_dir, null) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, file_object);

    if (hasException(env, f)) {
        f.ExceptionClear.?(env);
        return null;
    }

    const file_class = f.GetObjectClass.?(env, file_object) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, file_class);

    const get_absolute_path = f.GetMethodID.?(
        env,
        file_class,
        "getAbsolutePath",
        "()Ljava/lang/String;",
    ) orelse {
        clearException(env, f);
        return null;
    };

    const path_object = f.CallObjectMethodA.?(env, file_object, get_absolute_path, null) orelse {
        clearException(env, f);
        return null;
    };
    defer deleteLocalRef(env, f, path_object);

    if (hasException(env, f)) {
        f.ExceptionClear.?(env);
        return null;
    }

    const path_string: jni.jstring = @ptrCast(path_object);
    const utf = f.GetStringUTFChars.?(env, path_string, null) orelse {
        clearException(env, f);
        return null;
    };
    defer f.ReleaseStringUTFChars.?(env, path_string, utf);

    const path = std.mem.span(utf);
    if (path.len == 0) return null;

    return try alloc.dupe(u8, path);
}

fn hasException(env: [*c]jni.JNIEnv, f: *allowzero const jni.JNINativeInterface) bool {
    return f.ExceptionCheck.?(env) != 0;
}

fn clearException(env: [*c]jni.JNIEnv, f: *allowzero const jni.JNINativeInterface) void {
    if (hasException(env, f)) {
        f.ExceptionClear.?(env);
    }
}

fn deleteLocalRef(env: [*c]jni.JNIEnv, f: *allowzero const jni.JNINativeInterface, ref: jni.jobject) void {
    if (ref != null) {
        f.DeleteLocalRef.?(env, ref);
    }
}
