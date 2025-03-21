const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const options = .{
        .use_double_precision = b.option(
            bool,
            "use_double_precision",
            "Enable double precision",
        ) orelse false,
        .enable_asserts = b.option(
            bool,
            "enable_asserts",
            "Enable assertions",
        ) orelse (optimize == .Debug),
        .enable_cross_platform_determinism = b.option(
            bool,
            "enable_cross_platform_determinism",
            "Enables cross-platform determinism",
        ) orelse true,
        .enable_debug_renderer = b.option(
            bool,
            "enable_debug_renderer",
            "Enable debug renderer",
        ) orelse false,
        .shared = b.option(
            bool,
            "shared",
            "Build JoltC as shared lib",
        ) orelse false,
        .no_exceptions = b.option(
            bool,
            "no_exceptions",
            "Disable C++ Exceptions",
        ) orelse true,
    };

    const joltc_lib_mod = b.addModule("joltc", .{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .pic = true,
    });

    if (target.result.os.tag == .windows and options.shared) {
        joltc_lib_mod.addCMacro("JPC_API", "extern __declspec(dllexport)");
    }

    if (target.result.abi != .msvc) {
        joltc_lib_mod.link_libcpp = true;
    } else {
        joltc_lib_mod.linkSystemLibrary("advapi32", .{ .needed = true });
    }

    const joltc_lib = b.addLibrary(.{
        .name = "joltc",
        .root_module = joltc_lib_mod,
        .linkage = if (options.shared) .dynamic else .static,
    });

    const flags = &.{
        if (options.shared) "-DJPH_SHARED_LIBRARY_BUILD" else "",
        if (options.enable_cross_platform_determinism) "-DJPH_CROSS_PLATFORM_DETERMINISTIC" else "",
        if (options.enable_debug_renderer) "-DJPH_DEBUG_RENDERER" else "",
        if (options.use_double_precision) "-DJPH_DOUBLE_PRECISION" else "",
        if (options.enable_asserts) "-DJPH_ENABLE_ASSERTS" else "",
        if (options.shared) "-DJPH_SHARED_LIBRARY_BUILD" else "",
        if (options.no_exceptions) "-fno-exceptions" else "",
        "-fno-access-control",
        "-fno-sanitize=undefined",
    };

    //---------- Jolt Physics (JPH) ----------//
    const jph_dep = b.dependency("JoltPhysics", .{});
    joltc_lib.addIncludePath(jph_dep.path("."));

    //Walk the JPH source directory and add cpp files
    //Would be better to add the sources explicitly?
    var jph_src_dir = try std.fs.openDirAbsolute(
        jph_dep.path("Jolt").getPath(b),
        .{ .iterate = true },
    );
    defer jph_src_dir.close();

    var jph_src_walker = try jph_src_dir.walk(b.allocator);
    defer jph_src_walker.deinit();

    var jph_src_files = std.ArrayList([]const u8).init(b.allocator);
    defer {
        for (jph_src_files.items) |item| {
            b.allocator.free(item);
        }
        jph_src_files.deinit();
    }

    while (try jph_src_walker.next()) |*entry| {
        switch (entry.kind) {
            .file => {
                if (std.mem.endsWith(u8, entry.basename, ".cpp")) {
                    const path_copy = try b.allocator.dupe(u8, entry.path);
                    try jph_src_files.append(path_copy);
                }
            },
            else => {},
        }
    }

    joltc_lib.addCSourceFiles(.{
        .root = jph_dep.path("Jolt"),
        .files = jph_src_files.items,
        .flags = flags,
    });

    //---------- JoltC ----------//
    joltc_lib.addIncludePath(b.path("include"));
    joltc_lib.installHeadersDirectory(
        b.path("include"),
        "",
        .{},
    );

    joltc_lib.addCSourceFiles(.{
        .root = b.path("src"),
        .files = &.{
            "joltc.c",
            "joltc.cpp",
            "joltc_assert.cpp",
        },
        .flags = flags,
    });
    b.installArtifact(joltc_lib);

    //Run test
    const test_step = b.step("test", "Run joltc tests");
    const tests = b.addTest(.{
        .name = "joltc-tests",
        .root_source_file = b.path("src/test.zig"),
        .target = target,
        .optimize = optimize,
    });
    tests.addIncludePath(b.path("include"));
    tests.linkLibrary(joltc_lib);

    test_step.dependOn(&b.addRunArtifact(tests).step);
}
