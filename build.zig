const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const lib = b.addSharedLibrary(.{
        .name = "box2z",
        .target = target,
        .optimize = optimize,
    });

    lib.linkLibC();

    const box2c = b.dependency("box2c", .{});

    lib.addIncludePath(box2c.path("src"));
    lib.addIncludePath(box2c.path("include"));

    inline for (&[_][]const u8{
        "aabb.c",
        "allocate.c",
        "array.c",
        "bitset.c",
        "block_allocator.c",
        "body.c",
        "broad_phase.c",
        "contact.c",
        "contact_solver.c",
        "core.c",
        "distance.c",
        "distance_joint.c",
        "dynamic_tree.c",
        "geometry.c",
        "graph.c",
        "hull.c",
        "island.c",
        "joint.c",
        "manifold.c",
        "math.c",
        "mouse_joint.c",
        "pool.c",
        "prismatic_joint.c",
        "revolute_joint.c",
        "shape.c",
        "stack_allocator.c",
        "table.c",
        "timer.c",
        "types.c",
        "weld_joint.c",
        "world.c",
    }) |file| {
        lib.addCSourceFile(.{
            .file = box2c.path(b.pathJoin(&.{ "src", file })),
            .flags = &.{},
        });
    }

    b.installArtifact(lib);
    const main_tests = b.addTest(.{
        .root_source_file = .{ .path = "src/main.zig" },
        .target = target,
        .optimize = optimize,
    });
    // main_tests.linkLibC();
    main_tests.addIncludePath(box2c.path("include"));
    main_tests.linkLibrary(lib);

    const run_main_tests = b.addRunArtifact(main_tests);

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build test`
    // This will evaluate the `test` step rather than the default, which is "install".
    const test_step = b.step("test", "Run library tests");
    test_step.dependOn(&run_main_tests.step);
}
