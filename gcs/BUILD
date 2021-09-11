load("@rules_cc//cc:defs.bzl", "cc_test")
load("//:lint.bzl", "clang_format", "cppcheck")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_test(
    name = "problem1_test",
    srcs = ["problem1_test.cpp"],
    deps = [
        "//gcs/basic",
        "//gcs/core",
        "//gcs/g2d",
    ],
)

clang_format(
    name = "clang_format_all",
    srcs = glob([
        "**/*.cpp",
        "**/*.h",
    ]),
)

cppcheck(
    name = "cppcheck_all",
    srcs = glob(["**"]),
)
