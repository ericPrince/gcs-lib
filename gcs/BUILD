load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_test(
    name="problem1_test",
    srcs=["problem1_test.cpp"],
    deps=[
        "//gcs/core",
        "//gcs/g2d",
        "@com_github_ceres-solver_ceres-solver//:ceres",
    ]
)
