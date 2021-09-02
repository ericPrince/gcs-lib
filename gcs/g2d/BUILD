load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name="g2d",
    include_prefix="gcs/g2d/",
    srcs=glob(["*.cpp"], ["*_test.cpp"]),
    hdrs=glob(["*.h"]),
    deps = [
        "//gcs/core",
        "@com_github_ceres-solver_ceres-solver//:ceres",
        "@com_github_brunocodutra_metal//:metal",
        "@com_github_boostorg_preprocessor//:boost-preprocessor",
    ],
)
