load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "gcs",
    includes = ["gcs-lib/include"],
    srcs = glob(["gcs-lib/src/**"]),
    hdrs = glob(["gcs-lib/include/**"]),
    deps = [
        "@ceres-solver//:ceres",
        "@com_github_brunocodutra_metal//:metal",
        "@com_github_boostorg_preprocessor//:boost-preprocessor",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main/main.cpp"],
    deps = [
        "//:gcs",
        "@ceres-solver//:ceres",
    ],
)
