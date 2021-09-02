workspace(name = "ericprince_gcs")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

git_repository(
    name = "com_github_ceres-solver_ceres-solver", 
    remote = "https://github.com/ceres-solver/ceres-solver.git",
    tag = "2.0.0",
)

new_git_repository(
    name = "com_github_brunocodutra_metal",
    remote = "https://github.com/brunocodutra/metal",
    tag = "2.1.3",
    build_file_content = 
"""
cc_library(
    name = "metal",
    srcs = [],
    includes = ['include'],
    hdrs = glob(['include/**']),
    visibility = ['//visibility:public'],
)
"""
)

new_git_repository(
    name = "com_github_boostorg_preprocessor",
    remote = "https://github.com/boostorg/preprocessor",
    tag = "boost-1.77.0",
    build_file_content = 
"""
cc_library(
    name="boost-preprocessor",
    srcs = [],
    includes = ['include'],
    hdrs = glob(['include/**']),
    visibility = ['//visibility:public'],
)
"""
)


# the below are defined in the ceres-solver WORKSPACE

# External dependency: Google Flags; has Bazel build already.
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
    strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
    urls = [
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
    ],
)

# External dependency: Google Log; has Bazel build already.
http_archive(
    name = "com_github_google_glog",
    sha256 = "7083af285bed3995b5dc2c982f7de39bced9f0e6fd78d631f3285490922a0c3d",
    strip_prefix = "glog-3106945d8d3322e5cbd5658d482c9ffed2d892c0",
    urls = [
        "https://github.com/drigz/glog/archive/3106945d8d3322e5cbd5658d482c9ffed2d892c0.tar.gz",
    ],
)

# External dependency: Eigen; has no Bazel build.
http_archive(
    name = "com_gitlab_libeigen_eigen",
    sha256 = "0215c6593c4ee9f1f7f28238c4e8995584ebf3b556e9dbf933d84feb98d5b9ef",
    strip_prefix = "eigen-3.3.8",
    urls = [
        "https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.bz2",
    ],
    build_file_content =
"""
# TODO(keir): Replace this with a better version, like from TensorFlow.
# See https://github.com/ceres-solver/ceres-solver/issues/337.
cc_library(
    name = 'eigen',
    srcs = [],
    includes = ['.'],
    hdrs = glob(['Eigen/**']),
    visibility = ['//visibility:public'],
)
"""
)

# External dependency: Google Benchmark; has no Bazel build.
http_archive(
    name = "com_github_google_benchmark",
    urls = ["https://github.com/google/benchmark/archive/56f52ee228783547f544d9ac4a533574b9010e3f.zip"],
    sha256 = "8c1c6e90cd320b07504fabb86400f390faff2e599183ebd9396908817968ae79",
    strip_prefix = "benchmark-56f52ee228783547f544d9ac4a533574b9010e3f",
    build_file_content =
"""
cc_library(
    name = "benchmark",
    srcs = glob([
        "src/*.h",
        "src/*.cc",
    ]),
    hdrs = glob(["include/benchmark/*.h"]),
    copts = [
        "-DHAVE_STD_REGEX",
    ],
    includes = [
        "include",
    ],
    visibility = ["//visibility:public"],
)
"""
)