FROM ubuntu:20.04 as base

RUN apt update && apt install -y curl gnupg

RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg \
    && mv bazel.gpg /etc/apt/trusted.gpg.d/ \
    && echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list \
    && apt update && apt install -y bazel

RUN apt update && apt install -y build-essential git unzip zip clang-format

# TODO: these commands work from inside the container, just not when building?
# WORKDIR /usr/local/bin
# RUN curl -fsSL https://github.com/bazelbuild/buildtools/releases/download/4.0.1/buildifier-linux-amd64 -o buildifier \
#     && chmod +x buildifer
