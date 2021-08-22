FROM ubuntu:20.10 as base

RUN apt update && apt install -y curl gnupg

RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg \
    && mv bazel.gpg /etc/apt/trusted.gpg.d/ \
    && echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list \
    && apt update && apt install -y bazel

RUN apt install -y build-essential git unzip zip

RUN echo "build --cxxopt='-std=c++14'" > ~/.bazelrc

WORKDIR /gcs-cpp
COPY . .

FROM base as build

RUN bazel build //:main
