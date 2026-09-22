#!/usr/bin/env bash

sudo apt update && sudo apt install -y \
    rsync  ninja-build \
    libfmt-dev libceres-dev libeigen3-dev \
    nlohmann-json3-dev libyaml-cpp-dev
