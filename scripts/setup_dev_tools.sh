#!/bin/bash
# Build tools, Python hardware libraries, CLI utilities used across phases, and
# the pinned Python linters behind `racecar lint`.
set -eo pipefail

sudo apt-get install -y \
    build-essential cmake pkg-config ccache \
    python3-pip python3-venv python3-dev python3-setuptools python3-wheel \
    python3-serial python3-yaml python3-opencv python3-numpy python3-scipy \
    libcurl4-openssl-dev \
    git vim nano htop tmux screen tree jq bc less curl wget \
    v4l-utils usbutils lshw lsof pciutils \
    net-tools iputils-ping traceroute dnsutils

# Linters, pinned so every car agrees with pyproject.toml. Per-user install;
# the binaries land in ~/.local/bin, which `racecar lint` runs directly.
# Ubuntu 22.04's pip predates PEP 668, so there is no system-package guard to
# bypass here.
LINTERS=(ruff==0.16.8 black==26.5.1 mypy==2.3.1)
pip3 install --user "${LINTERS[@]}"
