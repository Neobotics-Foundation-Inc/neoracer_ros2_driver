#!/bin/bash
# Build tools, Python hardware libraries, and CLI utilities used across phases.
set -eo pipefail

sudo apt-get install -y \
    build-essential cmake pkg-config ccache \
    python3-pip python3-venv python3-dev python3-setuptools python3-wheel \
    python3-serial python3-yaml python3-opencv python3-numpy python3-scipy \
    libcurl4-openssl-dev \
    git vim nano htop tmux screen tree jq bc less curl wget \
    v4l-utils usbutils lshw lsof pciutils \
    net-tools iputils-ping traceroute dnsutils
