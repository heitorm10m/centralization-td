FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        git \
    && rm -rf /var/lib/apt/lists/*
