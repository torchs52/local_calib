#!/usr/bin/env bash

set -eu

FALLBACK_EXTRA='general'

print_fallback_and_exit() {
    printf '%s\n' "${FALLBACK_EXTRA}"
    exit 0
}

is_jetson() {
    [ -f /etc/nv_tegra_release ] && [ "$(uname -m)" = "aarch64" ]
}

detect_l4t_version() {
    dpkg-query --showformat='${Version}' --show nvidia-l4t-core 2>/dev/null | cut -d- -f1
}

select_extra_for_l4t() {
    local l4t_version="${1}"
    if dpkg --compare-versions "${l4t_version}" ge 36.4; then
        printf '%s\n' jetpack
    elif dpkg --compare-versions "${l4t_version}" ge 36.3; then
        printf '%s\n' jetpack60
    else
        echo "Unsupported Jetson L4T version: ${l4t_version}" >&2
        exit 1
    fi
}

if ! is_jetson; then
    print_fallback_and_exit
fi

L4T_VERSION="$(detect_l4t_version)"
if [ -z "${L4T_VERSION}" ]; then
    print_fallback_and_exit
fi

select_extra_for_l4t "${L4T_VERSION}"
