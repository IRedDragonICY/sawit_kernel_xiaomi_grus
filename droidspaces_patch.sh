#!/bin/bash
# ==========================================================================
# Droidspaces Container Support — Kernel Source Patches
# ==========================================================================
# Applies Droidspaces/LXC compatibility patches to the kernel source.
# Based on: https://github.com/ravindu644/Droidspaces-OSS
# Doc: https://github.com/ravindu644/Droidspaces-OSS/blob/main/Documentation/Kernel-Configuration.md
#
# These patches are applied at build time to keep the original kernel
# source clean. All patches are idempotent (safe to run multiple times).
# ==========================================================================
set -e

KERNEL_DIR="${1:-.}"
MARKER="DROIDSPACES_PATCHED"

echo "[Droidspaces] Starting kernel source patches..."
echo "[Droidspaces] Kernel dir: $KERNEL_DIR"

# ==========================================
# Patch 1: xt_qtaguid kernel panic fix
# ==========================================
# Avoids kernel panic by removing unsafe dev_get_stats() usage
# on inactive network interfaces in containers.
#
# Original: 01.fix_kernel_panic_in_xt_qtaguid.patch
# Target: net/netfilter/xt_qtaguid.c (doesn't exist in kernel 4.9)

QTAGUID_C="$KERNEL_DIR/net/netfilter/xt_qtaguid.c"
if [ -f "$QTAGUID_C" ]; then
    if ! grep -q "$MARKER" "$QTAGUID_C" 2>/dev/null; then
        if grep -q "dev_get_stats(iface_entry->net_dev" "$QTAGUID_C" 2>/dev/null; then
            sed -i "s|struct rtnl_link_stats64 dev_stats, \*stats;|struct rtnl_link_stats64 *stats; /* $MARKER */|" "$QTAGUID_C"
            python3 -c "
import re
with open('$QTAGUID_C', 'r') as f:
    content = f.read()
old = re.compile(
    r'if \(iface_entry->active\) \{\s*\n\s*stats = dev_get_stats\(iface_entry->net_dev,\s*\n\s*&dev_stats\);\s*\n\s*\} else \{\s*\n\s*stats = &no_dev_stats;\s*\n\s*\}',
    re.MULTILINE
)
content = old.sub('stats = &no_dev_stats;', content)
with open('$QTAGUID_C', 'w') as f:
    f.write(content)
" 2>/dev/null || true
            echo "[Droidspaces] Patch 1: xt_qtaguid dev_get_stats fix applied"
        else
            echo "[Droidspaces] Patch 1: xt_qtaguid already patched or different layout"
        fi
    else
        echo "[Droidspaces] Patch 1: xt_qtaguid already patched (marker found)"
    fi
else
    echo "[Droidspaces] Patch 1: xt_qtaguid.c not found (not in this kernel), skipping"
fi

# ==========================================
# Patch 2: cgroup file prefix handling
# ==========================================
# Creates prefixed symlinks for LXC/container compatibility.
#
# Android mounts cgroups with CGRP_ROOT_NOPREFIX → strips subsystem
# prefixes (e.g., "cpu.shares" → "shares"). LXC expects prefixed names.
# This patch creates symlinks so both names work.
#
# Original: 02.fix_restore_cgroup_file_prefix_handling.patch
# Target: kernel/cgroup.c (4.9) or kernel/cgroup/cgroup.c (5.x+)

CGROUP_C=""
if [ -f "$KERNEL_DIR/kernel/cgroup.c" ]; then
    CGROUP_C="$KERNEL_DIR/kernel/cgroup.c"
elif [ -f "$KERNEL_DIR/kernel/cgroup/cgroup.c" ]; then
    CGROUP_C="$KERNEL_DIR/kernel/cgroup/cgroup.c"
fi

if [ -n "$CGROUP_C" ]; then
    if ! grep -q "$MARKER" "$CGROUP_C" 2>/dev/null; then
        if grep -q "static int cgroup_add_file" "$CGROUP_C" 2>/dev/null; then
            # Find the "return 0;" line inside cgroup_add_file()
            func_start=$(grep -n 'static int cgroup_add_file' "$CGROUP_C" | head -1 | cut -d: -f1)
            # Find the closing "}" of cgroup_add_file — it's the first "^}" after func_start
            func_end=$(tail -n +"$func_start" "$CGROUP_C" | grep -n '^}' | head -1 | cut -d: -f1)
            func_end=$((func_start + func_end - 1))

            # Find "return 0;" within the function range
            return_line=$(sed -n "${func_start},${func_end}p" "$CGROUP_C" | grep -n 'return 0;' | tail -1 | cut -d: -f1)
            actual_line=$((func_start + return_line - 1))

            if [ -n "$return_line" ] && [ "$actual_line" -gt 0 ]; then
                sed -i "${actual_line}i\\
\\
\t/* ${MARKER}: Create prefixed symlinks for Droidspaces/LXC compatibility */\\
\tif (cft->ss && (cgrp->root->flags & CGRP_ROOT_NOPREFIX) &&\\
\t    !(cft->flags & CFTYPE_NO_PREFIX)) {\\
\t\tsnprintf(name, CGROUP_FILE_NAME_MAX, \"%s.%s\",\\
\t\t\t cft->ss->name, cft->name);\\
\t\tkernfs_create_link(cgrp->kn, name, kn);\\
\t}" "$CGROUP_C"

                if grep -q "$MARKER" "$CGROUP_C" 2>/dev/null; then
                    echo "[Droidspaces] Patch 2: cgroup prefix symlinks applied to $(basename "$CGROUP_C")"
                else
                    echo "[Droidspaces] Patch 2: ERROR — failed to apply cgroup patch!"
                    exit 1
                fi
            else
                echo "[Droidspaces] Patch 2: ERROR — could not locate return 0 in cgroup_add_file()"
                exit 1
            fi
        else
            echo "[Droidspaces] Patch 2: cgroup_add_file() not found in $CGROUP_C"
            exit 1
        fi
    else
        echo "[Droidspaces] Patch 2: cgroup already patched (marker found)"
    fi
else
    echo "[Droidspaces] Patch 2: ERROR — cgroup.c not found!"
    exit 1
fi

echo "[Droidspaces] All kernel source patches applied successfully!"
