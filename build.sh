#!/bin/bash
set -e

KERNEL_DIR="$(pwd)"
ANYKERNEL_DIR="${KERNEL_DIR}/AnyKernel3"
OUT_DIR="${KERNEL_DIR}/out"

# Configuration
BASE_CONFIG="sdm670-perf_defconfig"
XIAOMI_CONFIGS="xiaomi/sdm710-common.config xiaomi/grus.config"

# ── Build a single variant ──────────────────────────────────────────
build_variant() {
    local variant="$1"
    local zip_name final_configs

    case "$variant" in
        nonroot)
            zip_name="Sawit-Kernel-SDM710-NonRoot.zip"
            final_configs="$BASE_CONFIG $XIAOMI_CONFIGS nonroot.config"
            echo ""
            echo "============================================"
            echo "[*] Variant: Non-Root (Stable/Clean)"
            echo "============================================"
            ;;
        ksun)
            zip_name="Sawit-Kernel-SDM710-KSUNext.zip"
            final_configs="$BASE_CONFIG $XIAOMI_CONFIGS ksu_next_nosusfs.config"
            echo ""
            echo "============================================"
            echo "[*] Variant: KernelSU-Next"
            echo "============================================"
            ;;
        ksun_susfs|ksu)
            zip_name="Sawit-Kernel-SDM710-KSUNext-susfs.zip"
            final_configs="$BASE_CONFIG $XIAOMI_CONFIGS ksu_next.config"
            echo ""
            echo "============================================"
            echo "[*] Variant: KernelSU-Next + SUSFS"
            echo "============================================"
            ;;
        *)
            echo "[!] Unknown variant: $variant"
            echo "Usage: $0 {nonroot|ksun|ksun_susfs|all}"
            return 1
            ;;
    esac

    echo "[*] Cleaning previous build artifacts..."
    mkdir -p "$OUT_DIR"
    rm -f "${OUT_DIR}/arch/arm64/boot/Image.gz-dtb"
    rm -f "${OUT_DIR}/.config" # Force config regeneration

    echo "[*] Configuring kernel..."
    make O="${OUT_DIR}" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- $final_configs
    make O="${OUT_DIR}" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- olddefconfig

    echo "[*] Building kernel..."
    make O="${OUT_DIR}" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j$(nproc --all) V=0 Image.gz-dtb

    if [ -f "${OUT_DIR}/arch/arm64/boot/Image.gz-dtb" ]; then
        echo "[*] Kernel built successfully!"
        echo "[*] Packaging ${zip_name}..."
        cp "${OUT_DIR}/arch/arm64/boot/Image.gz-dtb" "${ANYKERNEL_DIR}/"
        cat "${OUT_DIR}/arch/arm64/boot/dts/qcom/sdm670.dtb" \
            "${OUT_DIR}/arch/arm64/boot/dts/qcom/sdm710.dtb" > "${ANYKERNEL_DIR}/dtb"
        cd "${ANYKERNEL_DIR}"
        rm -f AnyKernel3-*.zip
        zip -r9 "${zip_name}" * -x .git README.md *placeholder
        mv "${zip_name}" "${KERNEL_DIR}/"
        cd "${KERNEL_DIR}"
        echo "[*] Done! → ${KERNEL_DIR}/${zip_name}"
    else
        echo "[!] Build failed for variant: $variant"
        return 1
    fi
}

# ── Main ────────────────────────────────────────────────────────────
VARIANT="${1:-nonroot}"

if [ "$VARIANT" = "all" ]; then
    echo "========================================================"
    echo "[*] Building ALL variants: nonroot, ksun, ksun_susfs"
    echo "========================================================"
    FAILED=()
    for v in nonroot ksun ksun_susfs; do
        if build_variant "$v"; then
            echo "[✓] $v succeeded"
        else
            echo "[✗] $v failed"
            FAILED+=("$v")
        fi
    done
    echo ""
    echo "========================================================"
    if [ ${#FAILED[@]} -eq 0 ]; then
        echo "[*] All 3 variants built successfully!"
        ls -lh "${KERNEL_DIR}"/Sawit-Kernel-SDM710-*.zip
    else
        echo "[!] Failed variants: ${FAILED[*]}"
        exit 1
    fi
else
    build_variant "$VARIANT"
fi
