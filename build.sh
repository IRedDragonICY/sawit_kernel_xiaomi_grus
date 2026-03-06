#!/bin/bash
set -e

# Parameters
VARIANT="${1:-nonroot}"
KERNEL_DIR="$(pwd)"
ANYKERNEL_DIR="${KERNEL_DIR}/AnyKernel3"
OUT_DIR="${KERNEL_DIR}/out"

# Configuration
BASE_CONFIG="sdm670-perf_defconfig"
XIAOMI_CONFIGS="xiaomi/sdm710-common.config xiaomi/grus.config"

if [ "$VARIANT" = "ksu" ] || [ "$VARIANT" = "root" ]; then
    ZIP_NAME="Sawit-Kernel-SDM710-KSUNext-susfs.zip"
    FINAL_CONFIGS="$BASE_CONFIG $XIAOMI_CONFIGS ksu_next.config"
    echo "[*] Selected Variant: Root (KernelSU-Next)"
else
    ZIP_NAME="Sawit-Kernel-SDM710-NonRoot.zip"
    FINAL_CONFIGS="$BASE_CONFIG $XIAOMI_CONFIGS nonroot.config"
    echo "[*] Selected Variant: Non-Root (Stable/Clean)"
fi

echo "[*] Cleaning previous build artifacts..."
mkdir -p "$OUT_DIR"
rm -f "${OUT_DIR}/arch/arm64/boot/Image.gz-dtb"
rm -f "${ANYKERNEL_DIR}/Sawit-Kernel-SDM710-"*.zip
rm -f "${OUT_DIR}/.config" # Force config regeneration

echo "[*] Configuring kernel..."
make O="${OUT_DIR}" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- $FINAL_CONFIGS
make O="${OUT_DIR}" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- olddefconfig

echo "[*] Building kernel..."
make O="${OUT_DIR}" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j$(nproc --all) V=0 Image.gz-dtb

if [ -f "${OUT_DIR}/arch/arm64/boot/Image.gz-dtb" ]; then
    echo "[*] Kernel built successfully!"
    echo "[*] Packaging zip..."
    cp "${OUT_DIR}/arch/arm64/boot/Image.gz-dtb" "${ANYKERNEL_DIR}/"
    cat "${OUT_DIR}/arch/arm64/boot/dts/qcom/sdm670.dtb" "${OUT_DIR}/arch/arm64/boot/dts/qcom/sdm710.dtb" > "${ANYKERNEL_DIR}/dtb"
    cd "${ANYKERNEL_DIR}"
    rm -f AnyKernel3-*.zip
    zip -r9 "${ZIP_NAME}" * -x .git README.md *placeholder
    mv "${ZIP_NAME}" "${KERNEL_DIR}/"
    echo "[*] Done! Zip saved at: ${KERNEL_DIR}/${ZIP_NAME}"
else
    echo "[!] Build failed!"
    exit 1
fi
