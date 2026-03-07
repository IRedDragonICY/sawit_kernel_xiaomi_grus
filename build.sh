#!/bin/bash
set -e

KERNEL_DIR="$(pwd)"
ANYKERNEL_DIR="${KERNEL_DIR}/AnyKernel3"
OUT_DIR="${KERNEL_DIR}/out"

# Configuration
BASE_CONFIG="sdm670-perf_defconfig"
XIAOMI_CONFIGS="xiaomi/sdm710-common.config xiaomi/grus.config"

# ── Toolchain Selection ─────────────────────────────────────────────
# CLANG_OPT: empty = GCC (default), "clang" = Clang extreme optimization
CLANG_OPT=""

setup_gcc_toolchain() {
    MAKE_FLAGS=(
        O="${OUT_DIR}"
        ARCH=arm64
        CROSS_COMPILE=aarch64-linux-gnu-
    )
    echo "[*] Toolchain: GCC ($(aarch64-linux-gnu-gcc --version | head -1))"
}

setup_clang_toolchain() {
    # ── SDM710 Micro-Architecture: Kryo 360 ────────────────────────
    # Kryo 360 Gold = Cortex-A75 (2x, big cores, ARMv8.2-A)
    # Kryo 360 Silver = Cortex-A55 (6x, LITTLE cores, ARMv8.2-A)
    # Tune for A75 (prime/big core handles latency-critical paths)
    #
    # -mcpu=cortex-a75  : Pipeline scheduling tuned for A75 OoO engine
    #                     Sets PrefLoopAlignment=Align(16), optimal L1i fetch
    # -march=armv8.2-a  : Full ARMv8.2-A ISA (implied by cortex-a75)
    #   +crypto         : HW AES/SHA — accelerates fscrypt, dm-crypt, WireGuard
    #   +dotprod        : Dot-product instructions for vectorized operations
    # -mtune=cortex-a75 : Instruction latency model for the big core

    # ── Target flags ──
    # -mcpu=cortex-a75  : Pipeline scheduling + instruction latency for A75 OoO
    # +crypto           : HW AES/SHA for fscrypt, dm-crypt, WireGuard
    # Note: +dotprod omitted — guarded by -mgeneral-regs-only anyway,
    #       and some Kryo 360 steppings may not advertise it to EL1
    local TARGET_FLAGS="-mcpu=cortex-a75+crypto -mtune=cortex-a75"

    # ── Optimization flags ──
    # -O3           : Aggressive inlining, loop unrolling, vectorization
    #                 (appended after Makefile's -O2; Clang uses the last one)
    # -faddrsig     : Emit address-significance table (harmless metadata)
    #
    # ── Polly polyhedral loop optimizer ──
    # -mllvm -polly                      : Enable Polly's advanced loop transformations
    # -mllvm -polly-vectorizer=stripmine : Strip-mine outer loops for vectorizer
    # -mllvm -polly-run-dce              : Run dead code elimination after Polly
    # -mllvm -polly-process-unprofitable : Optimize even "unprofitable" loops (we want max perf)
    #
    # ── LLVM backend tuning ──
    # -mllvm --enable-gvn-hoist           : Hoist common expressions out of branches
    # -mllvm --inline-threshold=600       : More aggressive inlining (default=225)
    local OPT_FLAGS="-O3 -faddrsig"
    OPT_FLAGS+=" -mllvm -polly"
    OPT_FLAGS+=" -mllvm -polly-vectorizer=stripmine"
    OPT_FLAGS+=" -mllvm -polly-process-unprofitable"
    OPT_FLAGS+=" -mllvm --enable-gvn-hoist"
    OPT_FLAGS+=" -mllvm --inline-threshold=600"

    # NOTE: --gc-sections and --icf are intentionally NOT used.
    # --gc-sections deletes kernel sections referenced only by linker scripts
    # or assembly (.altinstructions, __ex_table, .pci_fixup) → instant panic.

    # ── Standalone LLVM 22 toolchain ──
    local LLVM22_DIR="$HOME/toolchains/LLVM-22.1.0-Linux-X64"
    if [ -d "$LLVM22_DIR" ]; then
        export PATH="$LLVM22_DIR/bin:$HOME/.local/bin:$PATH"
        # Fix libxml2.so.2 soname compat (Arch ships .so.16)
        export LD_LIBRARY_PATH="$LLVM22_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    else
        # Fallback to system Clang + gold for LTO
        export PATH="$HOME/.local/bin:$PATH"
    fi

    MAKE_FLAGS=(
        O="${OUT_DIR}"
        ARCH=arm64
        LLVM=1
        LLVM_IAS=1
        CROSS_COMPILE=aarch64-linux-gnu-
        "KCFLAGS=${TARGET_FLAGS} ${OPT_FLAGS}"
    )
    echo "[*] Toolchain: Clang $(clang --version | head -1)"
    echo "[*] Target:    Cortex-A75 (Kryo 360 Gold) / ARMv8.2-A+crypto"
    echo "[*] Optimize:  -O3 + Full LTO + Polly + GVN hoist + inline=600"
}

# ── Build a single variant ──────────────────────────────────────────
build_variant() {
    local variant="$1"
    local zip_name final_configs

    case "$variant" in
        nonroot)
            zip_name="Sawit-Kernel-SDM710-NonRoot.zip"
            final_configs="$BASE_CONFIG $XIAOMI_CONFIGS nonroot.config"
            VARIANT_SUFFIX="-sawit"
            echo ""
            echo "============================================"
            echo "[*] Variant: Non-Root (Stable/Clean)"
            echo "============================================"
            ;;
        ksun)
            zip_name="Sawit-Kernel-SDM710-KSUNext.zip"
            final_configs="$BASE_CONFIG $XIAOMI_CONFIGS ksu_next_nosusfs.config"
            VARIANT_SUFFIX="-sawit-ksun"
            echo ""
            echo "============================================"
            echo "[*] Variant: KernelSU-Next"
            echo "============================================"
            ;;
        ksun_susfs|ksu)
            zip_name="Sawit-Kernel-SDM710-KSUNext-susfs.zip"
            final_configs="$BASE_CONFIG $XIAOMI_CONFIGS ksu_next.config"
            VARIANT_SUFFIX="-sawit-ksun-susfs"
            echo ""
            echo "============================================"
            echo "[*] Variant: KernelSU-Next + SUSFS"
            echo "============================================"
            ;;
        *)
            echo "[!] Unknown variant: $variant"
            echo "Usage: $0 [--clang] {nonroot|ksun|ksun_susfs|all}"
            return 1
            ;;
    esac

    # Append extreme_clang.config when using Clang toolchain
    if [ -n "$CLANG_OPT" ]; then
        final_configs="$final_configs extreme_clang.config"
        VARIANT_SUFFIX="${VARIANT_SUFFIX}-lto"
    fi

    # Build dynamic EXTRAVERSION: e.g. -sawit-ksun-susfs-lto
    # Use a local copy so successive variant builds don't accumulate flags
    local EXTRA="${VARIANT_SUFFIX}"
    local -a BUILD_FLAGS=("${MAKE_FLAGS[@]}" "EXTRAVERSION=${EXTRA}")
    echo "[*] Version:   4.9.337${EXTRA}"

    echo "[*] Cleaning previous build artifacts..."
    mkdir -p "$OUT_DIR"
    rm -f "${OUT_DIR}/arch/arm64/boot/Image.gz-dtb"
    rm -f "${OUT_DIR}/.config" # Force config regeneration

    echo "[*] Configuring kernel..."
    make "${BUILD_FLAGS[@]}" $final_configs
    make "${BUILD_FLAGS[@]}" olddefconfig

    echo "[*] Building kernel..."
    make "${BUILD_FLAGS[@]}" -j$(nproc --all) V=0 Image.gz-dtb

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
# Parse --clang flag
for arg in "$@"; do
    case "$arg" in
        --clang) CLANG_OPT="clang" ;;
    esac
done

# Setup toolchain
if [ -n "$CLANG_OPT" ]; then
    setup_clang_toolchain
else
    setup_gcc_toolchain
fi

# Get variant (skip --clang flag)
VARIANT=""
for arg in "$@"; do
    case "$arg" in
        --clang) ;;
        *) VARIANT="$arg" ;;
    esac
done
VARIANT="${VARIANT:-nonroot}"

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
