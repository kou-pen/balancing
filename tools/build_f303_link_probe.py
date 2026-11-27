#!/usr/bin/env python3
"""F303CB向けのリンク検証。実GPIO初期化・書き込み機能なし。"""
from __future__ import annotations

import argparse
import pathlib
import re
import shutil
import struct
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
SDK_REV = "663d2baae3604244f90ed80d0d757df17e36ada9"
DRIVERS = {
    "Drivers/CMSIS/Device/ST/STM32F3xx": "5558e64e3675a1e1fcb1c71f468c7c407c1b1134",
    "Drivers/STM32F3xx_HAL_Driver": "953955afe65f89e60e556bbcdba752597f5da65d",
}


def run(argv):
    subprocess.run([str(x) for x in argv], check=True, cwd=ROOT)


def revision(path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=pathlib.Path, default=pathlib.Path.home() /
                        "STM32Cube/Repository/STM32Cube_FW_F3_V1.11.6")
    parser.add_argument("--toolchain", type=pathlib.Path,
                        help="arm-none-eabi-g++等を含むbinディレクトリ")
    args = parser.parse_args()
    sdk = args.sdk.resolve()
    if revision(sdk) != SDK_REV:
        raise SystemExit("F303 SDK revision mismatch")
    for sub, expected in DRIVERS.items():
        if revision(sdk / sub) != expected:
            raise SystemExit(f"SDK submodule mismatch: {sub}")
    # 異なる版や編集済みHALで合格した結果を再現可能な証拠にしない。
    for path in [sdk, *(sdk / sub for sub in DRIVERS)]:
        if subprocess.check_output(["git", "-C", str(path), "status", "--porcelain",
                                    "--untracked-files=no"], text=True).strip():
            raise SystemExit(f"SDK tracked files modified: {path}")
    toolchain = args.toolchain
    if toolchain is None:
        compiler = shutil.which("arm-none-eabi-g++")
        toolchain = (pathlib.Path(compiler).parent if compiler else
                     next(pathlib.Path("/opt/st").rglob("arm-none-eabi-g++")).parent)
    toolchain = toolchain.resolve()
    gcc = toolchain / "arm-none-eabi-gcc"
    gxx = toolchain / "arm-none-eabi-g++"
    out = ROOT / "build/f303-arm-link-probe"
    out.mkdir(parents=True, exist_ok=True)
    elf = out / "f303_link_probe_DO_NOT_FLASH.elf"
    # 古い成功生成物を今回の失敗時に残さない。
    elf.unlink(missing_ok=True)
    hal = sdk / "Drivers/STM32F3xx_HAL_Driver"
    cmsis = sdk / "Drivers/CMSIS"
    device = cmsis / "Device/ST/STM32F3xx"
    # ST公式のconfテンプレートから検証対象モジュールのみ有効化。
    # 生成物はbuild/内に限定し、CubeMX生成設定と混同しない。
    modules = ("ADC", "CAN", "CORTEX", "DMA", "FLASH", "GPIO", "EXTI",
               "IWDG", "PWR", "RCC", "TIM")
    conf = (hal / "Inc/stm32f3xx_hal_conf_template.h").read_text()
    conf = re.sub(r"^#define HAL_([A-Z0-9_]+)_MODULE_ENABLED\s*$",
                  lambda m: m[0] if m[1] in modules else "/* " + m[0].strip() + " */",
                  conf, flags=re.MULTILINE)
    (out / "stm32f3xx_hal_conf.h").write_text(conf)
    f303 = ROOT / "firmware/f303/balancing_f303/App"
    f446 = ROOT / "firmware/f446/balancing_f446/App"
    cpu = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    common = cpu + ["-Os", "-ffunction-sections", "-fdata-sections", "-DSTM32F303xC",
                    "-DUSE_HAL_DRIVER"]
    for directory in (out, hal / "Inc", cmsis / "Include", device / "Include",
                      f303 / "Inc", f446 / "Inc"):
        common += ["-I", str(directory)]
    c_files = [device / "Source/Templates/system_stm32f3xx.c"]
    c_files += [hal / "Src" / ("stm32f3xx_hal" + suffix + ".c") for suffix in
                ("", "_adc", "_adc_ex", "_can", "_cortex", "_dma", "_flash", "_flash_ex",
                 "_gpio", "_exti", "_iwdg", "_pwr", "_pwr_ex", "_rcc", "_rcc_ex",
                 "_tim", "_tim_ex")]
    cpp_files = sorted((f303 / "Src/balancing/power").glob("*.cpp"))
    cpp_files += [f446 / "Src/balancing/communication/can" / name for name in
                  ("PowerBoardCodec.cpp", "ProtocolId.cpp", "ProtocolPayload.cpp")]
    cpp_files += [ROOT / "tests/f303_arm_link_probe.cpp"]
    assembly = device / "Source/Templates/gcc/startup_stm32f303xc.s"
    objects = []
    for source in [*c_files, *cpp_files, assembly]:
        obj = out / (source.stem + ".o")
        flags = (["-std=c++17", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra", "-Werror"]
                 if source.suffix == ".cpp" else ["-std=c11"] if source.suffix == ".c" else [])
        run([gxx if source.suffix == ".cpp" else gcc, *common, *flags, "-c", source, "-o", obj])
        objects.append(obj)
        print(f"PASS compile {source.name}", flush=True)
    # HAL関数もリンク対象に残すが、probeのmainから呼ばない。
    retained = ["HAL_Init", "HAL_GPIO_Init", "HAL_ADC_Start_DMA", "HAL_CAN_Start",
                "HAL_TIM_Base_Start_IT", "HAL_IWDG_Init"]
    run([gxx, *cpu, "-nostartfiles", "--specs=nano.specs", "--specs=nosys.specs",
         "-T", ROOT / "tests/f303_arm_link_probe.ld", "-Wl,--gc-sections",
         "-Wl,--no-undefined", *["-Wl,--undefined=" + name for name in retained],
         "-Wl,-Map=" + str(out / "probe.map"), *objects, "-o", elf])
    nm = subprocess.check_output([toolchain / "arm-none-eabi-nm", "-C", elf], text=True)
    for name in ("Reset_Handler", "g_pfnVectors", "main", "PowerController::step",
                 "PowerMonitor::evaluate", "PowerCan::step", "BoardIO::evaluate", *retained):
        if name not in nm:
            raise SystemExit(f"missing linked symbol: {name}")
    # ベクタの実データを読み、リンク先と初期SP/Reset_Handlerを照合。
    vector = out / "vectors.bin"
    run([toolchain / "arm-none-eabi-objcopy", "-O", "binary", "-j", ".isr_vector", elf, vector])
    sp, reset = struct.unpack_from("<II", vector.read_bytes())
    entry = subprocess.check_output([toolchain / "arm-none-eabi-readelf", "-h", elf], text=True)
    entry_address = int(re.search(r"Entry point address:\s*(0x[0-9a-fA-F]+)", entry)[1], 16)
    sections = subprocess.check_output([toolchain / "arm-none-eabi-objdump", "-h", elf], text=True)
    match = re.search(r"^\s*\d+\s+\.isr_vector\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)", sections, re.MULTILINE)
    if not match or int(match[1], 16) != 0x08000000 or sp != 0x20008000 or reset != entry_address or not reset & 1:
        raise SystemExit("invalid F303 vector location / initial SP / Thumb reset")
    print(f"PASS vector: 0x08000000, SP=0x{sp:08x}, reset=0x{reset:08x}")
    print(entry)
    run([toolchain / "arm-none-eabi-size", elf])
    print("PASS: F303CB ARM link probe (not production firmware; DO NOT FLASH)")


if __name__ == "__main__":
    main()
