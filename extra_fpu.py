# Custom settings, as referred to as "extra_script" in platformio.ini
#
# See http://docs.platformio.org/en/latest/projectconf.html#extra-script

import os

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

env.Append(
    LINKFLAGS=[
        "-mcpu=cortex-m4",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
    ]
)

board = env.BoardConfig()
platform = env.PioPlatform()
toolchain_dir = platform.get_package_dir("toolchain-gccarmnoneeabi")
framework_package = "%s_Firmware_Library" % board.get("build.bsp", "AT32F403A_407")
framework_dir = os.path.join(os.path.dirname(toolchain_dir), framework_package)
cmsis_dir = os.path.join(framework_dir, "libraries", "cmsis")
cmsis_dsp_dir = os.path.join(cmsis_dir, "dsp")

if not os.path.isdir(cmsis_dsp_dir):
    raise RuntimeError("CMSIS-DSP sources not found: %s" % cmsis_dsp_dir)

core_dir = "cm0plus" if board.get("build.cpu", "cortex-m4") == "cortex-m0+" else "cm4"
env.Append(
    CPPPATH=[
        os.path.join(cmsis_dir, core_dir, "core_support"),
        os.path.join(cmsis_dsp_dir, "include")
    ]
)

env.BuildSources(
    os.path.join("$BUILD_DIR", "cmsis_dsp", "fast_math"),
    os.path.join(cmsis_dsp_dir, "Source", "FastMathFunctions"),
    src_filter=[
        "+<arm_sin_f32.c>",
        "+<arm_cos_f32.c>",
        "+<arm_atan2_f32.c>"
    ]
)
env.BuildSources(
    os.path.join("$BUILD_DIR", "cmsis_dsp", "common_tables"),
    os.path.join(cmsis_dsp_dir, "Source", "CommonTables"),
    src_filter=["+<arm_common_tables.c>"]
)
