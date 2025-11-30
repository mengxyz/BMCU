#!/usr/bin/env python3
# GPT 5 Generated
import configparser
import subprocess
import shutil
import sys
from pathlib import Path

def find_fw_envs(ini_path: Path):
    """Return list of env names starting with fw-"""
    config = configparser.ConfigParser()
    config.optionxform = str  # keep case
    with ini_path.open("r", encoding="utf-8") as f:
        config.read_file(f)

    envs = []
    for section in config.sections():
        if section.startswith("env:fw-"):
            env_name = section.split("env:", 1)[1]
            envs.append(env_name)
    return envs

def run_pio_build(env: str):
    print(f"\n=== Building {env} ===")
    cmd = ["pio", "run", "-e", env]
    result = subprocess.run(cmd)
    return result.returncode == 0

def copy_firmware(env: str, out_dir: Path):
    build_dir = Path(".pio") / "build" / env
    src = build_dir / "firmware.bin"

    if not src.exists():
        bins = list(build_dir.glob("*.bin"))
        if not bins:
            print(f"!! No firmware.bin found for {env}")
            return False
        src = bins[0]

    out_dir.mkdir(exist_ok=True)
    dst = out_dir / f"{env}.bin"
    shutil.copy2(src, dst)
    print(f"   Copied -> {dst}")
    return True

def main():
    ini_path = Path("platformio.ini")
    if not ini_path.exists():
        print("platformio.ini not found.")
        sys.exit(1)

    out_dir = Path("out")

    envs = find_fw_envs(ini_path)
    if not envs:
        print("No fw-* environments found.")
        sys.exit(1)

    print("Found fw-* envs:")
    for e in envs:
        print(" -", e)

    ok = []
    fail = []

    for env in envs:
        if not run_pio_build(env):
            fail.append(env)
            continue
        if copy_firmware(env, out_dir):
            ok.append(env)
        else:
            fail.append(env)

    print("\n=== Summary ===")
    for e in ok:
        print("✔", e)
    for e in fail:
        print("✘", e)

    sys.exit(0 if not fail else 1)

if __name__ == "__main__":
    main()
