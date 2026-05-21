"""
Общий запуск вспомогательных проверок аппаратной части.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys


def run_script(script: Path, *args: str) -> None:
    print()
    print("=" * 60)
    print(f"Running: {script.name}")
    print("=" * 60)
    subprocess.run([sys.executable, str(script), *args], check=True)


def main() -> None:
    current_dir = Path(__file__).resolve().parent

    run_script(current_dir / "pwm_mapping_check.py")
    run_script(
        current_dir / "sensor_log_analyzer.py",
        str(current_dir / "sample_sensor_log.csv"),
    )

    print()
    print("All auxiliary hardware checks passed.")


if __name__ == "__main__":
    main()
