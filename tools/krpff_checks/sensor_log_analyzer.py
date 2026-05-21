"""
Анализ CSV-лога датчиков.

Ожидаемые поля:
    time_ms,distance_m,gyro_z

Пустые значения distance_m или gyro_z считаются пропусками.
Скрипт нужен для быстрой проверки качества входных данных перед запуском
регулятора следования.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import mean


@dataclass
class SensorLogStats:
    rows: int
    distance_missing: int
    gyro_missing: int
    avg_period_ms: float | None
    max_period_ms: float | None
    distance_missing_percent: float
    gyro_missing_percent: float


def _is_empty(value: str | None) -> bool:
    return value is None or str(value).strip() == ""


def _to_float_or_none(value: str | None) -> float | None:
    if _is_empty(value):
        return None
    try:
        return float(str(value).replace(",", "."))
    except ValueError:
        return None


def analyze_sensor_log(path: Path) -> SensorLogStats:
    if not path.exists():
        raise FileNotFoundError(f"Log file not found: {path}")

    times: list[float] = []
    distance_missing = 0
    gyro_missing = 0
    rows = 0

    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)

        required = {"time_ms", "distance_m", "gyro_z"}
        if not reader.fieldnames or not required.issubset(set(reader.fieldnames)):
            raise ValueError(
                "CSV must contain fields: time_ms, distance_m, gyro_z"
            )

        for row in reader:
            rows += 1

            time_ms = _to_float_or_none(row.get("time_ms"))
            if time_ms is not None:
                times.append(time_ms)

            if _to_float_or_none(row.get("distance_m")) is None:
                distance_missing += 1

            if _to_float_or_none(row.get("gyro_z")) is None:
                gyro_missing += 1

    periods = [
        times[index] - times[index - 1]
        for index in range(1, len(times))
        if times[index] >= times[index - 1]
    ]

    avg_period = mean(periods) if periods else None
    max_period = max(periods) if periods else None

    distance_percent = (distance_missing / rows * 100.0) if rows else 0.0
    gyro_percent = (gyro_missing / rows * 100.0) if rows else 0.0

    return SensorLogStats(
        rows=rows,
        distance_missing=distance_missing,
        gyro_missing=gyro_missing,
        avg_period_ms=avg_period,
        max_period_ms=max_period,
        distance_missing_percent=distance_percent,
        gyro_missing_percent=gyro_percent,
    )


def print_report(stats: SensorLogStats) -> None:
    print("Sensor log analysis")
    print("-" * 40)
    print(f"Rows:                    {stats.rows}")
    print(
        f"Distance missing:        {stats.distance_missing} "
        f"({stats.distance_missing_percent:.1f}%)"
    )
    print(
        f"Gyro missing:            {stats.gyro_missing} "
        f"({stats.gyro_missing_percent:.1f}%)"
    )

    if stats.avg_period_ms is not None:
        print(f"Average polling period:  {stats.avg_period_ms:.1f} ms")
    else:
        print("Average polling period:  not enough data")

    if stats.max_period_ms is not None:
        print(f"Max polling gap:         {stats.max_period_ms:.1f} ms")
    else:
        print("Max polling gap:         not enough data")

    print("-" * 40)

    if stats.distance_missing_percent > 10.0:
        print("Warning: distance sensor has many missing values.")

    if stats.max_period_ms is not None and stats.max_period_ms > 250.0:
        print("Warning: polling gap is high for distance control.")

    if stats.distance_missing_percent <= 10.0 and (
        stats.max_period_ms is None or stats.max_period_ms <= 250.0
    ):
        print("Log looks acceptable for basic controller checks.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze robot sensor CSV log.")
    parser.add_argument(
        "path",
        nargs="?",
        default=str(Path(__file__).with_name("sample_sensor_log.csv")),
        help="Path to CSV file with fields: time_ms,distance_m,gyro_z",
    )
    args = parser.parse_args()

    stats = analyze_sensor_log(Path(args.path))
    print_report(stats)


if __name__ == "__main__":
    main()
