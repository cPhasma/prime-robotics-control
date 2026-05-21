"""
Проверка преобразования нормированной команды управления в инверсный PWM.

В проекте используется схема:
    u = 0.0 -> PWM = 255 -> стоп
    u = 1.0 -> PWM = 0   -> максимальная скорость

Файл относится к вспомогательным проверкам аппаратной части.
"""

from __future__ import annotations


PWM_MIN = 0
PWM_MAX = 255


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    """Ограничивает значение команды управления диапазоном [0; 1]."""
    return max(low, min(high, float(value)))


def norm_to_inverted_pwm(u: float) -> int:
    """
    Переводит нормированную команду управления в PWM.

    Принятая логика:
        u = 0 -> PWM = 255
        u = 1 -> PWM = 0
    """
    u_clamped = clamp(u)
    return int(round(PWM_MAX * (1.0 - u_clamped)))


def run_table_check() -> None:
    """Печатает таблицу контрольных значений."""
    test_values = [-0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5]

    print("Inverted PWM mapping check")
    print("-" * 34)
    print(f"{'u':>8} | {'PWM':>5} | note")
    print("-" * 34)

    for value in test_values:
        pwm = norm_to_inverted_pwm(value)
        note = ""
        if value < 0.0 or value > 1.0:
            note = "clamped"
        print(f"{value:8.2f} | {pwm:5d} | {note}")

    assert norm_to_inverted_pwm(0.0) == 255
    assert norm_to_inverted_pwm(1.0) == 0
    assert 120 <= norm_to_inverted_pwm(0.5) <= 135

    print("-" * 34)
    print("PWM mapping check passed.")


if __name__ == "__main__":
    run_table_check()
