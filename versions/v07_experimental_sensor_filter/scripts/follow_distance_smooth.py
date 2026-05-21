import time

# =========================
# Smooth follow controller
# =========================
# Идея:
# 1) фильтруем дистанцию и gyro_z
# 2) делаем мягкую оценку производной дистанции
# 3) держим среднюю тягу по дистанции
# 4) стабилизируем курс по углу и угловой скорости
# 5) ограничиваем скорость изменения управляющего сигнала и PWM
# 6) при плохих измерениях аккуратно останавливаемся

DT = 0.10
STEPS = 400
DESIRED_DISTANCE_CM = 30.0

# Базовое движение
BASE_U = 0.27
MIN_CRUISE_U = 0.18
MAX_U = 0.72

# Продольный канал
KP_DIST = 0.018
KD_DIST = 0.020
DIST_DEADBAND_CM = 1.5
MAX_APPROACH_RATE = 80.0   # ограничение производной ошибки, см/с

# Курсовой канал
K_THETA = 0.55
K_WZ = 0.20
TURN_DEADBAND = 0.015
MAX_TURN = 0.18

# Фильтры
DIST_ALPHA = 0.35          # EMA для distance
GYRO_ALPHA = 0.30          # EMA для gyro_z
DERIV_ALPHA = 0.82         # EMA для производной ошибки
THETA_LEAK = 0.997         # небольшая утечка для борьбы с дрейфом

# Ограничения плавности
MAX_US_STEP = 0.035        # изменение средней тяги за шаг
MAX_UD_STEP = 0.030        # изменение поворотной добавки за шаг
MAX_PWM_STEP = 12          # изменение PWM за шаг

# Надёжность датчиков
MAX_VALID_DISTANCE_CM = 250.0
MIN_VALID_DISTANCE_CM = 2.0
MAX_CONSECUTIVE_BAD = 6
CALIBRATION_SAMPLES = 20

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_deadband(x, band):
    if abs(x) <= band:
        return 0.0
    return x - band if x > 0 else x + band

def slew(current, target, max_step):
    delta = clamp(target - current, -max_step, max_step)
    return current + delta

def norm_to_pwm(u):
    u = clamp(u, 0.0, 1.0)
    return int(round(255 * (1.0 - clamp(u, 0.0, 1.0))))

def ramp_pwm(current, target, max_step):
    return int(round(slew(float(current), float(target), float(max_step))))

def stop_motors():
    motor(left_pwm=255, right_pwm=255, left_dir="forward", right_dir="forward")

def is_valid_distance(d):
    return MIN_VALID_DISTANCE_CM <= d <= MAX_VALID_DISTANCE_CM

def read_distance_cm():
    data = duration()
    value = float(data.get("distance_cm", -1.0))
    return value

def read_gyro_z():
    data = mpu6050()
    return float(data.get("gyro_z", 0.0))

def calibrate_gyro_bias(samples=CALIBRATION_SAMPLES):
    print(f"[calibration] gyro bias, samples={samples}")
    acc = 0.0
    ok = 0
    for _ in range(samples):
        wz = read_gyro_z()
        acc += wz
        ok += 1
        time.sleep(0.03)
    bias = acc / max(ok, 1)
    print(f"[calibration] gyro_bias={bias:.5f} rad/s")
    return bias

def prime_distance_filter(max_attempts=20):
    for _ in range(max_attempts):
        d = read_distance_cm()
        if is_valid_distance(d):
            print(f"[init] distance={d:.2f} cm")
            return d
        time.sleep(0.05)
    raise RuntimeError("No valid HC-SR04 distance during init")

# Калибровка перед стартом: машинка должна стоять спокойно
gyro_bias = calibrate_gyro_bias()
d_raw = prime_distance_filter()
d_f = d_raw
wz_f = 0.0
theta = 0.0
prev_e = d_f - DESIRED_DISTANCE_CM
e_dot_f = 0.0

# Предыдущие управляющие значения
u_s = 0.0
u_delta = 0.0
pwm_l = 255
pwm_r = 255
bad_distance_count = 0

print("[run] smooth follower started")

for step in range(STEPS):
    d_raw = read_distance_cm()
    wz_raw = read_gyro_z() - gyro_bias

    if is_valid_distance(d_raw):
        bad_distance_count = 0
        d_f = DIST_ALPHA * d_raw + (1.0 - DIST_ALPHA) * d_f
    else:
        bad_distance_count += 1
        print(f"[warn] invalid distance={d_raw:.2f} count={bad_distance_count}")
        # кратковременно держим прошлое значение, а при серии плохих чтений уходим в стоп
        if bad_distance_count >= MAX_CONSECUTIVE_BAD:
            print("[safe] too many invalid distance reads -> stop")
            break

    wz_f = GYRO_ALPHA * wz_raw + (1.0 - GYRO_ALPHA) * wz_f
    theta = THETA_LEAK * (theta + wz_f * DT)

    e = d_f - DESIRED_DISTANCE_CM
    e_db = apply_deadband(e, DIST_DEADBAND_CM)

    raw_e_dot = (e - prev_e) / DT
    raw_e_dot = clamp(raw_e_dot, -MAX_APPROACH_RATE, MAX_APPROACH_RATE)
    e_dot_f = DERIV_ALPHA * e_dot_f + (1.0 - DERIV_ALPHA) * raw_e_dot

    # Продольный канал
    target_u_s = BASE_U + KP_DIST * e_db + KD_DIST * e_dot_f

    # Если мы уже слишком близко, разрешаем полностью останавливаться
    if e < -4.0:
        target_u_s = min(target_u_s, 0.0)

    # Если мы около цели, держим очень мягкий ход без раскачки
    if abs(e) <= DIST_DEADBAND_CM:
        target_u_s = min(target_u_s, BASE_U)

    target_u_s = clamp(target_u_s, 0.0, MAX_U)
    if 0.0 < target_u_s < MIN_CRUISE_U:
        target_u_s = MIN_CRUISE_U

    # Курсовой канал
    target_u_delta = -K_THETA * theta - K_WZ * wz_f
    target_u_delta = apply_deadband(target_u_delta, TURN_DEADBAND)
    target_u_delta = clamp(target_u_delta, -MAX_TURN, MAX_TURN)

    # Ограничение скорости изменения команд
    u_s = slew(u_s, target_u_s, MAX_US_STEP)
    u_delta = slew(u_delta, target_u_delta, MAX_UD_STEP)

    # Итог на борта
    u_l = clamp(u_s - u_delta, 0.0, 1.0)
    u_r = clamp(u_s + u_delta, 0.0, 1.0)

    target_pwm_l = norm_to_pwm(u_l)
    target_pwm_r = norm_to_pwm(u_r)

    pwm_l = ramp_pwm(pwm_l, target_pwm_l, MAX_PWM_STEP)
    pwm_r = ramp_pwm(pwm_r, target_pwm_r, MAX_PWM_STEP)

    motor(
        left_pwm=pwm_l,
        right_pwm=pwm_r,
        left_dir="forward",
        right_dir="forward"
    )

    print(
        f"step={step} "
        f"d_raw={d_raw:.2f} d_f={d_f:.2f} "
        f"e={e:.2f} e_dot={e_dot_f:.2f} "
        f"wz={wz_f:.4f} theta={theta:.4f} "
        f"us={u_s:.3f} ud={u_delta:.3f} "
        f"pwm=({pwm_l},{pwm_r})"
    )

    prev_e = e
    time.sleep(DT)

stop_motors()
print("[run] smooth follower finished")
