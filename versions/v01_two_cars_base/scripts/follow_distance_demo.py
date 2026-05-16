import time

# Базовый пример следования.
# Для реального стенда рекомендуется запускать follow_distance_smooth.py
# — там добавлены фильтрация, deadzone, slew-rate limit и аварийное поведение.

DT = 0.10
DESIRED_DISTANCE = 30.0
U0 = 0.28
KP_D = 0.015
KD_D = 0.012
K_THETA = 0.55
K_WZ = 0.20
DERIV_ALPHA = 0.85
THETA_LEAK = 0.998

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def norm_to_pwm(u):
    u = clamp(u, 0.0, 1.0)
    return int(u * 255)

prev_e_d = 0.0
e_d_dot_f = 0.0
theta = 0.0

for step in range(200):
    dist_data = duration()
    imu_data = mpu6050()

    d = float(dist_data.get("distance_cm", 999.0))
    wz = float(imu_data.get("gyro_z", 0.0))

    e_d = d - DESIRED_DISTANCE
    raw_deriv = (e_d - prev_e_d) / DT
    e_d_dot_f = DERIV_ALPHA * e_d_dot_f + (1.0 - DERIV_ALPHA) * raw_deriv
    theta = THETA_LEAK * (theta + wz * DT)

    u_s = U0 + KP_D * e_d + KD_D * e_d_dot_f
    u_delta = -K_THETA * theta - K_WZ * wz

    u_l = clamp(u_s - u_delta, 0.0, 1.0)
    u_r = clamp(u_s + u_delta, 0.0, 1.0)

    motor(
        left_pwm=norm_to_pwm(u_l),
        right_pwm=norm_to_pwm(u_r),
        left_dir="forward",
        right_dir="forward"
    )

    print(f"step={step} d={d:.2f}cm wz={wz:.3f} ul={u_l:.3f} ur={u_r:.3f}")
    prev_e_d = e_d
    time.sleep(DT)

motor(left_pwm=0, right_pwm=0, left_dir="forward", right_dir="forward")
