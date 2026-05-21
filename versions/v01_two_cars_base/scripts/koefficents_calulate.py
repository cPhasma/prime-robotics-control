import numpy as np
from scipy import signal
from scipy.linalg import solve_discrete_are
import warnings
warnings.filterwarnings('ignore')

# ==========================================
# 1. ПАРАМЕТРЫ СИСТЕМЫ
# ==========================================
tau = 0.2    # Постоянная времени привода [с]
K = 1.0      # Статический коэффициент усиления
W = 0.13     # Колея [м]
v0 = 0.3     # Номинальная скорость для линеаризации [м/с]
Ts = 0.01    # Период дискретизации [с] (100 Гц)

# ==========================================
# 2. НЕПРЕРЫВНЫЕ МАТРИЦЫ A, B, C
# ==========================================
# Состояние: [vL, vR, theta, z, d, xi]
A = np.array([
    [-1/tau,  0,       0,       0,       0,       0],
    [0,      -1/tau,   0,       0,       0,       0],
    [-1/W,    1/W,     0,       0,       0,       0],
    [0,       0,       v0,      0,       0,       0],
    [-0.5,   -0.5,     0,       0,       0,       0],
    [0,       0,       0,       0,       1,       0]   # d(xi)/dt = d
])

B = np.array([
    [K/tau, 0],
    [0,     K/tau],
    [0,     0],
    [0,     0],
    [0,     0],
    [0,     0]
])

# Измеряемые выходы: [d, theta, omega]
# omega = theta_dot = -vL/W + vR/W
C = np.array([
    [0,     0,       0,       0,       1,       0],   # d
    [0,     0,       1,       0,       0,       0],   # theta
    [-1/W,  1/W,     0,       0,       0,       0]    # omega
])

# ==========================================
# 3. ДИСКРЕТИЗАЦИЯ (ZOH)
# ==========================================
D = np.zeros((3, 2))  # Матрица прямых связей
sys_disc = signal.cont2discrete((A, B, C, D), Ts, method='zoh')
Ad, Bd, Cd, Dd, _ = sys_disc

print("✅ Система дискретизирована (Ts = {:.3f} с)".format(Ts))

# ==========================================
# 4. РАСЧЁТ РЕГУЛЯТОРА (LQR)
# ==========================================
# Веса: diag(vL, vR, theta, z, d, xi)
Q = np.diag([0.1, 0.1, 5.0, 10.0, 2.0, 20.0])
R = np.diag([0.5, 0.5])  # Штраф за управление

P = solve_discrete_are(Ad, Bd, Q, R)
K_c = np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad

print("✅ Матрица регулятора K_c (2x6) найдена.")

# ==========================================
# 5. ЧАСТИЧНЫЙ НАБЛЮДАТЕЛЬ (только для vL, vR)
# ==========================================
print("\n=== ЧАСТИЧНЫЙ НАБЛЮДАТЕЛЬ ===")

# Упрощаем систему: оцениваем только vL и vR
# Измерения: omega (гироскоп) и d_dot (численная производная d)

# Матрица C для частичного наблюдателя (измеряем omega и d)
C_partial = np.array([
    [-1/W,  1/W],   # omega = (vR - vL) / W
    [-0.5, -0.5]    # d_dot = -(vL + vR) / 2
])

# Подматрица A для vL, vR
A_v = np.array([
    [-1/tau,  0],
    [0,      -1/tau]
])

# Подматрица B для vL, vR
B_v = np.array([
    [K/tau, 0],
    [0,     K/tau]
])

# Дискретизация частичной системы
sys_v_cont = (A_v, B_v, C_partial, np.zeros((2, 2)))
sys_v_disc = signal.cont2discrete(sys_v_cont, Ts, method='zoh')
Ad_v, Bd_v, Cd_v, _, _ = sys_v_disc

print("✅ Частичная система дискретизирована")

# Проверяем наблюдаемость частичной системы
obs_partial = np.vstack([
    Cd_v,
    Cd_v @ Ad_v
])
rank_partial = np.linalg.matrix_rank(obs_partial)
print(f"Ранг наблюдаемости частичной системы: {rank_partial}/2")

if rank_partial == 2:
    print("✅ Частичная система полностью наблюдаема!")
    
    # Расчёт наблюдателя для vL, vR через размещение полюсов
    # Полюса должны быть быстрее регулятора (0.3-0.7)
    desired_poles_v = np.array([0.5, 0.6])
    
    try:
        L_v = signal.place_poles(Ad_v.T, Cd_v.T, desired_poles_v, method='KNV0').gain_matrix.T
        print(f"✅ Матрица частичного наблюдателя L_v (2x2) найдена:")
        print(f"L_v = \n{L_v}")
    except Exception as e:
        print(f"⚠️  place_poles не сработал: {e}")
        print("Используем эвристические коэффициенты...")
        L_v = np.array([
            [0.3, -0.2],
            [-0.3, -0.2]
        ])
else:
    print("⚠️  Частичная система не наблюдаема! Используем эвристику...")
    L_v = np.array([
        [0.3, -0.2],
        [-0.3, -0.2]
    ])

# Проверяем устойчивость частичного наблюдателя
eigvals_v = np.linalg.eigvals(Ad_v - L_v @ Cd_v)
print(f"Полюса частичного наблюдателя: {np.abs(eigvals_v)}")

if np.all(np.abs(eigvals_v) < 1.0):
    print("✅ Частичный наблюдатель устойчив.")
else:
    print("⚠️  Частичный наблюдатель НЕ устойчив!")

# ==========================================
# 6. ПОЛНЫЙ ЗАКОН УПРАВЛЕНИЯ
# ==========================================
print("\n=== ПОЛНЫЙ РЕГУЛЯТОР ===")

# Для полного регулятора используем упрощённую модель без z
# Состояния: [vL, vR, theta, d, xi]
A_reduced = np.array([
    [-1/tau,  0,       0,       0,       0],
    [0,      -1/tau,   0,       0,       0],
    [-1/W,    1/W,     0,       0,       0],
    [-0.5,   -0.5,     0,       0,       0],
    [0,       0,       0,       1,       0]
])

B_reduced = np.array([
    [K/tau, 0],
    [0,     K/tau],
    [0,     0],
    [0,     0],
    [0,     0]
])

# Дискретизация
sys_reduced = signal.cont2discrete((A_reduced, B_reduced, np.eye(5), np.zeros((5, 2))), Ts, method='zoh')
Ad_red, Bd_red, _, _, _ = sys_reduced

# LQR для упрощённой системы
Q_red = np.diag([0.1, 0.1, 5.0, 2.0, 20.0])  # без z
R_red = np.diag([0.5, 0.5])

P_red = solve_discrete_are(Ad_red, Bd_red, Q_red, R_red)
K_c = np.linalg.inv(R_red + Bd_red.T @ P_red @ Bd_red) @ Bd_red.T @ P_red @ Ad_red

print(f"✅ Матрица регулятора K_c (2x5) найдена")

# Расчёт K_r
A_cl = Ad_red - Bd_red @ K_c
C_d_red = np.array([[0, 0, 0, 1, 0]])  # выделяем d
M = -C_d_red @ np.linalg.pinv(A_cl) @ Bd_red
K_r = np.linalg.pinv(M)

print(f"✅ Коэффициент K_r найден\n")

# ==========================================
# 7. ВЫВОД В ФОРМАТЕ C++
# ==========================================
print("=== КОЭФФИЦИЕНТЫ ДЛЯ ARDUINO ===\n")

print(f"// Частичный наблюдатель (оценка vL, vR)")
print(f"double L_v[2][2] = {{")
for i in range(2):
    print(f"    {{{L_v[i][0]:.6f}, {L_v[i][1]:.6f}}},")
print("};\n")

print(f"// Матрица регулятора K_c (2x5) для [vL, vR, theta, d, xi]")
print(f"double K_c[2][5] = {{")
for i in range(2):
    row = ", ".join([f"{K_c[i][j]:.6f}" for j in range(5)])
    print(f"    {{{row}}},")
print("};\n")

print(f"// Коэффициент точного слежения K_r (2x1)")
print(f"double K_r[2][1] = {{")
for i in range(2):
    print(f"    {{{K_r[i][0]:.6f}}},")
print("};\n")

print("// Параметры для Arduino:")
print(f"const double Ts = {Ts};  // период дискретизации")
print(f"const double v0 = {v0};  // номинальная скорость")
print(f"const double W = {W};    // колея")

# ==========================================
# 6. РАСЧЁТ K_r (ТОЧНОЕ СЛЕЖЕНИЕ ЗА d_ref)
# ==========================================
# ВАЖНО: используем Ad_red и Bd_red (размер 5x5 и 5x2)
A_cl = Ad_red - Bd_red @ K_c          # Замкнутая система (5x5)
C_d_red = np.array([[0, 0, 0, 1, 0]]) # Выделяем расстояние d (индекс 3)

# Статический коэффициент: K_r = -inv(C_d * A_cl^-1 * B)
M = -C_d_red @ np.linalg.inv(A_cl) @ Bd_red
K_r = np.linalg.pinv(M)

print("✅ Коэффициент K_r (2x1) найден.\n")

# ==========================================
# 7. ВЫВОД В ФОРМАТЕ C++
# ==========================================
print("=== КОЭФФИЦИЕНТЫ ДЛЯ ARDUINO ===\n")

print("// Частичный наблюдатель (оценка vL, vR)")
print("// x_hat_v = Ad_v * x_hat_v + Bd_v * u + L_v * (y - Cd_v * x_hat_v)")
print(f"double L_v[2][2] = {{")
for i in range(2):
    print(f"    {{{L_v[i][0]:.6f}, {L_v[i][1]:.6f}}},")
print("};\n")

print("// Матрицы частичной системы (для наблюдателя vL, vR)")
print(f"double Ad_v[2][2] = {{")
for i in range(2):
    print(f"    {{{Ad_v[i][0]:.6f}, {Ad_v[i][1]:.6f}}},")
print("};\n")

print(f"double Bd_v[2][2] = {{")
for i in range(2):
    print(f"    {{{Bd_v[i][0]:.6f}, {Bd_v[i][1]:.6f}}},")
print("};\n")

print(f"double Cd_v[2][2] = {{")
for i in range(2):
    print(f"    {{{Cd_v[i][0]:.6f}, {Cd_v[i][1]:.6f}}},")
print("};\n")

print(f"// Матрица регулятора K_c (2x5) для состояний [vL, vR, theta, d, xi]")
print(f"double K_c[2][5] = {{")
for i in range(2):
    row = ", ".join([f"{K_c[i][j]:.6f}" for j in range(5)])
    print(f"    {{{row}}},")
print("};\n")

print(f"// Коэффициент точного слежения K_r (2x1)")
print(f"double K_r[2][1] = {{")
for i in range(2):
    print(f"    {{{K_r[i][0]:.6f}}},")
print("};\n")

print("// Параметры системы:")
print(f"const double Ts = {Ts};  // период дискретизации [с]")
print(f"const double v0 = {v0};  // номинальная скорость [м/с]")
print(f"const double W = {W};    // колея [м]")
print(f"const double tau = {tau}; // постоянная времени [с]")
print(f"const double K = {K};    // коэффициент усиления")