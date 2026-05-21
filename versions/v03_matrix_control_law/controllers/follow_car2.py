import math
from .follow_base import BaseFollowController


class FollowControllerCar2(BaseFollowController):
    """Математический регулятор для второй машинки.

    ВАЖНО: функция compute_control формирует управление без ветвлений.
    Закон записан как аффинная матричная обратная связь по расширенному
    вектору состояния xi. Ограничения выполняются через математическое
    насыщение clamp(), а не через ветвление.
    """

    @staticmethod
    def mat_vec_mul(A, x):
        return [sum(a * b for a, b in zip(row, x)) for row in A]

    def compute_control(self, p, dt, K, target_d, distance_valid, gyro_valid):
        # 1) Оценка состояния по дискретной модели объекта.
        v = 0.5 * (self.v_l + self.v_r)
        self.theta = float(p['theta_leak']) * (self.theta + self.gyro_z_f * dt)
        self.z = float(p.get('z_leak', 0.97)) * self.z + v * math.sin(self.theta) * dt

        # 2) Оценка скорости впередиидущего объекта: vf = d_dot + v.
        vf_raw = self.clamp(self.d_dot_f + v, 0.0, K)
        vf_alpha = float(p['vf_alpha'])
        self.vf_hat = vf_alpha * vf_raw + (1.0 - vf_alpha) * self.vf_hat

        # 3) Расширенный вектор для матричного закона управления:
        # xi = [vL, vR, theta, z, d, d_dot, vf_hat, omega_z, d_target, 1]^T
        xi = [
            self.v_l,
            self.v_r,
            self.theta,
            self.z,
            self.d_f,
            self.d_dot_f,
            self.vf_hat,
            self.gyro_z_f,
            target_d,
            1.0,
        ]

        kp_d = float(p['kp_d'])
        kd_d = float(p['kd_d'])
        k_z = float(p['k_z'])
        k_theta = float(p['k_theta'])
        k_omega = float(p['k_omega'])

        # [u_v, u_w]^T = F * xi
        F = [
            [0.0, 0.0, 0.0, 0.0, kp_d, kd_d, 1.0 / K, 0.0, -kp_d, 0.0],
            [0.0, 0.0, -k_theta, -k_z, 0.0, 0.0, 0.0, -k_omega, 0.0, 0.0],
        ]
        u_v_raw, u_w_raw = self.mat_vec_mul(F, xi)

        # 4) Математическое насыщение управляющих воздействий.
        u_v = self.clamp(u_v_raw, 0.0, float(p['max_u']))
        u_w = self.clamp(u_w_raw, -float(p['max_turn']), float(p['max_turn']))

        return u_v, u_w, {
            'algorithm_file': 'controllers/follow_car2.py',
            'control_form': 'matrix_law_no_if',
        }
