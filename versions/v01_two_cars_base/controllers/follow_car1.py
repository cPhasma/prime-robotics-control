import math
from .follow_base import BaseFollowController


class FollowControllerCar1(BaseFollowController):
    """Регулятор для первой машинки.

    Менять алгоритм первой машинки нужно здесь, не в server.py.
    """

    def compute_control(self, p, dt, K, target_d, distance_valid, gyro_valid):
        v = 0.5 * (self.v_l + self.v_r)

        self.theta = float(p['theta_leak']) * (self.theta + self.gyro_z_f * dt)
        self.z = float(p.get('z_leak', 0.97)) * self.z + v * math.sin(self.theta) * dt

        vf_raw = self.clamp(self.d_dot_f + v, 0.0, K)
        va = float(p['vf_alpha'])
        self.vf_hat = va * vf_raw + (1.0 - va) * self.vf_hat

        e_d = self.d_f - target_d
        target_u_v = self.vf_hat / max(K, 1e-6) + float(p['kp_d']) * e_d + float(p['kd_d']) * self.d_dot_f
        target_u_v = self.clamp(target_u_v, 0.0, float(p['max_u']))

        target_u_w = -float(p['k_z']) * self.z - float(p['k_theta']) * self.theta - float(p['k_omega']) * self.gyro_z_f
        target_u_w = self.clamp(target_u_w, -float(p['max_turn']), float(p['max_turn']))
        if abs(target_u_w) < float(p.get('u_w_deadband', 0.006)):
            target_u_w = 0.0

        return target_u_v, target_u_w, {'algorithm_file': 'controllers/follow_car1.py'}
