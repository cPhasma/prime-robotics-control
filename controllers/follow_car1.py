from .follow_base import BaseFollowController


class FollowControllerCar1(BaseFollowController):
    """Матричный закон управления для первой машинки.

    В compute_control нет логики вида if/elif/else для выбора режима движения.
    Управление считается как матричное умножение:
        q = F * xi
        u = S * q
    где q = [u_v, u_w]^T, а u = [u_L, u_R]^T.
    """

    def compute_control(self, p, dt, K, target_d, distance_valid, gyro_valid):
        W = max(1e-6, float(p['W']))
        kp_d = float(p['kp_d'])
        kd_d = float(p['kd_d'])
        k_z = float(p['k_z'])
        k_theta = float(p['k_theta'])
        k_omega = float(p['k_omega'])

        # Расширенный вектор закона управления:
        # xi = [vL, vR, theta, z, d, vf_hat, d_target]^T
        xi = [
            self.v_l,
            self.v_r,
            self.theta,
            self.z,
            self.d_f,
            self.vf_hat,
            target_d,
        ]

        # u_v = vf/K + kp_d*(d-d_target) + kd_d*(vf - (vL+vR)/2)
        # u_w = -k_z*z - k_theta*theta - k_omega*((vR-vL)/W)
        # [u_v, u_w]^T = F * xi
        F = [
            [-0.5 * kd_d, -0.5 * kd_d, 0.0, 0.0, kp_d, (1.0 / K) + kd_d, -kp_d],
            [k_omega / W, -k_omega / W, -k_theta, -k_z, 0.0, 0.0, 0.0],
        ]

        u_v_raw, u_w_raw = self.mat_vec_mul(F, xi)

        u_v = self.clamp(u_v_raw, 0.0, float(p['max_u']))
        u_w = self.clamp(u_w_raw, -float(p['max_turn']), float(p['max_turn']))

        return u_v, u_w, {
            'algorithm_file': 'controllers/follow_car1.py',
            'control_form': 'q=F*xi; u=S*q; xi=[vL,vR,theta,z,d,vf_hat,d_target]^T',
            'control_matrix_F': F,
        }
