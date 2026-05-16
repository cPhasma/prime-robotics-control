import math
import time
import threading
import uuid
import logging

logger = logging.getLogger(__name__)


class BaseFollowController:
    """Базовая часть регулятора: поток, обмен с машинкой, фильтры, логи и графики.

    Сам закон управления специально вынесен в файлы follow_car1.py / follow_car2.py,
    чтобы для каждой машинки можно было менять алгоритм отдельно.
    """

    DEFAULTS = {
        'target_distance_m': 1.0,
        'dt': 0.10,
        'K': 1.0,
        'tau': 0.2,
        'W': 0.13,
        'kp_d': 0.85,
        'kd_d': 0.28,
        'k_z': 1.3,
        'k_theta': 1.8,
        'k_omega': 0.18,
        'vf_alpha': 0.25,
        'dist_alpha': 0.35,
        'gyro_alpha': 0.30,
        'deriv_alpha': 0.75,
        'theta_leak': 0.997,
        'max_u': 0.75,
        'max_turn': 0.22,
        'max_du': 0.05,
        'max_pwm_step': 14,
        'min_valid_distance_m': 0.03,
        'max_valid_distance_m': 3.0,
        # ВАЖНО: у этой машинки PWM инверсный: 0 = максимум, 255 = стоп.
        'invert_pwm': True,
        'gyro_bias_samples': 15,
        'z_leak': 0.97,
        'u_w_deadband': 0.006,
        'max_du_turn': 0.12,
        # Защита от кратковременной потери датчиков: алгоритм не падает,
        # а использует последнее корректное значение и немного снижает скорость.
        'sensor_log_every': 10,
        'sensor_slow_after': 3,
        'sensor_fail_max_u': 0.18,
    }

    def __init__(self, car_id, socketio, send_command_and_wait, is_car_connected):
        self.car_id = car_id
        self.socketio = socketio
        self.send_command_and_wait = send_command_and_wait
        self.is_car_connected = is_car_connected
        self.lock = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self.running = False
        self.params = dict(self.DEFAULTS)
        self.reset_state()

    def reset_state(self):
        self.v_l = 0.0
        self.v_r = 0.0
        self.theta = 0.0
        self.z = 0.0
        self.d_f = None
        self.prev_d = None
        self.d_dot_f = 0.0
        self.vf_hat = 0.0
        self.gyro_z_f = 0.0
        self.gyro_bias = 0.0
        self.u_v_prev = 0.0
        self.u_w_prev = 0.0
        self.pwm_l_prev = self.stop_pwm()
        self.pwm_r_prev = self.stop_pwm()
        self.step = 0
        self.bad_distance_reads = 0
        self.bad_gyro_reads = 0
        self.last_good_distance_m = None
        self.last_good_gyro_z = 0.0

    def snapshot(self):
        with self.lock:
            return {
                'car_id': self.car_id,
                'running': self.running,
                'params': dict(self.params),
            }

    def emit(self, event, payload):
        data = dict(payload or {})
        data.setdefault('car_id', self.car_id)
        self.socketio.emit(event, data)

    def log(self, message, level='info'):
        prefix = f'[{self.car_id.upper()}][FOLLOW] '
        full = prefix + message
        if level == 'error':
            logger.error(full)
        elif level == 'warning':
            logger.warning(full)
        else:
            logger.info(full)
        self.emit('follow_log', {'level': level, 'message': full})

    def update_params(self, params):
        with self.lock:
            # Не даём случайно отключить инверсную PWM-логику.
            self.params['invert_pwm'] = True
            for key, value in (params or {}).items():
                if key not in self.params or key == 'invert_pwm':
                    continue
                try:
                    self.params[key] = float(value)
                except Exception:
                    pass
        self.emit('follow_state', self.snapshot())

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def slew(current, target, max_step):
        return current + BaseFollowController.clamp(target - current, -max_step, max_step)

    def stop_pwm(self):
        # В нашей прошивке/драйвере 255 — минимальная скорость, фактически стоп.
        return 255

    def norm_to_pwm(self, u):
        # u: 0 = стоп, 1 = максимум. PWM инверсный: 255 = стоп, 0 = максимум.
        u = self.clamp(float(u), 0.0, 1.0)
        return int(round(255.0 * (1.0 - u)))

    def ramp_pwm(self, current, target, max_step):
        return int(round(self.slew(float(current), float(target), float(max_step))))

    def send_motor(self, pwm_l, pwm_r):
        return self.send_command_and_wait(self.car_id, {
            'cmd': 'motor',
            'command_id': str(uuid.uuid4()),
            'left_pwm': int(self.clamp(pwm_l, 0, 255)),
            'right_pwm': int(self.clamp(pwm_r, 0, 255)),
            'left_dir': 'forward',
            'right_dir': 'forward',
        }, timeout=0.75)

    def stop_motors(self, reason=''):
        try:
            pwm = self.stop_pwm()
            self.send_motor(pwm, pwm)
            self.log(f'stop motors {reason}'.strip(), 'warning')
        except Exception as exc:
            self.log(f'stop failed: {exc}', 'error')

    def read_distance_m(self):
        result = self.send_command_and_wait(
            self.car_id,
            {'cmd': 'duration', 'command_id': str(uuid.uuid4())},
            timeout=1.2,
        )
        if int(result.get('exit_code', 1)) != 0:
            raise RuntimeError(result.get('stderr') or result.get('stdout') or 'duration failed')
        parsed = result.get('_parsed_stdout')
        if isinstance(parsed, dict):
            if parsed.get('valid') is False:
                raise RuntimeError('ultrasonic no echo')
            if 'distance_cm' in parsed:
                return float(parsed['distance_cm']) / 100.0
            if 'duration_us' in parsed:
                return float(parsed['duration_us']) * 0.0343 / 2.0 / 100.0
        raise RuntimeError(f'duration parse failed: {result.get("stdout", "")[:120]}')

    def read_gyro_z(self):
        result = self.send_command_and_wait(
            self.car_id,
            {'cmd': 'mpu6050', 'command_id': str(uuid.uuid4())},
            timeout=1.2,
        )
        if int(result.get('exit_code', 1)) != 0:
            raise RuntimeError(result.get('stderr') or result.get('stdout') or 'mpu6050 failed')
        parsed = result.get('_parsed_stdout')
        if isinstance(parsed, dict):
            for key in ('gyro_z', 'gz', 'gyroZ', 'z'):
                if key in parsed:
                    return float(parsed[key])
        raise RuntimeError(f'mpu6050 parse failed: {result.get("stdout", "")[:120]}')

    def safe_read_distance_m(self, params, target_distance_m):
        try:
            d = self.read_distance_m()
            if not (float(params['min_valid_distance_m']) <= d <= float(params['max_valid_distance_m'])):
                raise RuntimeError(f'invalid distance {d:.3f} m')
            self.bad_distance_reads = 0
            self.last_good_distance_m = d
            return d, True
        except Exception as exc:
            self.bad_distance_reads += 1
            if self.bad_distance_reads == 1 or self.bad_distance_reads % int(params.get('sensor_log_every', 10)) == 0:
                self.log(f'distance read failed, keep working: {exc}', 'warning')
            fallback = self.last_good_distance_m
            if fallback is None:
                fallback = self.d_f if self.d_f is not None else target_distance_m
            return float(fallback), False

    def safe_read_gyro_z(self):
        try:
            wz = self.read_gyro_z() - self.gyro_bias
            self.bad_gyro_reads = 0
            self.last_good_gyro_z = wz
            return wz, True
        except Exception as exc:
            self.bad_gyro_reads += 1
            if self.bad_gyro_reads == 1 or self.bad_gyro_reads % 10 == 0:
                self.log(f'gyro read failed, keep working: {exc}', 'warning')
            return float(self.last_good_gyro_z or 0.0), False

    def calibrate_gyro(self):
        with self.lock:
            samples = int(self.params.get('gyro_bias_samples', 15))
            dt = float(self.params.get('dt', 0.10))
        acc = 0.0
        ok = 0
        for _ in range(max(1, samples)):
            if self.stop_event.is_set():
                break
            try:
                result = self.send_command_and_wait(
                    self.car_id,
                    {'cmd': 'mpu6050', 'command_id': str(uuid.uuid4())},
                    timeout=1.2,
                )
                parsed = result.get('_parsed_stdout')
                if isinstance(parsed, dict):
                    for key in ('gyro_z', 'gz', 'gyroZ', 'z'):
                        if key in parsed:
                            acc += float(parsed[key])
                            ok += 1
                            break
            except Exception as exc:
                self.log(f'gyro calibration read failed: {exc}', 'warning')
            time.sleep(min(0.04, dt))
        return acc / max(ok, 1)

    def start(self, params=None):
        if params:
            self.update_params(params)
        with self.lock:
            if self.thread and self.thread.is_alive():
                raise RuntimeError(f'{self.car_id}: follow controller already running')
            if not self.is_car_connected(self.car_id):
                raise RuntimeError(f'{self.car_id}: ESP не подключена')
            self.reset_state()
            self.stop_event.clear()
            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
        self.emit('follow_state', self.snapshot())

    def stop(self):
        with self.lock:
            if not self.running:
                return
            self.stop_event.set()
        self.stop_motors('requested')
        self.emit('follow_state', self.snapshot())

    def compute_control(self, params, dt, K, target_distance_m, distance_valid, gyro_valid):
        raise NotImplementedError('compute_control должен быть реализован в follow_car1.py / follow_car2.py')

    def _run(self):
        try:
            self.log('controller starting')
            self.stop_motors('before calibration')
            self.gyro_bias = self.calibrate_gyro()
            self.log(f'gyro_bias={self.gyro_bias:.6f} rad/s')

            while not self.stop_event.is_set():
                t0 = time.time()
                with self.lock:
                    p = dict(self.params)

                dt = max(0.03, float(p['dt']))
                K = max(1e-6, float(p['K']))
                tau = max(0.02, float(p['tau']))
                target_d = float(p['target_distance_m'])

                d_raw, distance_valid = self.safe_read_distance_m(p, target_d)
                wz_raw, gyro_valid = self.safe_read_gyro_z()

                if self.d_f is None:
                    self.d_f = d_raw
                    self.prev_d = d_raw
                else:
                    a = float(p['dist_alpha'])
                    self.d_f = a * d_raw + (1.0 - a) * self.d_f

                ga = float(p['gyro_alpha'])
                self.gyro_z_f = ga * wz_raw + (1.0 - ga) * self.gyro_z_f

                d_dot_raw = (self.d_f - self.prev_d) / dt
                da = float(p['deriv_alpha'])
                self.d_dot_f = da * self.d_dot_f + (1.0 - da) * d_dot_raw
                self.prev_d = self.d_f

                target_u_v, target_u_w, extra = self.compute_control(p, dt, K, target_d, distance_valid, gyro_valid)

                if not distance_valid and self.bad_distance_reads >= int(p.get('sensor_slow_after', 3)):
                    target_u_v = min(target_u_v, float(p.get('sensor_fail_max_u', 0.18)))

                self.u_v_prev = self.slew(self.u_v_prev, target_u_v, float(p['max_du']))
                self.u_w_prev = self.slew(self.u_w_prev, target_u_w, float(p.get('max_du_turn', p['max_du'])))

                u_l = self.clamp(self.u_v_prev - self.u_w_prev, 0.0, 1.0)
                u_r = self.clamp(self.u_v_prev + self.u_w_prev, 0.0, 1.0)

                target_pwm_l = self.norm_to_pwm(u_l)
                target_pwm_r = self.norm_to_pwm(u_r)
                self.pwm_l_prev = self.ramp_pwm(self.pwm_l_prev, target_pwm_l, float(p['max_pwm_step']))
                self.pwm_r_prev = self.ramp_pwm(self.pwm_r_prev, target_pwm_r, float(p['max_pwm_step']))

                self.send_motor(self.pwm_l_prev, self.pwm_r_prev)

                self.v_l += dt * ((K * u_l - self.v_l) / tau)
                self.v_r += dt * ((K * u_r - self.v_r) / tau)
                self.step += 1

                telemetry = {
                    'step': self.step,
                    'time': time.time(),
                    'd_raw': d_raw,
                    'd': self.d_f,
                    'd_target': target_d,
                    'e_d': self.d_f - target_d,
                    'd_dot': self.d_dot_f,
                    'v_l': self.v_l,
                    'v_r': self.v_r,
                    'v': 0.5 * (self.v_l + self.v_r),
                    'vf_hat': self.vf_hat,
                    'theta': self.theta,
                    'z': self.z,
                    'omega_z': self.gyro_z_f,
                    'gyro_bias': self.gyro_bias,
                    'u_l': u_l,
                    'u_r': u_r,
                    'u_v': self.u_v_prev,
                    'u_w': self.u_w_prev,
                    'pwm_l': self.pwm_l_prev,
                    'pwm_r': self.pwm_r_prev,
                    'distance_valid': distance_valid,
                    'gyro_valid': gyro_valid,
                    'bad_distance_reads': self.bad_distance_reads,
                    'bad_gyro_reads': self.bad_gyro_reads,
                }
                telemetry.update(extra or {})
                self.emit('follow_telemetry', telemetry)

                if self.step % max(1, int(round(1.0 / dt))) == 0:
                    self.log(
                        f"x=[vL={self.v_l:.4f}, vR={self.v_r:.4f}, theta={self.theta:.5f}, "
                        f"z={self.z:.5f}, d={self.d_f:.3f}] "
                        f"u=[uL={u_l:.4f}, uR={u_r:.4f}, uV={self.u_v_prev:.4f}, uW={self.u_w_prev:.4f}] "
                        f"pwm=[L={self.pwm_l_prev}, R={self.pwm_r_prev}] "
                        f"sensors=[dist={distance_valid}, gyro={gyro_valid}]"
                    )

                time.sleep(max(0.0, dt - (time.time() - t0)))
        except Exception as exc:
            self.log(f'controller error: {exc}', 'error')
            self.emit('follow_error', {'error': str(exc)})
            self.stop_motors('error')
        finally:
            with self.lock:
                self.running = False
                self.stop_event.clear()
            self.emit('follow_state', self.snapshot())
            self.log('controller stopped')
