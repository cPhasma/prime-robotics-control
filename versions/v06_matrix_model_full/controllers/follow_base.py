import time
import threading
import uuid
import logging

logger = logging.getLogger(__name__)


class BaseFollowController:
    """База регулятора следования.

    Служебная часть здесь отвечает за поток, обмен с ESP/Arduino, фильтрацию
    измерений, логи и графики. Сам закон управления вынесен в
    follow_car1.py / follow_car2.py.

    Модель объекта внутри контура записана матрично:
        x(k+1) = A_d x(k) + B_d u(k) + G_d vf(k) + H_d omega_gyro(k)
        y(k)   = C x(k)

    где x = [vL, vR, theta, z, d]^T.
    """

    DEFAULTS = {
        # Цель и период контура. dt лучше держать около 0.05 с, иначе машинка
        # успевает физически переехать целевую дистанцию.
        'target_distance_m': 1.0,
        'dt': 0.05,

        # Параметры объекта.
        'K': 1.0,
        'tau': 0.2,
        'W': 0.13,
        'v0': 0.30,

        # Коэффициенты матричного закона управления.
        'kp_d': 0.50,
        'kd_d': 0.65,
        'k_z': 1.30,
        'k_theta': 1.80,
        'k_omega': 0.18,

        # Фильтры оценивания.
        'vf_alpha': 0.25,
        'dist_alpha': 0.45,
        'gyro_alpha': 0.45,
        'deriv_alpha': 0.70,
        'theta_leak': 0.998,
        'z_leak': 0.995,

        # Ограничения управления.
        'max_u': 0.45,
        'max_turn': 0.18,
        'max_du': 0.06,
        'max_du_turn': 0.10,
        'max_pwm_step': 18,

        # Допустимый диапазон расстояний HC-SR04.
        'min_valid_distance_m': 0.03,
        'max_valid_distance_m': 3.0,

        # ВАЖНО: у этой машинки PWM инверсный: 0 = максимум, 255 = стоп.
        'invert_pwm': True,

        # MPU6050.
        # auto выбирает ось с наибольшей устойчивой угловой скоростью.
        # При необходимости можно вручную поставить x/y/z и gyro_sign=-1.
        'gyro_axis': 'auto',
        'gyro_sign': 1.0,
        'gyro_model_weight': 0.85,

        # Логи ошибок датчиков.
        'sensor_log_every': 10,
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
        self.combined_sensors_available = None
        self.reset_state()

    def reset_state(self):
        # x = [vL, vR, theta, z, d]^T
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
        self.gyro_axis_scores = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.selected_gyro_axis = 'z'

        self.u_v_prev = 0.0
        self.u_w_prev = 0.0
        self.pwm_l_prev = self.stop_pwm()
        self.pwm_r_prev = self.stop_pwm()
        self.step = 0

        self.bad_distance_reads = 0
        self.bad_gyro_reads = 0
        self.last_good_distance_m = None
        self.last_good_gyro_z = 0.0
        self.last_axes = {'x': 0.0, 'y': 0.0, 'z': 0.0}

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
                if key == 'gyro_axis':
                    value = str(value).strip().lower()
                    if value in ('auto', 'x', 'y', 'z'):
                        self.params[key] = value
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
    def mat_vec_mul(A, x):
        return [sum(a * b for a, b in zip(row, x)) for row in A]

    @staticmethod
    def vec_add(*vectors):
        return [sum(values) for values in zip(*vectors)]

    @staticmethod
    def scalar_vec_mul(scalar, vector):
        return [scalar * value for value in vector]

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
            timeout=0.85,
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

    def read_gyro_axes(self):
        result = self.send_command_and_wait(
            self.car_id,
            {'cmd': 'mpu6050', 'command_id': str(uuid.uuid4())},
            # Первый вызов mpu6050 может занять дольше: Arduino один раз
            # калибрует гироскоп через I2Cdevlib.
            timeout=8.0,
        )
        if int(result.get('exit_code', 1)) != 0:
            raise RuntimeError(result.get('stderr') or result.get('stdout') or 'mpu6050 failed')
        parsed = result.get('_parsed_stdout')
        if not isinstance(parsed, dict):
            raise RuntimeError(f'mpu6050 parse failed: {result.get("stdout", "")[:120]}')
        if parsed.get('valid') is False:
            raise RuntimeError('mpu6050 invalid')
        return {
            'x': float(parsed.get('omega_x', parsed.get('gyro_x', 0.0))),
            'y': float(parsed.get('omega_y', parsed.get('gyro_y', 0.0))),
            'z': float(parsed.get('omega_z', parsed.get('gyro_z', 0.0))),
        }

    def read_combined_sensors(self):
        result = self.send_command_and_wait(
            self.car_id,
            {'cmd': 'sensors', 'command_id': str(uuid.uuid4())},
            timeout=0.95,
        )
        if int(result.get('exit_code', 1)) != 0:
            raise RuntimeError(result.get('stderr') or result.get('stdout') or 'sensors failed')
        parsed = result.get('_parsed_stdout')
        if not isinstance(parsed, dict):
            raise RuntimeError(f'sensors parse failed: {result.get("stdout", "")[:120]}')
        return parsed

    def select_gyro_axis(self, axes, params):
        requested = str(params.get('gyro_axis', 'auto')).strip().lower()
        if requested in ('x', 'y', 'z'):
            axis = requested
        else:
            for name in ('x', 'y', 'z'):
                self.gyro_axis_scores[name] = 0.96 * self.gyro_axis_scores[name] + 0.04 * abs(float(axes.get(name, 0.0)))
            axis = max(self.gyro_axis_scores, key=self.gyro_axis_scores.get)
            if self.gyro_axis_scores[axis] < 1e-6:
                axis = self.selected_gyro_axis or 'z'
        self.selected_gyro_axis = axis
        return float(params.get('gyro_sign', 1.0)) * float(axes.get(axis, 0.0))

    def safe_read_sensors(self, params, target_distance_m):
        distance_valid = False
        gyro_valid = False
        distance_m = None
        axes = None

        # Новая прошивка поддерживает общий запрос sensors: HC-SR04 + MPU6050
        # за один обмен. Если прошивка старая, один раз пробуем и дальше
        # автоматически откатываемся на duration + mpu6050.
        if self.combined_sensors_available is not False:
            try:
                packet = self.read_combined_sensors()
                self.combined_sensors_available = True
                if packet.get('distance_valid', packet.get('valid', False)):
                    distance_m = float(packet['distance_cm']) / 100.0
                    distance_valid = True
                if packet.get('mpu_valid', packet.get('valid', False)):
                    axes = {
                        'x': float(packet.get('omega_x', packet.get('gyro_x', 0.0))),
                        'y': float(packet.get('omega_y', packet.get('gyro_y', 0.0))),
                        'z': float(packet.get('omega_z', packet.get('gyro_z', 0.0))),
                    }
                    gyro_valid = True
            except Exception as exc:
                self.combined_sensors_available = False
                self.log(f'combined sensors unavailable, fallback to separate commands: {exc}', 'warning')

        if distance_m is None:
            try:
                distance_m = self.read_distance_m()
                distance_valid = True
            except Exception as exc:
                self.bad_distance_reads += 1
                if self.bad_distance_reads == 1 or self.bad_distance_reads % int(params.get('sensor_log_every', 10)) == 0:
                    self.log(f'distance read failed, keep working: {exc}', 'warning')
                distance_m = self.last_good_distance_m
                if distance_m is None:
                    distance_m = self.d_f if self.d_f is not None else target_distance_m
        if axes is None:
            try:
                axes = self.read_gyro_axes()
                gyro_valid = True
            except Exception as exc:
                self.bad_gyro_reads += 1
                if self.bad_gyro_reads == 1 or self.bad_gyro_reads % int(params.get('sensor_log_every', 10)) == 0:
                    self.log(f'gyro read failed, keep working: {exc}', 'warning')
                axes = dict(self.last_axes)

        if distance_valid:
            if float(params['min_valid_distance_m']) <= distance_m <= float(params['max_valid_distance_m']):
                self.bad_distance_reads = 0
                self.last_good_distance_m = distance_m
            else:
                self.bad_distance_reads += 1
                distance_valid = False
                distance_m = self.last_good_distance_m if self.last_good_distance_m is not None else target_distance_m

        if gyro_valid:
            self.bad_gyro_reads = 0
            self.last_axes = dict(axes)

        omega = self.select_gyro_axis(axes, params)
        if gyro_valid:
            self.last_good_gyro_z = omega
        else:
            omega = float(self.last_good_gyro_z or 0.0)

        return float(distance_m), distance_valid, float(omega), gyro_valid

    def calibrate_gyro(self):
        # Совместимость со старым кодом. Сейчас калибровка гироскопа перенесена
        # в прошивку Arduino: I2Cdevlib выполняет её один раз при первом вызове mpu6050/sensors.
        return 0.0

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

    def build_discrete_matrices(self, params, dt, K, tau, W):
        """Возвращает A_d, B_d, G_d, H_d, C для матричной модели.

        С учётом MPU6050 theta обновляется как матричная смесь:
        omega = a*omega_gyro + (1-a)*(vR-vL)/W.
        Поэтому в дискретной модели появляется дополнительный вход H_d*omega_gyro.
        """
        v0 = float(params.get('v0', 0.30))
        z_leak = float(params.get('z_leak', 0.995))
        theta_leak = float(params.get('theta_leak', 0.998))
        gyro_weight = self.clamp(float(params.get('gyro_model_weight', 0.85)), 0.0, 1.0)

        Ad = [
            [1.0 - dt / tau, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0 - dt / tau, 0.0, 0.0, 0.0],
            [-(1.0 - gyro_weight) * dt / W * theta_leak,
             (1.0 - gyro_weight) * dt / W * theta_leak,
             1.0 * theta_leak, 0.0, 0.0],
            [0.0, 0.0, dt * v0, z_leak, 0.0],
            [-0.5 * dt, -0.5 * dt, 0.0, 0.0, 1.0],
        ]
        Bd = [
            [dt * K / tau, 0.0],
            [0.0, dt * K / tau],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
        ]
        Gd = [0.0, 0.0, 0.0, 0.0, dt]
        Hd = [0.0, 0.0, theta_leak * gyro_weight * dt, 0.0, 0.0]
        C = [
            [0.0, 0.0, 0.0, 0.0, 1.0],
            [-1.0 / W, 1.0 / W, 0.0, 0.0, 0.0],
        ]
        return Ad, Bd, Gd, Hd, C

    def predict_state_matrix(self, params, dt, K, tau, W, u_l, u_r, vf_hat, omega_gyro):
        Ad, Bd, Gd, Hd, C = self.build_discrete_matrices(params, dt, K, tau, W)
        x = [
            float(self.v_l),
            float(self.v_r),
            float(self.theta),
            float(self.z),
            float(self.d_f if self.d_f is not None else params['target_distance_m']),
        ]
        u = [float(u_l), float(u_r)]

        ax = self.mat_vec_mul(Ad, x)
        bu = self.mat_vec_mul(Bd, u)
        gv = self.scalar_vec_mul(float(vf_hat), Gd)
        hw = self.scalar_vec_mul(float(omega_gyro), Hd)
        x_next = self.vec_add(ax, bu, gv, hw)

        self.v_l, self.v_r, self.theta, self.z, self.d_f = x_next
        y_model = self.mat_vec_mul(C, x_next)
        return {
            'Ad': Ad,
            'Bd': Bd,
            'Gd': Gd,
            'Hd': Hd,
            'C': C,
            'x_next': x_next,
            'y_model': y_model,
        }

    def _run(self):
        try:
            self.log('controller starting')
            self.stop_motors('before gyro warmup')
            self.gyro_bias = 0.0

            # Первый запрос запускает однократную калибровку MPU6050 на Arduino.
            # Машинка в этот момент должна стоять неподвижно.
            with self.lock:
                p0 = dict(self.params)
            _, _, wz0, gyro0_ok = self.safe_read_sensors(p0, float(p0['target_distance_m']))
            self.gyro_z_f = wz0
            self.log(f'gyro ready: omega={wz0:.6f} rad/s, axis={self.selected_gyro_axis}, valid={gyro0_ok}')

            while not self.stop_event.is_set():
                t0 = time.time()
                with self.lock:
                    p = dict(self.params)

                dt = max(0.02, float(p['dt']))
                K = max(1e-6, float(p['K']))
                tau = max(0.02, float(p['tau']))
                W = max(1e-6, float(p['W']))
                target_d = float(p['target_distance_m'])

                d_raw, distance_valid, wz_raw, gyro_valid = self.safe_read_sensors(p, target_d)

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

                # Оценка скорости впереди идущего объекта из матричной модели расстояния:
                # d_dot = vf - (vL + vR)/2  =>  vf = d_dot + (vL + vR)/2.
                v_now = 0.5 * (self.v_l + self.v_r)
                vf_raw = self.clamp(self.d_dot_f + v_now, 0.0, K)
                vf_alpha = float(p['vf_alpha'])
                self.vf_hat = vf_alpha * vf_raw + (1.0 - vf_alpha) * self.vf_hat

                target_u_v, target_u_w, extra = self.compute_control(p, dt, K, target_d, distance_valid, gyro_valid)

                self.u_v_prev = self.slew(self.u_v_prev, target_u_v, float(p['max_du']))
                self.u_w_prev = self.slew(self.u_w_prev, target_u_w, float(p.get('max_du_turn', p['max_du'])))

                # [u_L, u_R]^T = S * [u_v, u_w]^T
                S = [[1.0, -1.0], [1.0, 1.0]]
                u_l_raw, u_r_raw = self.mat_vec_mul(S, [self.u_v_prev, self.u_w_prev])
                u_l = self.clamp(u_l_raw, 0.0, 1.0)
                u_r = self.clamp(u_r_raw, 0.0, 1.0)

                target_pwm_l = self.norm_to_pwm(u_l)
                target_pwm_r = self.norm_to_pwm(u_r)
                self.pwm_l_prev = self.ramp_pwm(self.pwm_l_prev, target_pwm_l, float(p['max_pwm_step']))
                self.pwm_r_prev = self.ramp_pwm(self.pwm_r_prev, target_pwm_r, float(p['max_pwm_step']))

                self.send_motor(self.pwm_l_prev, self.pwm_r_prev)

                matrices = self.predict_state_matrix(p, dt, K, tau, W, u_l, u_r, self.vf_hat, self.gyro_z_f)
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
                    'gyro_axis': self.selected_gyro_axis,
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
                    'model_form': 'x(k+1)=Ad*x(k)+Bd*u(k)+Gd*vf(k)+Hd*omega_gyro(k); y=C*x',
                    'x_state': matrices['x_next'],
                    'y_model_d': matrices['y_model'][0],
                    'y_model_omega': matrices['y_model'][1],
                }
                telemetry.update(extra or {})
                self.emit('follow_telemetry', telemetry)

                if self.step % max(1, int(round(1.0 / dt))) == 0:
                    self.log(
                        f"x=[vL={self.v_l:.4f}, vR={self.v_r:.4f}, theta={self.theta:.5f}, "
                        f"z={self.z:.5f}, d={self.d_f:.3f}] "
                        f"u=[uL={u_l:.4f}, uR={u_r:.4f}, uV={self.u_v_prev:.4f}, uW={self.u_w_prev:.4f}] "
                        f"pwm=[L={self.pwm_l_prev}, R={self.pwm_r_prev}] "
                        f"axis={self.selected_gyro_axis} sensors=[dist={distance_valid}, gyro={gyro_valid}]"
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
