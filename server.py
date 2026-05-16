from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import socket
import json
import threading
import time
import math
import logging
import re
import uuid
import ast
import importlib
from pathlib import Path
from datetime import datetime

from controllers.follow_car1 import FollowControllerCar1
from controllers.follow_car2 import FollowControllerCar2

# ========== НАСТРОЙКА ЛОГГЕРОВ ==========

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

json_logger = logging.getLogger('JSON_MESSAGES')
json_handler = logging.StreamHandler()
json_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s', datefmt='%H:%M:%S'))
json_logger.addHandler(json_handler)
json_logger.setLevel(logging.INFO)
json_logger.propagate = False

raw_logger = logging.getLogger('RAW_MESSAGES')
raw_handler = logging.StreamHandler()
raw_handler.setFormatter(logging.Formatter('%(asctime)s - RAW: %(message)s', datefmt='%H:%M:%S'))
raw_logger.addHandler(raw_handler)
raw_logger.setLevel(logging.INFO)
raw_logger.propagate = False

# ========== FLASK ==========

BASE_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = BASE_DIR / 'scripts'
SCRIPTS_DIR.mkdir(exist_ok=True)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'wifi-car-secret'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# ========== ДВЕ МАШИНКИ ==========

CAR_IDS = ('car1', 'car2')


class EspConnection:
    def __init__(self, car_id, sock, address):
        self.car_id = car_id
        self.sock = sock
        self.address = address
        self.recv_buffer = ''
        self.connected = True
        self.send_lock = threading.Lock()
        self.connected_at = datetime.now().isoformat(timespec='seconds')


esp_clients = {car_id: None for car_id in CAR_IDS}
esp_lock = threading.Lock()
registered_commands_by_car = {car_id: [] for car_id in CAR_IDS}

pending_commands = {}
pending_lock = threading.Lock()

# ВАЖНО: у этой машинки PWM инверсный: 0 = максимум, 255 = стоп.
MOTOR_PWM_INVERTED = True


def normalize_car_id(car_id):
    car_id = str(car_id or 'car1').strip().lower()
    if car_id not in CAR_IDS:
        raise ValueError(f'Неизвестная машинка: {car_id}')
    return car_id


def is_car_connected(car_id):
    car_id = normalize_car_id(car_id)
    with esp_lock:
        conn = esp_clients.get(car_id)
        return bool(conn and conn.connected)


def cars_status_snapshot():
    with esp_lock:
        cars = {}
        for car_id in CAR_IDS:
            conn = esp_clients.get(car_id)
            cars[car_id] = {
                'connected': bool(conn and conn.connected),
                'address': str(conn.address) if conn and conn.address else None,
                'connected_at': conn.connected_at if conn else None,
                'commands_count': len(registered_commands_by_car.get(car_id, [])),
            }
    return cars


def emit_car_status():
    cars = cars_status_snapshot()
    socketio.emit('esp_status', {
        'connected': any(info['connected'] for info in cars.values()),
        'cars': cars,
    })


def choose_free_car_id():
    with esp_lock:
        for car_id in CAR_IDS:
            conn = esp_clients.get(car_id)
            if conn is None or not conn.connected:
                return car_id
    return None


def register_connection(conn):
    with esp_lock:
        old = esp_clients.get(conn.car_id)
        if old and old is not conn:
            old.connected = False
            try:
                old.sock.close()
            except Exception:
                pass
        esp_clients[conn.car_id] = conn
    emit_car_status()


def unregister_connection(conn):
    with esp_lock:
        if esp_clients.get(conn.car_id) is conn:
            esp_clients[conn.car_id] = None
            registered_commands_by_car[conn.car_id] = []
    emit_car_status()


def reassign_connection(conn, requested_car_id):
    requested_car_id = normalize_car_id(requested_car_id)
    if requested_car_id == conn.car_id:
        return
    with esp_lock:
        target = esp_clients.get(requested_car_id)
        if target and target is not conn and target.connected:
            logger.warning(f'⚠️ {requested_car_id} already connected, keep {conn.car_id}')
            return
        old_car_id = conn.car_id
        if esp_clients.get(old_car_id) is conn:
            esp_clients[old_car_id] = None
            registered_commands_by_car[old_car_id] = []
        conn.car_id = requested_car_id
        esp_clients[requested_car_id] = conn
    logger.info(f'🔁 ESP reassigned: {old_car_id} -> {requested_car_id}')
    emit_car_status()

# ========== SCRIPT RUNNER ==========


class ScriptRunner:
    def __init__(self):
        self.thread = None
        self.lock = threading.Lock()
        self.pause_event = threading.Event()
        self.pause_event.set()
        self.stop_event = threading.Event()
        self.current_script = None
        self.current_car_id = 'car1'
        self.state = 'idle'
        self.started_at = None

    def snapshot(self):
        return {
            'state': self.state,
            'current_script': self.current_script,
            'current_car_id': self.current_car_id,
            'started_at': self.started_at,
        }

    def emit_state(self):
        socketio.emit('script_state', self.snapshot())

    def log(self, message, level='info'):
        payload = {
            'message': message,
            'level': level,
            'ts': datetime.now().strftime('%H:%M:%S'),
            'car_id': self.current_car_id,
        }
        socketio.emit('script_log', payload)
        if level == 'error':
            logger.error(f'[SCRIPT][{self.current_car_id}] {message}')
        else:
            logger.info(f'[SCRIPT][{self.current_car_id}] {message}')

    def ensure_not_stopped(self):
        if self.stop_event.is_set():
            raise RuntimeError('Script stopped by user')
        while not self.pause_event.is_set():
            if self.stop_event.is_set():
                raise RuntimeError('Script stopped by user')
            time.sleep(0.05)

    def _validate_ast(self, code: str):
        tree = ast.parse(code, mode='exec')

        allowed_imports = {
            'time', 'math', 'random', 'statistics', 'json', 're',
            'numpy', 'numpy.linalg',
            'scipy', 'scipy.linalg', 'scipy.signal',
        }

        banned_nodes = (
            ast.Global, ast.Nonlocal, ast.With, ast.AsyncWith,
            ast.ClassDef, ast.Lambda, ast.Delete, ast.Yield, ast.YieldFrom, ast.Await,
            ast.AsyncFunctionDef,
        )
        banned_calls = {
            'open', 'exec', 'eval', 'compile', '__import__', 'input',
            'globals', 'locals', 'vars', 'dir', 'getattr', 'setattr', 'delattr',
            'help', 'breakpoint',
        }
        banned_attrs = {
            '__class__', '__dict__', '__bases__', '__mro__', '__subclasses__',
            '__globals__', '__code__', '__closure__', '__func__', '__self__',
            '__getattribute__', '__getattr__', '__setattr__', '__delattr__',
            '__reduce__', '__reduce_ex__', '__loader__', '__spec__', '__builtins__',
            'system', 'popen', 'spawn', 'fork', 'execv', 'execve', 'remove', 'unlink',
            'rmdir', 'rename', 'replace', 'chmod', 'chown', 'mkdir', 'makedirs',
        }

        def import_allowed(module_name: str) -> bool:
            return any(module_name == allowed or module_name.startswith(allowed + '.') for allowed in allowed_imports)

        for node in ast.walk(tree):
            if isinstance(node, banned_nodes):
                raise ValueError(f'Запрещённая конструкция: {type(node).__name__}')

            if isinstance(node, ast.Import):
                for alias in node.names:
                    if not import_allowed(alias.name):
                        raise ValueError(f'Импорт модуля {alias.name} запрещён')

            if isinstance(node, ast.ImportFrom):
                if node.level and node.level != 0:
                    raise ValueError('Относительные импорты запрещены')
                if node.module is None or not import_allowed(node.module):
                    raise ValueError(f'Импорт из модуля {node.module} запрещён')
                for alias in node.names:
                    if alias.name == '*':
                        raise ValueError('Импорт через * запрещён')

            if isinstance(node, ast.Attribute):
                if node.attr.startswith('__') or node.attr in banned_attrs:
                    raise ValueError(f'Доступ к атрибуту {node.attr} запрещён')
                if isinstance(node.value, ast.Name) and node.value.id.startswith('__'):
                    raise ValueError('Доступ к dunder-атрибутам запрещён')

            if isinstance(node, ast.Name) and node.id.startswith('__'):
                raise ValueError('Использование dunder-имен запрещено')

            if isinstance(node, ast.Call):
                if isinstance(node.func, ast.Name) and node.func.id in banned_calls:
                    raise ValueError(f'Вызов {node.func.id} запрещён')
                if isinstance(node.func, ast.Attribute):
                    if node.func.attr.startswith('__') or node.func.attr in banned_attrs:
                        raise ValueError(f'Вызов атрибута {node.func.attr} запрещён')

        return tree

    def _build_api(self, car_id):
        allowed_imports = {
            'time', 'math', 'random', 'statistics', 'json', 're',
            'numpy', 'numpy.linalg',
            'scipy', 'scipy.linalg', 'scipy.signal',
        }

        def import_allowed(module_name: str) -> bool:
            return any(module_name == allowed or module_name.startswith(allowed + '.') for allowed in allowed_imports)

        def safe_import(name, globals=None, locals=None, fromlist=(), level=0):
            if level and level != 0:
                raise ImportError('Относительные импорты запрещены')
            if not import_allowed(name):
                raise ImportError(f'Импорт модуля {name} запрещён')
            module = TimeProxy if name == 'time' else importlib.import_module(name)
            if fromlist:
                for item in fromlist:
                    if item == '*':
                        raise ImportError('Импорт через * запрещён')
                    candidate = f'{name}.{item}'
                    if import_allowed(candidate):
                        continue
                    if not hasattr(module, item):
                        raise ImportError(f'Модуль {name} не содержит {item}')
            return module

        def make_command(command_name, timeout):
            def _command(**kwargs):
                self.ensure_not_stopped()
                payload = {
                    'cmd': command_name,
                    'command_id': str(uuid.uuid4()),
                    **kwargs,
                }
                self.log(f'→ {command_name} {kwargs}', 'info')
                result = send_json_command_and_wait(car_id, payload, timeout=timeout)
                if int(result.get('exit_code', 1)) != 0:
                    raise RuntimeError(f'{command_name} failed: {result.get("stderr") or result.get("stdout") or result}')
                parsed = result.get('_parsed_stdout')
                return parsed if parsed is not None else result
            return _command

        def _print(*args, **kwargs):
            parts = [str(x) for x in args]
            if kwargs:
                parts.append(str(kwargs))
            self.log(' '.join(parts), 'info')

        runner = self

        class TimeProxy:
            @staticmethod
            def sleep(seconds):
                try:
                    seconds = float(seconds)
                except Exception as exc:
                    raise ValueError('time.sleep ожидает число') from exc
                if seconds < 0:
                    raise ValueError('time.sleep не принимает отрицательные значения')
                end_at = time.time() + seconds
                while time.time() < end_at:
                    runner.ensure_not_stopped()
                    time.sleep(min(0.05, end_at - time.time()))

        safe_builtins = {
            '__import__': safe_import,
            'range': range, 'len': len, 'min': min, 'max': max, 'abs': abs,
            'int': int, 'float': float, 'str': str, 'bool': bool,
            'sum': sum, 'round': round, 'enumerate': enumerate, 'zip': zip,
            'list': list, 'dict': dict, 'tuple': tuple, 'set': set,
            'sorted': sorted, 'reversed': reversed, 'map': map, 'filter': filter, 'iter': iter,
            'isinstance': isinstance, 'issubclass': issubclass, 'type': type, 'object': object,
            'Exception': Exception, 'RuntimeError': RuntimeError, 'ValueError': ValueError,
            'TypeError': TypeError, 'KeyError': KeyError, 'IndexError': IndexError,
            'AttributeError': AttributeError, 'ImportError': ImportError, 'TimeoutError': TimeoutError,
            'print': _print,
        }

        api = {
            'motor': make_command('motor', timeout=0.75),
            'duration': lambda: make_command('duration', timeout=1.5)(),
            'mpu6050': lambda: make_command('mpu6050', timeout=8.0)(),
            'ping': lambda: make_command('ping', timeout=1.0)(),
            'time': TimeProxy,
            'print': _print,
            '__builtins__': safe_builtins,
        }
        api.update({k: v for k, v in safe_builtins.items() if k != '__import__'})
        return api

    def _run(self, car_id: str, script_name: str, code: str):
        try:
            if not is_car_connected(car_id):
                raise RuntimeError(f'{car_id}: ESP не подключена')
            self._validate_ast(code)
            api = self._build_api(car_id)
            compiled = compile(code, script_name, 'exec')
            self.log(f'Старт скрипта: {script_name}', 'success')
            exec(compiled, api, api)
            self.log(f'Скрипт завершён: {script_name}', 'success')
        except Exception as exc:
            self.log(f'Ошибка выполнения: {exc}', 'error')
            try_emergency_stop(car_id, 'script error')
        finally:
            if self.stop_event.is_set():
                try_emergency_stop(car_id, 'script stopped')
            with self.lock:
                self.thread = None
                self.current_script = None
                self.state = 'idle'
                self.started_at = None
                self.stop_event.clear()
                self.pause_event.set()
            self.emit_state()

    def start(self, car_id: str, script_name: str, code: str):
        car_id = normalize_car_id(car_id)
        with self.lock:
            if self.thread and self.thread.is_alive():
                raise RuntimeError('Уже выполняется другой скрипт')
            self.stop_event.clear()
            self.pause_event.set()
            self.current_script = script_name
            self.current_car_id = car_id
            self.state = 'running'
            self.started_at = datetime.now().isoformat(timespec='seconds')
            self.thread = threading.Thread(target=self._run, args=(car_id, script_name, code), daemon=True)
            self.thread.start()
        self.emit_state()

    def pause(self):
        with self.lock:
            if self.state != 'running':
                raise RuntimeError('Пауза доступна только для running')
            self.pause_event.clear()
            self.state = 'paused'
        self.log('Скрипт поставлен на паузу', 'info')
        self.emit_state()

    def resume(self):
        with self.lock:
            if self.state != 'paused':
                raise RuntimeError('Resume доступен только для paused')
            self.pause_event.set()
            self.state = 'running'
        self.log('Скрипт продолжен', 'info')
        self.emit_state()

    def stop(self):
        with self.lock:
            if self.state not in ('running', 'paused'):
                raise RuntimeError('Нет активного скрипта для остановки')
            self.stop_event.set()
            self.pause_event.set()
            self.state = 'stopping'
        self.log('Запрошена остановка скрипта', 'info')
        self.emit_state()


def try_emergency_stop(car_id='car1', reason=''):
    try:
        if is_car_connected(car_id):
            payload = {
                'cmd': 'motor',
                'command_id': str(uuid.uuid4()),
                'left_pwm': 255,
                'right_pwm': 255,
                'left_dir': 'forward',
                'right_dir': 'forward',
            }
            send_to_esp(car_id, json.dumps(payload, ensure_ascii=False))
            suffix = f': {reason}' if reason else ''
            logger.warning(f'🛑 Emergency stop sent to {car_id}{suffix}')
    except Exception as exc:
        logger.error(f'Failed to send emergency stop to {car_id}: {exc}')


script_runner = ScriptRunner()

# ========== JSON HELPERS ==========


def fix_unescaped_quotes_in_stdout(json_str):
    pattern = r'("stdout"\s*:\s*")(.+?)(")(?=\s*[,}])'

    def replace_stdout(match):
        prefix = match.group(1)
        value = match.group(2)
        suffix = match.group(3)
        fixed = value.replace('\\', '\\\\').replace('"', '\\"')
        return prefix + fixed + suffix

    return re.sub(pattern, replace_stdout, json_str)


def safe_parse_json(line):
    try:
        return json.loads(line), None
    except json.JSONDecodeError:
        pass

    try:
        fixed = fix_unescaped_quotes_in_stdout(line)
        if fixed != line:
            raw_logger.info(f'Fixed: {line[:80]}...')
        return json.loads(fixed), 'Fixed stdout quotes'
    except Exception:
        return None, 'Cannot parse'


def try_parse_embedded_json(value):
    if isinstance(value, (dict, list)):
        return value
    if not isinstance(value, str):
        return None

    text = value.strip()
    if not text:
        return None

    candidates = [text]
    obj_start = text.find('{')
    obj_end = text.rfind('}')
    if obj_start >= 0 and obj_end > obj_start:
        candidates.append(text[obj_start:obj_end + 1])

    arr_start = text.find('[')
    arr_end = text.rfind(']')
    if arr_start >= 0 and arr_end > arr_start:
        candidates.append(text[arr_start:arr_end + 1])

    for candidate in candidates:
        try:
            return json.loads(candidate)
        except Exception:
            pass
    return None


def parse_distance_cm_from_stdout(stdout):
    parsed = try_parse_embedded_json(stdout)
    if isinstance(parsed, dict):
        if parsed.get('valid') is False:
            return -1.0
        for key in ('distance_cm', 'distanceCm', 'cm'):
            if key in parsed:
                return float(parsed[key])
        if 'duration_us' in parsed:
            return float(parsed['duration_us']) * 0.0343 / 2.0

    text = '' if stdout is None else str(stdout)
    match = re.search(r'(?:distance|dist|range)[^0-9+\-]*([+\-]?\d+(?:[.,]\d+)?)\s*(cm|сm|см|m|м)?', text, re.I)
    if not match:
        match = re.search(r'([+\-]?\d+(?:[.,]\d+)?)\s*(cm|сm|см|m|м)\b', text, re.I)
    if match:
        value = float(match.group(1).replace(',', '.'))
        unit = (match.group(2) or 'cm').lower()
        return value * 100.0 if unit in ('m', 'м') else value

    match = re.search(r'(?:duration|echo|us|мкс)[^0-9+\-]*([+\-]?\d+(?:[.,]\d+)?)', text, re.I)
    if match:
        return float(match.group(1).replace(',', '.')) * 0.0343 / 2.0

    raise ValueError(f'cannot parse distance from stdout: {text[:160]}')


def parse_gyro_z_from_stdout(stdout):
    parsed = try_parse_embedded_json(stdout)
    if isinstance(parsed, dict):
        for key in ('omega_z', 'gyro_z', 'gyro_z_rad_s', 'gz', 'gyroZ', 'z'):
            if key in parsed:
                return float(parsed[key])
    text = '' if stdout is None else str(stdout)
    match = re.search(r'(?:omega[_\s-]*z|gyro[_\s-]*z|gz|z)[^0-9+\-]*([+\-]?\d+(?:[.,]\d+)?)', text, re.I)
    if match:
        return float(match.group(1).replace(',', '.'))
    raise ValueError(f'cannot parse gyro_z from stdout: {text[:160]}')


def attach_parsed_stdout(result):
    if not isinstance(result, dict):
        return result
    stdout = result.get('stdout')
    parsed = try_parse_embedded_json(stdout)
    if parsed is None:
        try:
            if result.get('command') == 'duration':
                parsed = {'distance_cm': parse_distance_cm_from_stdout(stdout)}
            elif result.get('command') == 'mpu6050':
                parsed = {'gyro_z': parse_gyro_z_from_stdout(stdout)}
        except Exception:
            parsed = None
    if parsed is not None:
        result['_parsed_stdout'] = parsed
    return result

# ========== ЛОГГИРОВАНИЕ ==========


def log_json_message(direction, message, source='ESP'):
    arrow = '→' if direction == 'OUT' else '←'
    color = '\033[92m' if direction == 'OUT' else '\033[94m'
    reset = '\033[0m'
    try:
        data = json.loads(message) if isinstance(message, str) else message
        formatted = json.dumps(data, indent=2, ensure_ascii=False)
        json_logger.info(f'{color}{arrow} [{source}] {reset}\n{formatted}')
    except Exception:
        json_logger.info(f'{color}{arrow} [{source}] {reset}{message}')

# ========== PENDING COMMANDS ==========


def register_pending_command(car_id, command_id):
    event = threading.Event()
    with pending_lock:
        pending_commands[(car_id, command_id)] = {
            'event': event,
            'result': None,
            'created_at': time.time(),
        }
    return event


def resolve_pending_command(car_id, command_id, result):
    if not command_id:
        return
    with pending_lock:
        item = pending_commands.get((car_id, command_id))
        if item:
            item['result'] = result
            item['event'].set()


def pop_pending_result(car_id, command_id):
    with pending_lock:
        item = pending_commands.pop((car_id, command_id), None)
    return item['result'] if item else None


def cleanup_stale_pending(max_age=10.0):
    now = time.time()
    stale = []
    with pending_lock:
        for key, item in pending_commands.items():
            if now - item['created_at'] > max_age:
                stale.append(key)
        for key in stale:
            pending_commands.pop(key, None)
    if stale:
        logger.warning(f'🧹 Removed stale pending commands: {len(stale)}')


def send_json_command_and_wait(car_id, payload, timeout=1.0):
    car_id = normalize_car_id(car_id)
    command_id = payload.get('command_id')
    if not command_id:
        raise ValueError('payload must contain command_id')

    event = register_pending_command(car_id, command_id)
    ok = send_to_esp(car_id, json.dumps(payload, ensure_ascii=False))
    if not ok:
        with pending_lock:
            pending_commands.pop((car_id, command_id), None)
        raise RuntimeError(f'{car_id}: ESP not connected or send failed')

    if not event.wait(timeout):
        with pending_lock:
            pending_commands.pop((car_id, command_id), None)
        raise TimeoutError(f'{car_id}: timeout waiting for command_id={command_id}')

    result = pop_pending_result(car_id, command_id)
    if result is None:
        raise RuntimeError(f'{car_id}: no result for command_id={command_id}')
    return attach_parsed_stdout(result)

# ========== TCP SERVER ==========


def start_tcp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.settimeout(2)
    server_socket.bind(('0.0.0.0', 5001))
    server_socket.listen(5)

    logger.info('=' * 60)
    logger.info('📡 TCP Server started on port 5001, waiting for car1/car2')
    logger.info('=' * 60)

    while True:
        try:
            client_sock, address = server_socket.accept()
            car_id = choose_free_car_id()
            if car_id is None:
                logger.warning(f'⚠️ Extra ESP rejected from {address}')
                client_sock.close()
                continue

            conn = EspConnection(car_id, client_sock, address)
            register_connection(conn)
            cleanup_stale_pending()

            logger.info('=' * 60)
            logger.info(f'✅ {car_id.upper()} ESP CONNECTED from {address[0]}:{address[1]}')
            logger.info('=' * 60)

            threading.Thread(target=handle_esp_connection, args=(conn,), daemon=True).start()

        except socket.timeout:
            continue
        except Exception as exc:
            logger.error(f'❌ TCP accept error: {exc}')
            time.sleep(1)


def handle_esp_connection(conn):
    try:
        conn.sock.settimeout(0.1)
        while conn.connected:
            try:
                data = conn.sock.recv(4096)
                if not data:
                    logger.warning(f'⚠️ {conn.car_id} ESP disconnected')
                    break
                conn.recv_buffer += data.decode('utf-8', errors='ignore')
                while '\n' in conn.recv_buffer:
                    line, conn.recv_buffer = conn.recv_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        raw_logger.info(f'← [{conn.car_id}] {line}')
                        handle_arduino_message(conn, line)
            except socket.timeout:
                continue
            except BlockingIOError:
                continue
            except OSError:
                break
            except Exception as exc:
                logger.error(f'❌ Receive error from {conn.car_id}: {exc}')
                break
    finally:
        conn.connected = False
        try:
            conn.sock.close()
        except Exception:
            pass
        unregister_connection(conn)
        logger.info('=' * 60)
        logger.info(f'❌ {conn.car_id.upper()} ESP DISCONNECTED')
        logger.info('=' * 60)


def handle_arduino_message(conn, line):
    data, fix_msg = safe_parse_json(line)

    if data is None:
        logger.error(f'❌ Invalid JSON from {conn.car_id}: {line[:160]}')
        return

    if fix_msg:
        logger.info(f'✅ Auto-fixed from {conn.car_id}: {fix_msg}')

    try:
        msg_type = data.get('type', 'unknown')

        if msg_type == 'hello':
            requested = data.get('car_id') or data.get('device_id') or data.get('id')
            if requested in CAR_IDS:
                reassign_connection(conn, requested)
            socketio.emit('arduino_raw', {'car_id': conn.car_id, **data})
            return

        logger.info(f'📥 [{conn.car_id}] Processing: {msg_type}')

        if msg_type == 'capabilities':
            commands = data.get('commands', [])
            registered_commands_by_car[conn.car_id] = commands
            logger.info(f'✅ [{conn.car_id}] Registered {len(commands)} commands')
            socketio.emit('capabilities', {'car_id': conn.car_id, 'commands': commands})
            emit_car_status()

        elif msg_type == 'telemetry':
            payload = data.get('data', {}) or {}
            payload['car_id'] = conn.car_id
            socketio.emit('telemetry', payload)

        elif msg_type == 'command_result':
            data['car_id'] = conn.car_id
            attach_parsed_stdout(data)
            logger.info(f"✅ [{conn.car_id}] Result: {data.get('command')} - exit_code={data.get('exit_code')}")
            resolve_pending_command(conn.car_id, data.get('command_id', ''), data)
            socketio.emit('command_result', data)

        elif msg_type == 'error':
            logger.error(f"❌ [{conn.car_id}] Arduino error: {data.get('message')}")
            socketio.emit('arduino_error', {'car_id': conn.car_id, **data})

        elif msg_type == 'system':
            logger.info(f"💬 [{conn.car_id}] Arduino: {data.get('message')}")
            socketio.emit('arduino_raw', {'car_id': conn.car_id, **data})

        else:
            socketio.emit('arduino_raw', {'car_id': conn.car_id, **data})

    except Exception as exc:
        logger.error(f'❌ Handle error from {conn.car_id}: {exc}')


def send_to_esp(car_id, message):
    car_id = normalize_car_id(car_id)
    with esp_lock:
        conn = esp_clients.get(car_id)
    if conn and conn.connected:
        try:
            log_json_message('OUT', message, f'SERVER->{car_id}')
            with conn.send_lock:
                conn.sock.sendall((message + '\n').encode('utf-8'))
            logger.info(f'✅ Sent to {car_id} ({len(message)} bytes)')
            return True
        except Exception as exc:
            logger.error(f'❌ Send error to {car_id}: {exc}')
            conn.connected = False
            return False
    logger.warning(f'⚠️ {car_id} ESP not connected')
    return False

# ========== FOLLOW CONTROLLERS ==========

follow_controllers = {
    'car1': FollowControllerCar1('car1', socketio, send_json_command_and_wait, is_car_connected),
    'car2': FollowControllerCar2('car2', socketio, send_json_command_and_wait, is_car_connected),
}

# ========== SCRIPT STORAGE ==========


def sanitize_script_name(name: str) -> str:
    name = (name or '').strip()
    if not name:
        raise ValueError('Имя скрипта пустое')
    if not re.fullmatch(r'[A-Za-z0-9_\-\. ]{1,80}', name):
        raise ValueError('Имя скрипта содержит недопустимые символы')
    if not name.endswith('.py'):
        name += '.py'
    return name


def list_scripts():
    items = []
    for path in sorted(SCRIPTS_DIR.glob('*.py')):
        stat = path.stat()
        items.append({
            'name': path.name,
            'size': stat.st_size,
            'modified_at': datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
        })
    return items

# ========== WEBSOCKET ==========


@socketio.on('connect')
def handle_connect():
    logger.info(f'🌐 Web client connected: {request.sid}')
    emit_car_status()
    for car_id in CAR_IDS:
        commands = registered_commands_by_car.get(car_id, [])
        if commands:
            emit('capabilities', {'car_id': car_id, 'commands': commands})
        emit('follow_state', follow_controllers[car_id].snapshot())
    emit('script_state', script_runner.snapshot())
    emit('scripts_list', {'scripts': list_scripts()})


@socketio.on('disconnect')
def handle_disconnect():
    logger.info(f'🌐 Web client disconnected: {request.sid}')


@socketio.on('send_command')
def handle_send_command(data):
    data = data or {}
    car_id = normalize_car_id(data.get('car_id', 'car1'))
    command = data.get('command')
    params = data.get('params', {}) or {}
    command_id = data.get('command_id', f'cmd_{int(time.time()*1000)}')
    payload = {'cmd': command, 'command_id': command_id, **params}

    logger.info(f'🎯 Command from web: {command} -> {car_id}')
    try:
        result = send_json_command_and_wait(car_id, payload, timeout=1.5)
        emit('command_result', result)
    except Exception as exc:
        emit('command_error', {'car_id': car_id, 'command_id': command_id, 'error': str(exc)})


@socketio.on('request_capabilities')
def handle_request_capabilities(data=None):
    data = data or {}
    requested = str(data.get('car_id', 'all')).lower()
    target_ids = CAR_IDS if requested == 'all' else (normalize_car_id(requested),)

    for car_id in target_ids:
        if is_car_connected(car_id):
            request_json = json.dumps({'cmd': 'get_capabilities', 'command_id': f'capabilities_request_{car_id}'})
            if send_to_esp(car_id, request_json):
                logger.info(f'✅ Capabilities request sent to {car_id}')
            else:
                emit('command_error', {'car_id': car_id, 'error': 'Failed to send request'})
        else:
            logger.warning(f'⚠️ {car_id} ESP not connected')
            emit('command_error', {'car_id': car_id, 'error': f'{car_id} ESP not connected'})


@socketio.on('save_script')
def handle_save_script(data):
    try:
        name = sanitize_script_name(data.get('name', 'script.py'))
        code = data.get('code', '')
        (SCRIPTS_DIR / name).write_text(code, encoding='utf-8')
        emit('script_saved', {'name': name})
        socketio.emit('scripts_list', {'scripts': list_scripts()})
    except Exception as exc:
        emit('script_error', {'error': str(exc)})


@socketio.on('load_script')
def handle_load_script(data):
    try:
        name = sanitize_script_name(data.get('name', ''))
        code = (SCRIPTS_DIR / name).read_text(encoding='utf-8')
        emit('script_loaded', {'name': name, 'code': code})
    except Exception as exc:
        emit('script_error', {'error': str(exc)})


@socketio.on('delete_script')
def handle_delete_script(data):
    try:
        name = sanitize_script_name(data.get('name', ''))
        path = SCRIPTS_DIR / name
        if path.exists():
            path.unlink()
        emit('script_deleted', {'name': name})
        socketio.emit('scripts_list', {'scripts': list_scripts()})
    except Exception as exc:
        emit('script_error', {'error': str(exc)})


@socketio.on('run_script')
def handle_run_script(data):
    try:
        car_id = normalize_car_id((data or {}).get('car_id', 'car1'))
        name = sanitize_script_name((data or {}).get('name', 'untitled.py'))
        code = (data or {}).get('code', '')
        script_runner.start(car_id, name, code)
    except Exception as exc:
        emit('script_error', {'error': str(exc)})


@socketio.on('pause_script')
def handle_pause_script():
    try:
        script_runner.pause()
    except Exception as exc:
        emit('script_error', {'error': str(exc)})


@socketio.on('resume_script')
def handle_resume_script():
    try:
        script_runner.resume()
    except Exception as exc:
        emit('script_error', {'error': str(exc)})


@socketio.on('stop_script')
def handle_stop_script():
    try:
        script_runner.stop()
    except Exception as exc:
        emit('script_error', {'error': str(exc)})


@socketio.on('follow_start')
def handle_follow_start(data):
    try:
        car_id = normalize_car_id((data or {}).get('car_id', 'car1'))
        follow_controllers[car_id].start((data or {}).get('params', {}))
    except Exception as exc:
        emit('follow_error', {'car_id': (data or {}).get('car_id', 'car1'), 'error': str(exc)})


@socketio.on('follow_start_both')
def handle_follow_start_both(data):
    errors = []
    params = (data or {}).get('params', {})
    for car_id in CAR_IDS:
        try:
            follow_controllers[car_id].start(params)
        except Exception as exc:
            errors.append(f'{car_id}: {exc}')
    if errors:
        emit('follow_error', {'car_id': 'all', 'error': '; '.join(errors)})


@socketio.on('follow_stop')
def handle_follow_stop(data=None):
    try:
        car_id = normalize_car_id((data or {}).get('car_id', 'car1'))
        follow_controllers[car_id].stop()
    except Exception as exc:
        emit('follow_error', {'car_id': (data or {}).get('car_id', 'car1'), 'error': str(exc)})


@socketio.on('follow_stop_both')
def handle_follow_stop_both():
    for car_id in CAR_IDS:
        try:
            follow_controllers[car_id].stop()
        except Exception as exc:
            socketio.emit('follow_error', {'car_id': car_id, 'error': str(exc)})


@socketio.on('follow_update_params')
def handle_follow_update_params(data):
    try:
        car_id = normalize_car_id((data or {}).get('car_id', 'car1'))
        follow_controllers[car_id].update_params((data or {}).get('params', {}))
    except Exception as exc:
        emit('follow_error', {'car_id': (data or {}).get('car_id', 'car1'), 'error': str(exc)})


@socketio.on('follow_update_params_both')
def handle_follow_update_params_both(data):
    params = (data or {}).get('params', {})
    for car_id in CAR_IDS:
        try:
            follow_controllers[car_id].update_params(params)
        except Exception as exc:
            socketio.emit('follow_error', {'car_id': car_id, 'error': str(exc)})

# ========== HTTP ==========


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/commands')
def get_commands():
    return jsonify(registered_commands_by_car)


@app.route('/api/status')
def get_status():
    return jsonify({
        'cars': cars_status_snapshot(),
        'script': script_runner.snapshot(),
        'follow': {car_id: ctrl.snapshot() for car_id, ctrl in follow_controllers.items()},
    })


@app.route('/api/scripts')
def get_scripts():
    return jsonify({'scripts': list_scripts()})

# ========== ЗАПУСК ==========


if __name__ == '__main__':
    logger.info('=' * 60)
    logger.info('🚗 WIFI CAR CONTROL SERVER: car1 + car2')
    logger.info('Web: http://localhost:5000')
    logger.info('ESP TCP: port 5001')
    logger.info('=' * 60)

    tcp_thread = threading.Thread(target=start_tcp_server, daemon=True)
    tcp_thread.start()

    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
