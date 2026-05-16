# Prime Robotics Control

Репозиторий проекта команды «Прайм» по программному управлению роботами.

Проект включает сайт управления, код Arduino/ESP, регулятор следования, поддержку одной и двух машинок, обработку датчиков и матричную модель управления.

## Что реализовано

- Подключение `car1` и `car2` к одному сайту управления.
- Возможность работать с одной машинкой без ошибки отсутствия второй.
- Параллельный запуск двух регуляторов.
- Разделение регуляторов по файлам:
  - `controllers/follow_base.py`
  - `controllers/follow_car1.py`
  - `controllers/follow_car2.py`
- Исправление инверсной PWM-логики:
  - `PWM = 0` — максимум;
  - `PWM = 255` — стоп.
- Защита от кратковременной потери сигнала дальномера.
- Чтение MPU6050 через I2Cdevlib.
- Однократная калибровка гироскопа при первом обращении.
- Быстрый общий опрос датчиков через команду `sensors`.
- Матричная модель объекта и матричный закон управления.

## Итоговая математическая запись

Модель объекта:

```text
x(k+1) = Ad*x(k) + Bd*u(k) + Gd*vf(k) + Hd*omega_gyro(k)
y(k)   = C*x(k)
```

Закон управления:

```text
q(k)   = F*xi(k)
u(k)   = S*q(k)
PWM(k) = 255*(1-u(k))
```

где:

```text
x  = [vL, vR, theta, z, d]^T
u  = [uL, uR]^T
q  = [u_v, u_w]^T
xi = [vL, vR, theta, z, d, vf_hat, d_target]^T
```

## Структура

```text
prime-robotics-control/
├── server.py
├── controllers/
├── arduino/
├── templates/
├── scripts/
├── docs/
├── versions/
└── archive/
```

## Быстрый запуск сайта

Установить зависимости:

```bash
pip install -r requirements.txt
```

Запустить сервер:

```bash
python server.py
```

Открыть сайт:

```text
http://localhost:5000
```

## Где смотреть версии

```text
versions/
```

Карта версий:

```text
docs/VERSION_MAP.md
```

История изменений:

```text
CHANGELOG.md
```

Досье участника:

```text
DOSSIER_SOUTHBOARD.md
```

## Что показывать на защите

1. `README.md` — общая структура.
2. `CHANGELOG.md` — история развития проекта.
3. `docs/VERSION_MAP.md` — какая версия за что отвечает.
4. `controllers/follow_car1.py` — матричный закон управления.
5. `controllers/follow_base.py` — матричная модель объекта.
6. `arduino/arduino_uno/arduino_uno/cmd_mpu6050.cpp.cpp` — MPU6050 через I2Cdev.
7. `arduino/arduino_uno/arduino_uno/cmd_sensors.cpp` — общий опрос датчиков.
8. `docs/DEFENSE_TEXT.md` — короткий текст защиты.
