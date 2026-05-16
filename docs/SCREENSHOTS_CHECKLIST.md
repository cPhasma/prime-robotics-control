# Что добавить в GitHub для убедительности

Папка для скриншотов:

```text
docs/screenshots/
```

Добавь туда:

1. `github_commits.png` — история коммитов на GitHub.
2. `github_structure.png` — структура репозитория.
3. `site_main_page.png` — главная страница сайта управления.
4. `two_cars_connected.png` — если есть, экран с подключением двух машинок.
5. `single_car_mode.png` — работа с одной машинкой.
6. `arduino_code.png` — код Arduino с командами `duration`, `mpu6050`, `sensors`.
7. `matrix_controller_code.png` — `controllers/follow_car1.py`, где видно `q = F*xi`, `u = S*q`.
8. `pwm_fix_code.png` — место, где видно инверсную PWM-логику.
9. `mpu_i2cdev_code.png` — `cmd_mpu6050.cpp.cpp` с `I2Cdev.h`, `MPU6050.h`, `CalibrateGyro(6)`.
10. `testing_photo_or_video_frame.png` — фото/кадр машинки на тесте.

## Что писать в подписи к скринам

Коротко и профессионально:

```text
Рисунок 1 — структура репозитория с историей версий проекта.
Рисунок 2 — фрагмент матричного закона управления в follow_car1.py.
Рисунок 3 — реализация однократной калибровки MPU6050 через I2Cdev.
Рисунок 4 — интерфейс сайта управления двумя машинками.
```
