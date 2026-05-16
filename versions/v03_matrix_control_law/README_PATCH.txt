Patch v3

Что изменено
- разрешены whitelisted import-ы в пользовательских скриптах
- разрешены def/return и функции с аргументами/значениями по умолчанию
- exec переведён на единое пространство имён, чтобы пользовательские функции видели импорты и переменные
- сохранён запрет на опасные импорты и dunder-доступ

Whitelist импортов
- time
- math
- random
- statistics
- json
- numpy, numpy.linalg
- scipy, scipy.linalg, scipy.signal

Что по-прежнему запрещено
- import *
- относительные импорты
- open/eval/exec/compile/input
- доступ к dunder-атрибутам
- try/except, raise, class, with



Patch v4
- добавлен плавный регулятор follow_distance_smooth.py
- добавлены фильтрация, deadzone, gyro bias calibration, slew-rate limit, PWM ramp limit
- базовый follow_distance_demo.py оставлен как простой пример, но для стенда рекомендуется smooth-версия
