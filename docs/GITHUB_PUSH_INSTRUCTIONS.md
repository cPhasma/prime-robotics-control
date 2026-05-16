# Как залить проект на GitHub

## Вариант 1 — правильно, с сохранением истории коммитов

В архиве уже есть локальная Git-история. После распаковки зайди в папку:

```bash
cd prime-robotics-control
```

Проверь историю:

```bash
git log --oneline --graph
```

Создай пустой репозиторий на GitHub с названием:

```text
prime-robotics-control
```

Потом выполни:

```bash
git remote add origin https://github.com/YOUR_LOGIN/prime-robotics-control.git
git push -u origin main
```

## Вариант 2 — если GitHub не принял историю

Тогда можно создать репозиторий вручную и залить файлы через сайт, но история коммитов потеряется. Для защиты лучше использовать вариант 1.

## Какие коммиты должны быть видны

```bash
git log --oneline
```

Ожидаемая логика:

```text
Initialize repository structure and documentation
Add two-car control baseline and split follow controller
Fix inverted PWM mapping for motor driver
Implement matrix-only follow control law
Switch MPU6050 reading to I2Cdev with one-time calibration
Support single-car mode and capabilities refresh
Integrate matrix state model and combined sensor polling
Polish repository documentation and archive version snapshots
```

## Если нужно пересоздать коммиты вручную

```bash
git init -b main
git add README.md CHANGELOG.md DOSSIER_SOUTHBOARD.md
git commit -m "Initialize repository structure and documentation"

git commit -m "Add two-car control baseline and split follow controller"
git commit -m "Fix inverted PWM mapping for motor driver"
git commit -m "Implement matrix-only follow control law"
git commit -m "Switch MPU6050 reading to I2Cdev with one-time calibration"
git commit -m "Support single-car mode and capabilities refresh"
git commit -m "Integrate matrix state model and combined sensor polling"

git add docs versions archive
git commit -m "Polish repository documentation and archive version snapshots"
```
