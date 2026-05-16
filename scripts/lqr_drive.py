import math
import time

def drive_step(speed, delay_s=0.1):
    motor(left_pwm=speed, right_pwm=speed, left_dir="forward", right_dir="forward")
    time.sleep(delay_s)

def stop_car():
    motor(left_pwm=0, right_pwm=0, left_dir="forward", right_dir="forward")

for i in range(3):
    drive_step(140 + i * 10, 0.1)

distance_data = duration()
distance_cm = distance_data.get("distance_cm", 999)
print("distance_cm =", round(float(distance_cm), 2))
print("cos(45°) =", round(math.cos(math.pi / 4), 4))
stop_car()
