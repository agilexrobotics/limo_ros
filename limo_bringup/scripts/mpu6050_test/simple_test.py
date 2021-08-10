import board
import time
import adafruit_mpu6050

# https://github.com/adafruit/Adafruit_CircuitPython_MPU6050/blob/main/examples/mpu6050_simpletest.py
# 这个脚本各种报错，无法使用

i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

while True:
    # 线加速度
    print("Acceleration: X:%.2f, Y:%.2f, Z:%.2f m/s^2" % (mpu.acceleration))
    # 角加速度
    print("Gyro X:%.2f, Y:%.2f, Z:%.2f rad/s" % (mpu.gyro))
    # 温度
    print("Temperature: %.2f C" % mpu.temperature)
    print("")

    time.sleep(1)
print("done")
