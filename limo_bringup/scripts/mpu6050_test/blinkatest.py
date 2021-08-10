import board
import busio

print("hello")

i2c = busio.I2C(board.SCL, board.SDA)
print("I2C 1 ok")

print("done")
