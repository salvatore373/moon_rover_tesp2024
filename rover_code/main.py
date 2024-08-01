#!/usr/bin/env pybricks-micropython
#Initial code copied from
# https://pybricks.com/projects/tutorials/wireless/hub-to-device/pc-communication/

import socket

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()
back_wheel = Motor(Port.A)
front_left_wheel = Motor(Port.B)
front_right_wheel = Motor(Port.C)
#sonic_sensor = UltrasonicSensor(Port.S1)

setIP = "169.254.196.106"
setPort = 4000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ai = socket.getaddrinfo(setIP, setPort)
print("Bind address info:", ai)
addr = ai[0][-1]

# A port on which a socket listened remains inactive during some time.
# This means that if you run this sample, terminate it, and run again
# you will likely get an error. To avoid this timeout, set SO_REUSEADDR
# socket option.
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((setIP,setPort))
s.listen(1)
print("Listening")


def robot_stop():
    front_left_wheel.hold()
    front_right_wheel.hold()
    back_wheel.hold()

# def turn_right(speed, time = 0):
#     front_left_wheel.run(speed)
#     front_right_wheel.run(0.25*speed)
#     back_wheel.run(-0.5*speed)
#     if time > 0:
#         wait(time)

# def turn_left(speed, time = 0):
#     front_left_wheel.run(0.25*speed)
#     front_right_wheel.run(speed)
#     back_wheel.run(-0.5*speed)
#     if time > 0:
#         wait(time)

# def move_backward(speed, time = 0):
#     front_left_wheel.run(-1 * speed)
#     front_right_wheel.run(-1 * speed)
#     back_wheel.run(speed) 
#     if time > 0:
#         wait(time)

# def move_forward(speed, time = 0):
#     front_left_wheel.run(speed)
#     front_right_wheel.run(speed)
#     back_wheel.run(-1 * speed) 
#     if time > 0:
#         wait(time) 

def main():

    while True:
        print("waiting to accept")
        res = s.accept()
        print("accepted")
        client_s = res[0]
        client_addr = res[1]
        print("Client address:", client_addr)
        print("Client socket:", client_s)

        req = client_s.recv(12)
        print("Request:")
        print(req)
        req = req.decode()
        if (req[1] == "S"):
            robot_stop()
        else:
            lth = int(req[1:4])
            rth = int(req[5:8])
            bth = int(req[9:12])
            front_left_wheel.run(lth)
            front_right_wheel.run(rth)
            back_wheel.run(-1 * bth)
        client_s.send(b"RECV")
        client_s.close()
        print()


if __name__ == "__main__":
    main()
