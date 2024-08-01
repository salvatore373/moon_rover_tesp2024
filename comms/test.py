# SPDX-License-Identifier: MIT
# Copyright (c) 2020 Henrik Blidh
# Copyright (c) 2022-2023 The Pybricks Authors
# Code from https://pybricks.com/projects/tutorials/wireless/hub-to-device/pc-communication/

"""
Example program for computer-to-hub communication.

Requires Pybricks firmware >= 3.3.0.
"""

#import asyncio
#import socket 
#from contextlib import suppress

# Replace this with the name of your hub if you changed
# it when installing the Pybricks firmware.
#HUB_NAME = "Pybricks Hub"
#HUB_IP = "169.254.130.7"
#HUB_PORT = 4000

#sockaddr = socket.getaddrinfo(HUB_IP, HUB_PORT)[0][-1]
#print(sockaddr)
# Now you can use that address

# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# print("Attempting Connection")
# sock.connect((HUB_IP,HUB_PORT))
# print("connected")
from EV3Comms import EV3Socket
from time import sleep

ip = "169.254.101.15"
EV3 = EV3Socket(ip)

EV3.updateMotors("080", "080", "080")
print("a")
sleep(5)
print("b")
EV3.updateMotors("080", "080", "080")
sleep(5)
print("c")
EV3.stopRobot()
EV3.kill()

exit()
