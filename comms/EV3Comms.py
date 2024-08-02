import socket 
from pybricks.tools import wait
#USAGE

#from EV3Comms import EV3Socket
#EV3 = EV3Socket(ip,port) 
# (ip and port optional)
#EV3.updateMotors(100, 400, 800)
#                   L   R    B

#Functions
#reconnect(ip, port) - attempts to restablished previously closed or lost connection
#updateMotors(leftSpeed, rightSpeed, rearSpeed) - input deg/sec speed for left, right, and rear motors
#kill() - closes connection 

class EV3Socket:
    
    def __init__(self, ip = "169.254.101.15", port = 4000, sock=None):
        self.reconnect(ip, port)

    def reconnect(self, ip, port, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        print("Attempting Connection")
        self.sock.connect((ip,port))
        print("Connected")

    def updateMotors(self, leftThrottle = 0, rightThrottle = 0, backThrottle = 0):
        L = str(leftThrottle).zfill(3)
        R = str(rightThrottle).zfill(3)
        B = str(backThrottle).zfill(3)

        wait(100)
        EV3Socket.stopRobot()

        payload = b"L" + L.encode('UTF-8') + b"R" + R.encode('UTF-8') + b"B" + B.encode('UTF-8')
        self.sock.send(payload)
        ack = self.sock.recv(4)
        if ack != b"RECV":
            print("Invalid ACK")

    def stopRobot(self):
        payload = b"S00000000000"
        self.sock.send(payload)
        ack = self.sock.recv(4)
        print(ack)
        if ack != b"RECV":
            print("Invalid ACK")

    def kill(self):
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        self.sock = None

    def setupServer(self, setIP = "192.168.10.108", setPort = 4000):
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

if __name__ == "__main__":
    EV3 = EV3Socket
    
    
    #EV3.updateMotors(100, 400, 800)

    #If receive stop from PC
        #EV3.stopRobot()
        #EV3.kill()