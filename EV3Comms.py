import socket 

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
    
    def __init__(self, ip = "169.254.196.106", port = 4000, sock=None):
        self.reconnect(ip, port)

    def reconnect(self, ip, port, sock=None):
        if sock is None:
            self.sock = socket.socket(
                            socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        print("Attempting Connection")
        self.sock.connect((ip,port))
        print("Connected")

    def updateMotors(self, L = 0, R = 0, B = 0):
        payload = b"L" + str(L).encode('UTF-8') + b"R" + str(R).encode('UTF-8') + b"B" + str(B).encode('UTF-8')
        self.sock.send(payload)

        ack = self.sock.recv(4)
        if ack != b"RECV":
            print("Invalid ACK")

    def kill(self):
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        self.sock = None