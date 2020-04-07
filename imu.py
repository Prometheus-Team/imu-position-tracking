import socket
from time import sleep
import curses
import math
import numpy
import pyrr
import StatusToDictionary
#Create Connection Thread

INTERVAL = 0.2

tello_ip = '192.168.10.1'
tello_port = 8889
tello_adderss = (tello_ip, tello_port)



class IMU_Distance():
    def __init__(self, master=None):
        self.INTERVAL = 0.2
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.index=0
        self.createConnection()
        self.positionVectors=(0,0,0)
    def createConnection(self):
        local_ip = ''
        local_port = 8890
        self.socket.bind((local_ip, local_port))
        socket.sendto('command'.encode('utf-8'), tello_adderss)
    def recieveLoop(self):
        index = 0
        while True:
            self.index += 1
            response, ip = socket.recvfrom(1024)
            if response == 'ok':
                continue
            out = response.replace(';', ';\n')
            out = 'Tello State:\n' + out
            self.updateState(out)
            sleep(INTERVAL)
    def updateState(self,out):
        status=statusStringToDictionary(out)
        eulers=pyrr.euler.create(status.pitch, status.roll, status.yaw)
        pyrr.matrix33.create_from_eulers(eulers)
        pyrr.quaternion.create_from_eulers(eulers)
        pyrr.matrix33.apply_to_vector(mat, vec)
