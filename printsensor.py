import socket
from time import sleep
import math
import numpy
import pyrr
import json
import urllib.request
import time


URL = "http://192.168.1.109:8080/sensors.json"

index = 0
while True:
    response = urllib.request.urlopen(URL)
    recievedData=json.loads(response.read().decode())
    print(str(recievedData).encode('utf-8'))
    sleep(1/3)