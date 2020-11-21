import socket
from time import sleep
import math
import numpy
import pyrr
import json
import urllib.request
import time
from madgwickahrs import MadgwickAHRS
from quaternion import Quaternion

IP = "192.168.1.100"
PORT=8001
FREQ = 3 # Hertz
INTERVAL = 0.2




class IMU_Distance():
    def __init__(self,imu=0, master=None):
        self.INTERVAL = 0.2
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.index=0
        self.positionVectors=numpy.zeros(3)
        self.imu=imu
        self.currentAccelerations=numpy.zeros(3)
        self.previousAccelerations=numpy.zeros(3)
        self.currentRotationVector=""
        self.currentVelocities=numpy.zeros(3)
        self.previousVelocities=numpy.zeros(3)
        self.initialRotVector=""
        self.previousTime=0.0
        self.previousMotion=0
        self.madgwickahrs=0
        self.recieveLoop()
    def recieveLoop(self):
        index = 0
        print("here")
        tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #response = urllib.request.urlopen(URL)
        print(IP)
        print(PORT)
        tcp_client.connect((IP, PORT))
        try:
            while True:
                response = tcp_client.recv(2048)
                recievedData=json.loads(response.decode())
                print("here")
                self.newUpdateStateFunction(recievedData)
                print("here 4")
                #sleep(1/3)
        except Exception as e:
                print(e)
        finally:
                tcp_client.close()
    def updateState(self,out):
        #print(out)
        recievedQuaternion=out['rot_vector']['data'][1][1]
        timeDifference=0
        currentTime=float(out['rot_vector']['data'][1][0])
        if(self.previousTime!=0.0):
            timeDifference= (currentTime - self.previousTime) /1000
            self.previousTime=currentTime
        else:
            self.previousTime=currentTime
        
        print(str(timeDifference)+ " s")
        self.currentRotationVector=pyrr.quaternion.create(recievedQuaternion[0],recievedQuaternion[1],recievedQuaternion[2],recievedQuaternion[3]) 
    
        rotationMatrix=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentRotationVector)
        #print(out['rot_vector']['data'][1][0])
        #print(out['lin_accel']['data'][1][1])
        #print(out['accel']['data'][1][1])
        #print(self.currentVelocities)
        #print(rotationMatrix)
        linAcceleration=numpy.array(out['lin_accel']['data'][1][1])
        adjustedAccelerations=pyrr.matrix33.apply_to_vector(rotationMatrix, linAcceleration)

        #adjustedVelocities=self.currentVelocities + (adjustedAccelerations * timeDifference)+ ((adjustedAccelerations - self.currentAccelerations)*timeDifference *timeDifference * 0.5)
        adjustedVelocities= numpy.add(adjustedAccelerations,self.currentAccelerations) * timeDifference
        
        #self.positionVectors= self.positionVectors + (self.currentVelocities * timeDifference) + (adjustedAccelerations * (0.5 * timeDifference * timeDifference)) + ((adjustedAccelerations - self.currentAccelerations) * (timeDifference * timeDifference * timeDifference  ) / 6)
        
        self.positionVectors= numpy.add(self.positionVectors, numpy.array(adjustedVelocities * timeDifference))
        print(out['rot_vector']['data'])
        print()
        print(out['lin_accel']['data'])
        print()
        print("Adjusted Acceleration"+ str(adjustedAccelerations))
        print("Adjusted Velocities"+ str(adjustedVelocities))
        print("Distance " + str(self.positionVectors))
    
        self.currentVelocities=adjustedVelocities
        self.currentAccelerations= adjustedAccelerations

    def extractUsefulData(self,data):
        deserializedData=json.loads(data)
        print(deserializedData.encode("utf-8"))
        self.currentRotationVector=pyrr.quaternion.create() 
        #rotationMatrix=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentRotationVector)
        #vecotors=pyrr.matrix33.apply_to_vector(rotationMatrix, velocity_vector)

    def newUpdateStateFunction(self, data):
        rotationData=data['rotationVector']['value']
        linAccelerationData=data['linearAcceleration']['value']
        accelerometerData=data['accelerometer']['value']
        magData=data['magneticField']['value']
        gyroData=data['gyroscope']['value']
        
        
        #print(linAccelerationData)
        self.currentTime=data['accelerometer']['timestamp']
        
        self.currentRotationVector=rotationData
            
        print(numpy.array(self.currentRotationVector))

        if self.currentTime!=self.previousTime and self.previousTime!=0:
            timeDifference=(self.currentTime-self.previousTime)/1000
            #self.currentRotationVector=pyrr.quaternion.create(rotationData[0],rotationData[1],rotationData[2],rotationData[3])

            
            #self.madgwickahrs.update(gyroData,accelerometerData,magData,timeDifference)
                #return
            adjustedVector= numpy.subtract( numpy.array(self.currentRotationVector), numpy.array(self.initialRotVector))
            print("adjusted vector"+ str(adjustedVector))
            #adjustedAccelerations=np.dot(adjustedVector, linAccelerationData)
            
            adjustedAccelerations=numpy.zeros(3)
            # for i in range(0,len(adjustedAccelerations)):
            #     if abs(adjustedAccelerations[i]) < 0.01:
            #         adjustedAccelerations[i]=0
            #adjustedVelocities= numpy.add(self.currentVelocities,numpy.add(adjustedAccelerations * timeDifference, numpy.array(numpy.add(adjustedAccelerations,self.currentAccelerations*-1)*timeDifference *timeDifference * 0.5)))
            

            


            #print("quaternion from madgwick"+str(self.madgwickahrs.quaternion._get_q()))
            print("quaternion measured"+str(self.currentRotationVector))



            interm1=numpy.add(adjustedAccelerations, numpy.negative(self.currentAccelerations))/timeDifference
            interm2= interm1 * timeDifference *timeDifference * 0.5
            interm3=adjustedAccelerations * timeDifference
            interm4= numpy.add(interm2, interm3)

            
            adjustedVelocities= numpy.add(self.currentVelocities,interm4)
            # for i in range(0,len(adjustedVelocities)):
            #     if abs(adjustedVelocities[i]) < 0.01:
            #         adjustedVelocities[i]=0

            self.positionVectors= numpy.add(self.positionVectors, numpy.array(adjustedVelocities * timeDifference))
            self.previousTime=self.currentTime
            self.currentAccelerations=adjustedAccelerations
            self.currentVelocities=adjustedVelocities
            #print(str(timeDifference)+ " s")
            print("Adjusted Acceleration"+ str(adjustedAccelerations))
            print("unadjusted acceleration"+ str(accelerometerData))
            #print("rotation vectors"+str(self.currentRotationVector))
            #print("madgwick vectors"+str(self.madgwickahrs.quaternion._get_q()))
            print("Adjusted Velocities"+ str(adjustedVelocities))

            print("Distance " + str(self.positionVectors))

            #print("euler", Quaternion(self.currentRotationVector).to_euler123()[0]*180/3.14)
            print()

        elif self.previousTime==0:
            self.previousTime=self.currentTime
            #self.currentAccelerations=linAccelerationData
            self.currentAccelerations=numpy.zeros(3)
            #self.madgwickahrs=MadgwickAHRS(quaternion=Quaternion(i[1][0],i[1][1],i[1][2],i[1][3]))
            #self.madgwickahrs=MadgwickAHRS(quaternion=Quaternion(self.currentRotationVector))
            self.initialRotVector=self.currentRotationVector
            print("here 132")
class imu():
    def __init__(self,master=None):
        self.f = open("sensors.json", "r", encoding="utf-8")
    def getData(self):
       return(self.f.read())


IMU_Distance()