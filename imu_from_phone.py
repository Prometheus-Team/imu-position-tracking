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

from kalman import Kalman_Accel

from pynput.keyboard import Listener, Key
import threading

URL = "http://192.168.137.139:8080/sensors.json"
#URL="http://10.5.214.189:8080/sensors.json"
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
        self.currentQuaternion=0
        self.currentVelocities=numpy.zeros(3)
        self.previousVelocities=numpy.zeros(3)
        self.initialQuaternion=""
        self.previousTime=0.0
        self.previousMotion=0
        self.madgwickars=0
        self.previousOut=numpy.zeros(3)
        self.previousMeasurement=numpy.zeros(3)
        self.kalmanX=0
        self.kalmanY=0
        self.kalmanZ=0
        self.num=1
        self.average=numpy.zeros(3)
        self.up =False
        self.var=True
        t = threading.Thread(target=self.recieveLoop, args=())
        t.start()
        with Listener(on_press=self.onPress, on_release=self.onRelease) as listener:
            listener.join()
            t.join()


    def onPress(self,key):
        if key == Key.up and not self.up:
            self.up = True
    def onRelease(self,key):
        if key == Key.up or key == Key.down:
            self.up = False
    def lowPass(self,acceleration):
        out=numpy.zeros(3)
        for index,i in enumerate(acceleration,start=0):
            out[index]=self.previousAccelerations[index]*0.15+ acceleration[index] * 0.85
        return numpy.array(out)
    def recieveLoop(self):
        index = 0
        while True:
            response = urllib.request.urlopen(URL)
            recievedData=json.loads(response.read().decode())
            self.newUpdateStateFunction(recievedData)
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
        self.currentQuaternion=pyrr.quaternion.create(recievedQuaternion[0],recievedQuaternion[1],recievedQuaternion[2],recievedQuaternion[3]) 
    
        rotationMatrix=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentQuaternion)
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
        self.currentQuaternion=pyrr.quaternion.create() 
        #rotationMatrix=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentQuaternion)
        #vecotors=pyrr.matrix33.apply_to_vector(rotationMatrix, velocity_vector)

    def highpass(self,accelerations):
        out=numpy.zeros(3)
        for index,i in enumerate(accelerations,start=0):
            dt=(self.currentTime-self.previousTime)/1000
            alpha = dt / (dt + 1 / (2 * numpy.pi * 0.01))
            out[index]= alpha * (self.previousOut[index] + i - self.previousMeasurement[index])
        self.previousOut=out
        return out
        

    def newUpdateStateFunction(self, data):
        if self.up==True:
            rotationData=data['rot_vector']['data']
            linAccelerationData=data['lin_accel']['data']
            accelerometerData=data['accel']['data']
            magData=data['mag']['data']
            gyroData=data['gyro']['data']
            

            #print(linAccelerationData)
            for index,i in enumerate(rotationData,start=0):
                self.currentTime=i[0]
                if self.up==False:
                    break
                timeDifference=(self.currentTime-self.previousTime)/1000
                
                    
                self.currentQuaternion=pyrr.quaternion.create(i[1][0],i[1][1],i[1][2],i[1][3])
                    
                quat=self.currentQuaternion
                if self.currentTime!=self.previousTime and self.previousTime!=0:
                    
                    if index < len(gyroData) and  index < len(accelerometerData) and  index < len(magData) :
                        self.madgwickahrs.update(gyroData[index][1],accelerometerData[index][1],magData[index][1],timeDifference)
                    #elif  index < len(gyroData) and  index < len(accelerometerData):
                        #self.madgwickahrs.update_imu(gyroData[index][1],accelerometerData[index][1] ,timeDifference)

                    #self.currentQuaternion=pyrr.Quaternion.from_eulers(self.madgwickahrs.quaternion.to_euler123())
                    
                    #c= self.currentQuaternion-self.initialQuaternion

                    a=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentQuaternion)
                    b= pyrr.matrix33.create_from_quaternion(self.initialQuaternion)
                    adjustedAccelerations1=pyrr.matrix33.apply_to_vector(a, linAccelerationData[index][1])
                    adjustedAccelerations=pyrr.matrix33.apply_to_vector(b, adjustedAccelerations1)
                    #adjustedAccelerations=adjustedAccelerations- numpy.array([ 1.14548891e-05 , 3.24514926e-05, -3.21555242e-04])
                    #filtered=self.highpass(adjustedAccelerations)

                    #for i in range(0,len(adjustedAccelerations)):
                    #    if abs(adjustedAccelerations[i]) < 0.01:
                    #       adjustedAccelerations[i]=0

                    kalmanx=self.kalmanX.update(adjustedAccelerations[0])
                    kalmany=self.kalmanY.update(adjustedAccelerations[1])
                    kalmanz=self.kalmanZ.update(adjustedAccelerations[2])
                    print( "x"+ str(kalmanx))
                    print("y"+ str(kalmany))
                    print("z"+ str(kalmanz))

                    #adjustedAccelerations= [kalmanx[2],kalmany[2],kalmanz[2]]
                    #adjustedAccelerations= self.lowPass(adjustedAccelerations)
                
                    #for i in range(0,len(adjustedAccelerations)):
                    #    if abs(adjustedAccelerations[i]) < 0.01:
                    #        adjustedAccelerations[i]=0
                    #adjustedVelocities= numpy.add(self.currentVelocities,numpy.add(adjustedAccelerations * timeDifference, numpy.array(numpy.add(adjustedAccelerations,self.currentAccelerations*-1)*timeDifference *timeDifference * 0.5)))

                    #adjustedVelocities=[kalmanx[1],kalmany[1],kalmanz[1]]


                    #adjustedAccelerations=numpy.array([self.kalmanX.update(adjustedAccelerations[0]),self.kalmanY.update(adjustedAccelerations[1]),self.kalmanZ.update(adjustedAccelerations[2])])


                    #print("quaternion from madgwick"+str(self.madgwickahrs.quaternion._get_q()))
                    #print("quaternion measured"+str(self.currentQuaternion))
                    
                    for i in range(0,len(adjustedAccelerations)):
                        if abs(adjustedAccelerations[i]) < 0.02:
                            adjustedAccelerations[i]=0

                    #self.average= (self.average * self.num + adjustedAccelerations)/( self.num + 1)
                    #self.num+=1

                    interm1=numpy.add(adjustedAccelerations, numpy.negative(self.currentAccelerations))/timeDifference
                    interm2= numpy.array(interm1) * timeDifference *timeDifference * 0.5
                    #interm2=0
                    interm3=numpy.array(self.currentAccelerations) * timeDifference
                    interm4= numpy.add(interm2, interm3)

                    
                    adjustedVelocities= numpy.add(self.currentVelocities,interm4)

                    for k in range(0,len(adjustedVelocities)):
                        if abs(adjustedVelocities[k]) < 0.1:
                            adjustedVelocities[k]=0.0
                    interm5=0.5*interm3*timeDifference
                    interm6=(interm2 * timeDifference )/3
                    #self.positionVectors=numpy.add(self.positionVectors,interm5)

                    #self.positionVectors=numpy.add(self.positionVectors,interm6)
                    
                    #adjustedVelocities=numpy.array([kalmanx[1],kalmany[1],kalmanz[1]])

                    self.positionVectors= numpy.add(self.positionVectors, numpy.array(adjustedVelocities * timeDifference))


                    self.previousTime=self.currentTime
                    self.currentAccelerations=adjustedAccelerations
                    self.currentVelocities=adjustedVelocities

                    self.previousMeasurement=linAccelerationData[index][1]
                    print(str(timeDifference)+ " s")
                    print(str(self.num))
                    #print("Average"+str(self.average))
                    print("Adjusted Acceleration"+ str(adjustedAccelerations))
                    
                    #print("Filtered Acceleration"+ str(filtered))
                    print("unadjusted acceleration"+ str(data['lin_accel']['data'][index][1]))
                    print("rotation vectors"+str(quat ))
                    print("madgwick vectors"+str(self.madgwickahrs.quaternion.q))
                    print("Adjusted Velocities"+ str(adjustedVelocities))

                    print("Distance " + str(self.positionVectors))
                    
                    # for j in range(0,len(self.positionVectors)):
                    #     if abs(self.positionVectors[j])>1:
                    #         sleep(10)


                    #print("euler", Quaternion(self.currentQuaternion).to_euler123()[0]*180/3.14)
                    print("__________________________________________________")

                elif self.previousTime==0:
                    self.previousTime=self.currentTime
                    #self.currentAccelerations=linAccelerationData[index][1]
                    #self.madgwickahrs=MadgwickAHRS(quaternion=Quaternion(i[1][0],i[1][1],i[1][2],i[1][3]))
                    self.madgwickahrs=MadgwickAHRS(quaternion=Quaternion(self.currentQuaternion))
                    self.initialQuaternion=self.currentQuaternion
                    self.previousMeasurement=linAccelerationData[index][1]
                    self.kalmanX=Kalman_Accel(0)
                    self.kalmanY=Kalman_Accel(0)
                    self.kalmanZ=Kalman_Accel(0)
        else: 
            self.currentAccelerations=numpy.zeros(3)
            self.currentVelocities=numpy.zeros(3)
            self.previousAccelerations=numpy.zeros(3)
            print("Not Moving")
            
            print("Distance " + str(self.positionVectors))
            if self.previousTime:
                print("rotation vectors"+str(self.currentQuaternion))
            #distanceY= (self.positionVectors[0]**2 + self.positionVectors[1]**2)**0.5
            #print("Distance " + str(distanceY)+","+str(self.positionVectors[2]))
            #rotationData=data['rot_vector']['data']
            #for index,i in enumerate(rotationData,start=0):
            #    self.currentTime=i[0]
class imu():
    def __init__(self,master=None):
        self.f = open("sensors.json", "r", encoding="utf-8")
    def getData(self):
       return(self.f.read())


def main():
    IMU_Distance()

if __name__=='__main__':
    main()