import sim
import time
import math
import numpy as np

'''
comando a colocar en coppelia
simRemoteApi.start(x)

donde x es el puerto a usar,si se usa el puerto "19997" no se necesita colocar este codigo
'''

def Connect(Port):
# Establece la conexión a Coppelia
# Port debe coincidir con el puerto de conexión Coppelia
# retorna el número de cliente o -1 si no puede establecer conexión

    sim.simxFinish(-1) #Cerrando todas las conexiones existentes
    clientID=sim.simxStart('127.0.0.1',Port,True,True,2000,5) # Conectarse
    
    if clientID == 0: 
        print(f'conectado al puerto: {Port}')
    else: 
        print('\nNo se pudo conectar al puerto. \n')
        exit()

    return clientID

#Para obtener el handle de una pieza
def Handle(Nombre):
    handle = sim.simxGetObjectHandle(clientID,Nombre,sim.simx_opmode_blocking)
    return handle[-1]

#Para obtener la posicion de un objeto
def Join_Position(handle):
    Location = sim.simxGetJointPosition(clientID, handle, sim.simx_opmode_blocking)
    return Location[-1]

def Position(handle):
    Location = sim.simxGetObjectPosition(clientID, handle, -1 ,sim.simx_opmode_blocking)
    return Location[-1]

def Posicion(Handle):
    Posicion = sim.simxGetObjectPosition(clientID,Handle,-1,sim.simx_opmode_buffer)
    return Posicion[-1][1]

#Funciones para mover el carro
def MoveLeft(Left,Right,Velocity):
    sim.simxSetJointTargetVelocity(clientID,Left,Velocity,sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID,Right,0,sim.simx_opmode_oneshot)
    

def MoveRight(Left,Right,Velocity):
    sim.simxSetJointTargetVelocity(clientID,Left,0,sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID,Right,Velocity,sim.simx_opmode_oneshot)

def Forward(Left,Right,Velocity):
    sim.simxSetJointTargetVelocity(clientID,Left,Velocity,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID,Right,Velocity,sim.simx_opmode_oneshot_wait)

def Stop(Left,Right):
    sim.simxSetJointTargetVelocity(clientID,Left,0,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID,Right,0,sim.simx_opmode_oneshot_wait)

def Reverse(Left,Right,Velocity):
    sim.simxSetJointTargetVelocity(clientID,Left,-Velocity,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID,Right,-Velocity,sim.simx_opmode_oneshot_wait)

#Funciones de sensores
def AproxSensor(sensor):
    Sensor = sim.simxReadProximitySensor(clientID,sensor,sim.simx_opmode_buffer)
    print(Sensor[1])
    return Sensor 

def LineSensor(Sensor):
    Vsensor = sim.simxReadVisionSensor(clientID,Sensor,sim.simx_opmode_buffer)
    return Vsensor[-1][0][1:4]


#Conectandose a Coppelia
clientID = Connect(19997)
Velocity = 100*math.pi/180
print(sim.simxGetPingTime(clientID))

#Handle de los joints
Sensor_Handle = Handle('Sensor')
Carro = Handle('Pioneer_p3dx')
MotorL = Handle('Left')
MotorR = Handle('Right')
Vsensor_L = Handle('Vsensor_L')
Vsensor_R = Handle('Vsensor_R')

#Inicializar sensores
sim.simxReadProximitySensor(clientID,Sensor_Handle,sim.simx_opmode_streaming)
sim.simxReadVisionSensor(clientID,Vsensor_L,sim.simx_opmode_streaming) 
sim.simxReadVisionSensor(clientID,Vsensor_R,sim.simx_opmode_streaming) 
sim.simxGetObjectPosition(clientID,Carro,-1,sim.simx_opmode_streaming)

# 0.02 tiempo minimo entre streaming y buffer
time.sleep(0.02)

#Para detectar linea negra
while True:
    SensorL = LineSensor(Vsensor_L)
    SensorR = LineSensor(Vsensor_R)

    #print(SensorL)
    #print(SensorR)
         
    if (SensorL == [0,0,0]) and (SensorR == SensorL):
        Stop(MotorL,MotorR)
        break 

#Para hacer un giro de 180 grados (en beta)
Pinitial = Posicion(Carro)

while True:
    MoveRight(MotorL,MotorR,Velocity)

    Pout = -Posicion(Carro)
    print(f'i = {Pinitial} ; f = {Pout}')

    if Pinitial == Pout:
        Stop(MoveLeft,MoveRight) 
        break

#Terminar conexion entre puertos
sim.simxFinish(clientID)