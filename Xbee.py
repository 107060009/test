import serial
import time
import paho.mqtt.client as paho
import matplotlib.pyplot as plt
import numpy as np
mqttc = paho.Client()


# Settings for connection

host = '192.168.52.160'

topic= "tilt"

port = 1883


# Callbacks

def on_connect(self, mosq, obj, rc):

    print("Connected rc: " + str(rc))


def on_message(mosq, obj, msg):

    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n");


def on_subscribe(mosq, obj, mid, granted_qos):

    print("Subscribed OK")


def on_unsubscribe(mosq, obj, mid, granted_qos):

    print("Unsubscribed OK")


# Set callbacks

mqttc.on_message = on_message

mqttc.on_connect = on_connect

mqttc.on_subscribe = on_subscribe

mqttc.on_unsubscribe = on_unsubscribe


#Connect and subscribe

print("Connecting to " + host + "/" + topic)

mqttc.connect(host, port=1883, keepalive=60)

mqttc.subscribe(topic, 0)

#XBee setting
serdev = '/dev/ttyUSB2'
s = serial.Serial(serdev, 9600)
s.write("+++".encode())
char = s.read(2)
print("Enter AT mode.")
print(char.decode())
s.write("ATMY 0x233\r\n".encode())
char = s.read(3)
print("Set MY <BASE_MY>.")
print(char.decode())
s.write("ATDL 0x232\r\n".encode())
char = s.read(3)
print("Set DL <BASE_DL>.")
print(char.decode())
s.write("ATID 0x0\r\n".encode())
char = s.read(3)
print("Set PAN ID <PAN_ID>.")
print(char.decode())
s.write("ATWR\r\n".encode())
char = s.read(3)
print("Write config.")
print(char.decode())
s.write("ATMY\r\n".encode())
char = s.read(4)
print("MY :")
print(char.decode())
s.write("ATDL\r\n".encode())
char = s.read(4)
print("DL : ")
print(char.decode())
s.write("ATCN\r\n".encode())
char = s.read(3)
print("Exit AT mode.")
print(char.decode())
print("start sending RPC")
N=10
x = np.zeros(3*N)
y = np.zeros(3*N) # signal vector; create Fs samples
z = np.zeros(3*N)
num = np.zeros(N)
i=0
j=0
k=0
while i<=N:
    # send RPC to remote
    s.write("/status/run\r".encode())
    line=s.readline()
    line=line.decode()
    line=line.split()
    print(line)
    if len(line)==1:
        tmp=float(line[0])
        if tmp==100:
            while 1:
                line=s.readline()
                line=line.decode()
                line=line.split()
                tmp=float(line[0])
                if tmp==500:
                    break
             #   print(line)
                if len(line)==1:
                    num[j]=int(line[0])
                #    print(num[j])
                    j=j+1
                else:
                    x[k]=float(line[0])
                    y[k]=float(line[1])
                    z[k]=float(line[2])
               #     print(x[k],k)
                    k=k+1
    time.sleep(1)
    i=i+1

x=x[0:k]
y=y[0:k]
z=z[0:k]
#print(x)

tilt = np.zeros(k)
import math
mqttc.publish(topic, j)
mqttc.publish(topic, k)
for i in range(0,k):
    mqttc.publish(topic, x[i])
    mqttc.publish(topic, y[i])
    mqttc.publish(topic, z[i])
    a1 = math.atan(x[i] / math.sqrt(y[i]*y[i]+z[i]*z[i]))
    b1= math.atan(y[i] / math.sqrt(x[i]*x[i]+z[i]*z[i]))
    c1= math.atan(z[i] / math.sqrt(x[i]*x[i]+y[i]*y[i]))
    thex = a1*180/math.pi
    they =b1*180/math.pi
    thez =c1*180/math.pi
    if abs(thex) >=45 or abs(they) >=45:
        tilt[i]=1
    else:
        tilt[i]=0
    mqttc.publish(topic, tilt[i])

mqttc.publish(topic, 500)

t = np.arange(0,j,1)
plt.plot(t,num)
plt.xlabel('timestamp')
plt.ylabel('number')
plt.title('#collected data plot')
plt.show()

s.close()