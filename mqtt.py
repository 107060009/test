import paho.mqtt.client as paho
import time
import matplotlib.pyplot as plt
import numpy as np
mqttc = paho.Client()

# Settings for connection
host = "192.168.52.160"
topic= "tilt"
port = 1883
N=10
x = np.zeros(3*N)
y = np.zeros(3*N) # signal vector; create Fs samples
z = np.zeros(3*N)
tilt = np.zeros(3*N)
i=0
stop=0
k=1
num=1
# Callbacks
def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    global i,x,y,z,k,num
    print("i=",i)
    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n");
    if i==0:
        num=int(msg.payload)
        i+=1
    elif i==1:
        k=int(msg.payload)
        i+=1       
    else:
        j=i-2
        if j%4==0:
            j = int (j/4)
            x[j] = float(msg.payload)
        elif j%4==1:
            j = int (j/4)
            y[j] = float(msg.payload) 
        elif j%4==2:
            j = int (j/4)
            z[j] = float(msg.payload) 
        elif j%4==3:
            j = int (j/4)
            tilt[j] = float(msg.payload)
        i+=1


def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)

for n in range(0,12*N):
    mqttc.loop()

x=x[0:k]
y=y[0:k]
z=z[0:k]
tilt=tilt[0:k]

t= np.arange(0,num,num/k)
fig, ax = plt.subplots(2, 1)
ax[0].plot(t,x)
ax[0].plot(t,y)
ax[0].plot(t,z)
ax[0].set_xlabel('Timestamp')
ax[0].set_ylabel('acc value')
ax[0].legend(['x','y','z'])
ax[1].stem(t,tilt)
ax[1].set_xlabel('Timestamp')
ax[1].set_ylabel('tilt')
plt.show()
   