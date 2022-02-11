import paho.mqtt.client as mqtt
import time

nInstances = 100
clients=[]

mqtt.Client.connected_flag = False
broker = "127.0.0.1"

def on_disconnect(client, userdata, rc = 0):
    client.loop_stop()

#create clients
for i in range(nInstances):
    # j = 0 is server instance, i = 1 is robot instance
    for j in range(2):
        name = ""
        recv_topic = ""
        send_topic = ""
        message = ""
        if j == 0:
            name = "server_instance" + str(i)
            send_topic = "test" + str(i) + "/server"
            message = "Test message from server_instance" + str(i)
        else:
            name = "robot" + str(i)
            send_topic = "test" + str(i) + "/robot"
            message = "Test message from robot" + str(i)

        client = mqtt.Client(name)
        client.connect(broker)
        client.loop_start()
        client.publish(send_topic, message)
        clients.append(client)

time.sleep(1)
for client in clients:
    client.disconnect()
time.sleep(1)