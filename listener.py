import paho.mqtt.client as mqtt

# BROKER = "test.mosquitto.org"
# BROKER = "127.0.0.1"
BROKER = "192.168.0.2"

PORT = 1883
TOPIC = "Some topic"

def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected with result code", rc)
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    print("Received:", msg.payload.decode())

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_forever()


