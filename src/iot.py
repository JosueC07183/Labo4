#Se importan las librerias
import serial, time, json
import paho.mqtt.client as mqtt

# Client status
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.connected = True
        print("La conexión fue exitosa")
    else: 
        print("No se pudo realizar la conexión: ", rc)
        client.loop_stop()
def on_disconnect(client, userdata, rc):
    if(rc == 0):
        print("Se ha desconectado de forma exitosa")
    else:
        print("Sistema desconectado mediante el código: ", rc)
        
# El puerto a utilizar (/dev/ttyACM#) se determina corriendo ls /dev/tty* despues de conectar el MCU
datos = serial.Serial("/dev/ttyACM1",115200,timeout=1) 
print("Conectado al puerto serial /dev/ttyACM1")
client = mqtt.Client()
client.connected = False
client.on_connect = on_connect
client.on_disconnect = on_disconnect

broker ="iot.eie.ucr.ac.cr"
port = 1883
topic = "v1/devices/me/telemetry"
device = "r6ac0sfasqmjw0ydx6d8"
client.username_pw_set(device)
client.connect(broker, port)
diccionario = dict()
while client.connected != True:
    client.loop()
    time.sleep(2)

while (1):
    data = datos.readline()
    mensaje = data.decode('utf-8').replace('\r', "").replace('\n', "").strip()
    valores = mensaje.split(',') # Valores estaban separadas por coma

    diccionario["Eje X"] = valores[0]
    diccionario["Eje Y"] = valores[1]
    diccionario["Eje Z"] = valores[2]
    diccionario["V bat"] = valores[3]

    if(float( valores[3]) < 7):
        diccionario["Bat Baja"] = "Si"
    else:
        diccionario["Bat Baja"] = "No"
    
    output = json.dumps(diccionario)
    print(output)
    client.publish(topic, output)
