#!/usr/bin/env python3
import serial
import paho.mqtt.client as mqtt
import json
import threading


def on_connect(client, userdata, flags, rc):
    threading.Thread(target=start_gw).start()

def start_gw():
    s = serial.Serial("/dev/ttyACM1", baudrate=115200)

    with open("entities.json") as f:
        entities = json.load(f)

    for id, entity in enumerate(entities):
        entity['state_topic'] = f'homeassistant/sensor/{id}/state'
        entity.setdefault('device', {})['identifiers'] = ['mujsenzor']
        entity['device']['name'] = 'MujSenzor'
        entity['unique_id'] = id

        client.publish(f"homeassistant/{entity['type']}/{id}/config", json.dumps(entity))

    while True:
        try:
            line = s.readline().strip().decode('utf-8')
            print(line)
            parts = line.split(' ')
            if len(parts) != 3:
                print("SKIP")
                continue

            seq, id, val = parts

            entity = entities[int(id)]
            if entity['type'] == 'binary_sensor':
                val = 'ON' if int(val) else 'OFF'
            elif entity['type'] == 'cover':
                val = int(val)
                vals = ['open', 'closed', 'opening', 'closing', 'stopped']
                if val < len(vals):
                    val = vals[val]
                else:
                    print("Invalid value for cover:", val)
                    continue

            client.publish(entity['state_topic'], val)
        except UnicodeDecodeError as e:
            print(e)


client = mqtt.Client()
client.on_connect = on_connect
client.connect('localhost')
client.loop_forever()
