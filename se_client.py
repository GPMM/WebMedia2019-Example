#! /usr/bin/env python3

from enum import Enum
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time

class ConnState(Enum):
    OK = 0
    ERR_PROTO = 1
    ERR_CLIENT_ID = 2
    ERR_UNAVAIL = 3
    ERR_BAD_CRED = 4
    ERR_NO_AUTH = 5
    IDLE = 6

def on_conn(client, userdata, flags, rc):
    userdata.handle_conn(flags, rc)

def on_msg(client, userdata, msg):
    userdata.handle_msg(msg.topic, str(msg.payload.decode("utf-8")))

class MQTT:
    def __init__(self, client_id = ''):
        self.state = ConnState.IDLE
        self.client_id = client_id
        self.topics = {}
        self.client = mqtt.Client(self.client_id, userdata = self)
        self.client.on_connect = on_conn
        self.client.on_message = on_msg

    def connect(self, host, port = 1883, keepalive = 60):
        self.host = host
        self.port = port
        self.keepalive = 60
        self.client.connect(self.host, self.port, self.keepalive)

    def handle_conn(self, flags, rc):
        self.state = ConnState(rc)
        if self.state != ConnState.OK:
            print('Connection error: ' + str(self.state))

    def handle_msg(self, topic, msg):
        if topic in self.topics:
            for callback in self.topics[topic]:
                callback(msg)

    def subscribe(self, topic, callback, qos = 0):
        if topic not in self.topics:
            self.topics[topic] = []
            self.topics[topic].append(callback)
            self.client.subscribe(topic, qos)
        else:
            self.topics[topic].append(callback)

    def loop(self):
        self.client.loop_forever()

class LED:
    instances = 0
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, False)
        LED.instances += 1

    def __del__(self):
        LED.instances -= 1
        if LED.instances == 0:
            GPIO.cleanup()

    def on(self):
        GPIO.output(self.pin, True)

    def off(self):
        GPIO.output(self.pin, False)

def led_cb(led):
    def led_func(msg):
        if msg.find('activate="true"') != -1:
            led.on()
        elif msg.find('activate="false"') != -1: 
            led.off()

    return led_func

def main():
    mqtt = MQTT('raspberry')
    mqtt.connect('169.254.180.85')

    led17 = LED(17)
    led27 = LED(27)
    led22 = LED(22)
    led18 = LED(18)
    led23 = LED(23)
    led24 = LED(24)

    mqtt.subscribe('30,45,5,5', led_cb(led27))
    mqtt.subscribe('30,45,5,5', led_cb(led24))
    mqtt.subscribe('30,75,5,5', led_cb(led23))
    mqtt.subscribe('30,105,5,5', led_cb(led22))
    mqtt.subscribe('30,135,5,5', led_cb(led17))
    mqtt.subscribe('30,135,5,5', led_cb(led18))

    mqtt.loop()

if __name__ == "__main__":
    main()
