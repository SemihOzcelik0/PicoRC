from machine import Pin, SPI, ADC, I2C
from ssd1306 import SSD1306_I2C
from nrf24l01 import NRF24L01
import machine
import network
import rp2
import struct
import socket
import time
import utime
import ast

# Functions

def setup():
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=16)
    nrf.open_tx_pipe(pipes[0])
    nrf.open_rx_pipe(1, pipes[1])
    nrf.start_listening()
    return nrf

def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)

def main(nrf):
    while True:
        try:
            nrf.send(struct.pack("iii", speed.read_u16(), aileron.read_u16(), elevator.read_u16()))
            utime.sleep(0.1)
        except:
            print('message cannot send')
            pass
        
        buf = nrf.recv()
        recieved = struct.unpack("ff", buf)
        altitude = recieved[0]
        gps_data = recieved[1]
        ast.literal_eval(gps_data)
        hiz= gps_data[0]    
        
        try:
            cl, addr = s.accept()
            print('client connected from', addr)
            request = cl.recv(1024)

            request = str(request)
            """led_on = request.find('/light/on')
            led_off = request.find('/light/off')
            stateis = "Led is off"

            if led_on == 6:
                led.value(1)
                stateis = "0"

            if led_off == 6:
                led.value(0)
                stateis = "45

            sensor_value = sensor.read_u16() * (3.3 / (65535))
            temperature = 0
            time.sleep(0.5) 27 - (sensor_value - 0.706) / 0.001721"""

            response = html.replace("%s", str(altitude)).replace("%s", str(hiz))

            cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
            cl.send(response)
            cl.close()

        except OSError as e:
            cl.close()
            print('connection closed')

# Defines

#NRF24L01
csn = Pin(15, mode=Pin.OUT, value=1)
ce  = Pin(14, mode=Pin.OUT, value=0)
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")

#SSD1306
i2c = I2C(0, scl = Pin(9), sda = Pin(8), freq = 400000)
display = SSD1306_I2C(128, 64, i2c)

#Pins
speed = ADC(Pin(28))
aileron = ADC(Pin(27))
elevator = ADC(Pin(26))

#Network
ssid = 'Pico'
password = '123456789'
new_ip = '192.168.4.2' #Static IP

ap = network.WLAN(network.AP_IF)
ap.config(essid=ssid, password=password)
ap.active(True)

ap.ifconfig((new_ip, '255.255.255.0', '192.168.4.1', '8.8.8.8'))

counter = 0
while ap.active() == False:
    counter += 1
    print("Wi-Fi connection creating - Try: {}".format(counter))
    time.sleep(0.5)
    if(counter == 60):
        raise OSError("Couldn't establish a Wi-Fi connection.")
    pass

print('WiFi aktive')
status = ap.ifconfig()
pico_ip = status[0]
print('ip = ' + status[0])

addr = (pico_ip, 80)
s = socket.socket()
s.bind(addr)
s.listen(1)
print('listening', addr)

file = open("index.html")
html = file.read()

nrf = setup()
auto_ack(nrf)
main(nrf)