import _thread
import rp2
import machine
from machine import Pin, SPI, PWM, SoftI2C
import bmp180
from servo import Servo
import struct
from nrf24l01 import NRF24L01
import utime

# NRF24L01
csn = Pin(15, Pin.OUT)
ce = Pin(14, Pin.OUT)
pipes = (b"\xd2\xf0\xf0\xf0\xf0", b"\xe1\xf0\xf0\xf0\xf0")

# GPS
uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))

#BMP180
i2c = SoftI2C(sda=Pin(16), scl = Pin(17), freq=100000)
handle_i2c = bmp180.BMP180(i2c)

# Outputs
servo_aileron1 = PWM(Pin(9))
servo_aileron1.freq(50)

servo_aileron2 = PWM(Pin(10))
servo_aileron2.freq(50)

servo_elevator = PWM(Pin(12))
servo_elevator.freq(50)

rpm = PWM(Pin(28))

def setup():
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=16)

    nrf.open_tx_pipe(pipes[0])
    nrf.open_rx_pipe(1, pipes[1])
    nrf.start_listening()

    return nrf

def get_gps_data():
    while True:
        if uart.any():
            data = uart.readline().decode('utf-8', 'ignore')
            utime.sleep(1)
            if data[0:6] == '$GPRMC':
                values = data.split(',')
                if len(values) >= 13:
                    try:
                        enlem = float(values[3])
                        enlem_hemisfer = values[4][0]
                        boylam = float(values[5])
                        boylam_hemisfer = values[6][0]
                        hiz = float(values[7])
                        saat = values[1][:2] + ':' + values[1][2:4] + ':' + values[1][4:6]
                        tarih = values[9][4:6] + '/' + values[9][2:4] + '/' + values[9][:2]
                        durum = values[2]

                        gps_data = [enlem, enlem_hemisfer, boylam, boylam_hemisfer, hiz, saat, tarih, durum]
                        return gps_data
                        
                    except ValueError:
                        print("Geçersiz GPS verisi")
                else:
                    print("Geçersiz GPS verisi uzunluğu")
            else:
                print("Geçersiz GPS verisi başlangıcı")

def avionics(nrf):
    while True:
        buf = nrf.recv()
        recieved = struct.unpack("iii", buf)

        elevator = recieved[0]
        aileron = recieved[1]
        speed = recieved[2]

        servo_elevator.duty_u16(elevator)
        servo_aileron1.duty_u16(aileron )
        servo_aileron2.duty_u16(aileron)

        rpm.duty_u16(speed)

        e = str(elevator)
        a = str(aileron)
        print("Elevator :" + e + " Aileron : " + a)
        utime.sleep_ms(22)

def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes

def transmitter_thread():
    while True:
        gps_data = get_gps_data()
        altitude = handle_i2c.altitude - ground
        nrf.send(struct.pack("fff", gps_data, altitude))
        print(gps_data)
        print(altitude)
        utime.sleep_ms(1000)
        
utime.sleep(5)
ground = handle_i2c.altitude

nrf = setup()

_thread.start_new_thread(transmitter_thread, ())

auto_ack(nrf)
avionics(nrf)

