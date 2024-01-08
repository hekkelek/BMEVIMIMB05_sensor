from m5stack import *
from m5ui import *
from uiflow import *
from machine import I2S, Pin, UART
import imu
import hat
import struct
import array

#setScreenColor(0x111111)

# Use the M5StickC built-in microphone.
#mic = I2S(I2S.NUM0, ws=Pin(0), sdin=Pin(34), mode=I2S.MASTER_PDM,
#    dataformat=I2S.B16, channelformat=I2S.ONLY_RIGHT,
#    samplerate=8000, dmacount=16, dmalen=256)

#Create a serial port instance
uart1 = UART(1, tx=1, rx=3)

#Initialize the serial port
uart1.init(115200, bits=8, parity=None, stop=1)

#initialize IMU and ENV.II module instances
imu0 = imu.IMU()
hat_env20 = hat.get(hat.ENV2)

measurements = [0 for i in range(6)]

while True:
  lcd.clear()
  lcd.setCursor(0,0)
  measurements[0] = imu0.acceleration[0]
  measurements[1] = imu0.acceleration[1]
  measurements[2] = imu0.acceleration[2]
  measurements[3] = imu0.gyro[0]
  measurements[4] = imu0.gyro[1]
  measurements[5] = imu0.gyro[2]
  #temp = int((hat_env20.temperature))
  #hum = int((hat_env20.humidity))
  #baro = hat_env20.pressure

  sendbuffer = [0 for i in range(6*4+2)]
  sendbuffer[0] = 0x55
  sendbuffer[1:5]   = list( struct.pack('f', measurements[0]) )
  sendbuffer[5:9]   = list( struct.pack('f', measurements[1]) )
  sendbuffer[9:13]  = list( struct.pack('f', measurements[2]) )
  sendbuffer[13:17] = list( struct.pack('f', measurements[3]) )
  sendbuffer[17:21] = list( struct.pack('f', measurements[4]) )
  sendbuffer[21:25] = list( struct.pack('f', measurements[5]) )
  sendbuffer[25] = 0xAA
  sendbuffer = bytearray(sendbuffer)
  
  lcd.print("Xa: ")
  lcd.print(measurements[0])
  lcd.print("\nYa: ")
  lcd.print(measurements[1])
  lcd.print("\nZa: ")
  lcd.print(measurements[2])
  lcd.print("\nX: ")
  lcd.print(measurements[3])
  lcd.print("\nY: ")
  lcd.print(measurements[4])
  lcd.print("\nZ: ")
  lcd.print(measurements[5])
  
  uart1.write(sendbuffer)
  wait_ms(1)

#while True:
    #mic.readinto(buffer)
    
    #uart1.write(buffer)
