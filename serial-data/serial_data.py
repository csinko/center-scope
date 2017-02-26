## Serial Data Testing File
import serial
import time

ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=19200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        xonxoff=serial.XOFF,
        rtscts=False,
        dsrdtr=False
        )

ser.isOpen()
print("Initializing Serial")
print("Write Command")
ser.write(bytearray([253,254,65,4,239]))
time.sleep(.05)
ser.write(bytearray([253,254,17,8,199]))
time.sleep(.05)
ser.write(bytearray([253,254,33,8,62]))
time.sleep(.05)
ser.write(bytearray([253,254,21,6,165,174,2,0,193]))
time.sleep(.05)
ser.write(bytearray([253,254,37,6,82,87,1,0,108]))
time.sleep(.05)
ser.write(bytearray([253,254,17,1,248]))
time.sleep(.3)
#ser.write(bytearray([253,254,35,4,20,188,199]))
time.sleep(.05)

## LEFT COMMAND
#ser.write(bytearray([253,254,19,2,255,255,9]))

## DOWN COMMAND
#ser.write(bytearray([253,254,35,2,255,255,160]))


## RIGHT COMMAND
#ser.write(bytearray([253,254,19,2,0,1,42]))

## UP COMMAND
#ser.write(bytearray([253,254,35,2,0,1,131]))


time.sleep(2)
#Up/Down Stop Command
#ser.write(bytearray([253,254,35,2,0,0,132]))

#Left/Right Stop Command
#ser.write(bytearray([253,254,19,2,0,0,45]))

print("Stopped")
