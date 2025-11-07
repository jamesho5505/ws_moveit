import minimalmodbus
import serial
import binascii
import time
import robotiq_gripper as rq

# instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 9, debug = True)

# instrument.serial.port                     # this is the serial port name
# instrument.serial.baudrate = 115200         # Baud
# instrument.serial.bytesize = 8
# instrument.serial.parity   = serial.PARITY_NONE
# instrument.serial.stopbits = 1
# instrument.serial.timeout  = 0.2          # seconds
# instrument.address = 9                         # this is the slave address number
# instrument.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode
# instrument.clear_buffers_before_each_transaction = True
#
# print(instrument)

# instrument.write_registers(1000,[0,0,0]) # Reset the gripper
# instrument.write_registers(1000,[256,0,0]) #Activate the gripper

# instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 9, debug = True)
\

# instrument.serial.baudrate = 115200         # Baud
myGripper = rq.RobotiqGripper(portname='/dev/ttyUSB1')
myGripper.printInfo()
# myGripper.resetActivate()   #activate()
time.sleep(1)
# myGripper.calibrate(0,85)
saved_a = -0.3744493392070485
saved_b = 86.12334801762114
myGripper._aCoef = saved_a
myGripper._bCoef = saved_b
myGripper.openmm = 85
myGripper.closemm = 0

myGripper.printInfo()
# print("0")
# myGripper.goTomm(0,150,255)
# myGripper.printInfo()
current_pos = myGripper.getPositionmm()
print("current pos:", current_pos)
time.sleep(1.5)
print("85")
myGripper.goTomm(85,255,255)
time.sleep(0.5)
print("30")
myGripper.goTomm(30,255,255)
myGripper.printInfo()
current_pos = myGripper.getPositionmm()
print("current pos:", current_pos)
print("aCoef:", myGripper._aCoef)
print("bCoef:", myGripper._bCoef)
# myGripper._mmToBit()