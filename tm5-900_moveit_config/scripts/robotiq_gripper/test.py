import minimalmodbus
import serial
import binascii
import time
import robotiq_gripper as rq

instrument = minimalmodbus.Instrument('COM3', 9, debug = True)

# instrument.serial.port                     # this is the serial port name
instrument.serial.baudrate = 115200         # Baud
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

instrument = minimalmodbus.Instrument('COM3', 9, debug = True)
instrument.serial.baudrate = 115200         # Baud
myGripper = rq.RobotiqGripper(portname='COM3',slaveaddress=9)
myGripper.activate()
time.sleep(5)
myGripper.calibrate(0,40)
myGripper.goTomm(20,255,255)