import serial
arduinoSerialData = serial.Serial("/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_75435353038351F06192-if00",9600)
#where we tell the car to do stuff
while (True):
    print("Options:")
    print("w Forward")
    print("a Left")
    print("d Right")
    print("s Backward")
    print("q Park")
    serialCommand = input("Serial Variable: ")
arduinoSerialData.write(serialCommand.encode())