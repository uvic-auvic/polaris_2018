from serial import Serial as ser

port = raw_input("/dev/ttyUSB")
baud = raw_input("Baudrate: ")

dev = ser("/dev/ttyUSB" + port, baudrate=baud)

while True:
    dev.flush()

    dev.write("RID\n")

    try:
        print "%s" % dev.readline() 
    except KeyboardInterrupt:
        print "Failed to receive"

    cont = raw_input("Continue? (y/n)")



    if cont == "n":
        break

dev.close()

