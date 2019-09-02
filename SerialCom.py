import serial 


class SerialCom:    
    #arduinoOut = serial.Serial('COM7',115200,timeout =.2)
    
    def getConn(self,comm,baudRate):
        self.arduinoOut = serial.Serial(comm,baudRate)
        return self.arduinoOut

    def sendString(self,data):
        data += "\n"
        self.arduinoOut.write(data.encode())

    def sendAngles(self,data):
        data = "{},{},{},{}\n".format(data[0],data[1],data[2],data[3])
        self.sendString(data)

    def receiveString(self):
        data = self.arduinoOut.readline()
        #print(data)
        return data
    
    def receiveAngles(self):
        data = self.receiveString()
        data = data.strip(" ")
        angles  = []
        for angle in data:
            angles.append(int(angle))
        return angles
        
    def closeConn(self):
        self.arduinoOut.close()


def main():
    SerialObj = SerialCom()
    arduinoOut = SerialObj.getConn('COM9',9600)
    while True:
        string_ = input('>')
        SerialObj.sendString(string_)


    


if __name__=="__main__":
    main()