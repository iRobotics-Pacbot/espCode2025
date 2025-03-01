import socket
import struct
import time

class UDPPeer:

    def __init__(self):
        self.TOF_COUNT = 6
        self.MDNS = 'uiucpacbot.local'
        self.MY_IP = "192.168.0.101"
        self.UDP_PORT = 8089
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.MY_IP, self.UDP_PORT))




    #Single receive call for any incoming packet
    def processDin(self):
        packType, data = self._recvGeneric()
        if (packType == ord('a')):
            print("recvd data for mcl")
            #update the values fed into MCL
        elif (packType == ord('b')):
            print(f"chars recvd:{data.decode()}")
        else :
            print(f"recvd unnknown packType identifier {packType}")





    #Each type of data send has its own function to call
    #-----------------------------------------------------------------
    def sendMCLDout(self, MCLOut):
        pass

    def sendPath(self, path):
        pass

    def sendString(self, string):
        string_bytes = string.encode('utf-8')
        packed_bytes = bytes([ord('c')]) + string_bytes
        self._sendGeneric(packed_bytes)  # Send string as bytes
    #------------------------------------------------------------------


    def _sendGeneric(self, packed_data):
        self.sock.sendto(packed_data, (self.MDNS, self.UDP_PORT))
    def _recvGeneric(self):
        data = self.sock.recv(1024)
        return (data[0], data[1:]) #just seperate out to say the first param is the identifier




    # def recvMCL(self):
    #     data = self.sock.recv(1024)
    #     fmt = f"{2 * self.TOF_COUNT + 6}"
    #     unpackedData = struct.unpack(fmt, data)
    #     distances = unpackedData[:self.TOF_COUNT]
    #     TOFstds = unpackedData[self.TOF_COUNT:2*self.TOF_COUNT]
    #     x, y, vx, vy, stdvx, stdvy = unpackedData[2*self.TOF_COUNT:]
    #     return (distances, TOFstds, x, y, vx, vy, stdvx, stdvy)

    # def sendMCL(self, x, y, vx, vy, oldX, oldY):
    #     fmt = f"Bffffff"
    #     packedData = struct.pack(fmt, ord('a'), x, y, vx, vy, oldX, oldY)
    #     self.sock.sendto(packedData, (self.MDNS, self.UDP_PORT))
        
    # def sendPath(self, x, y):
    #     fmt = f"Bff"
    #     packedData = struct.pack(fmt, ord('b'), x, y)
    #     self.sock.sendto(packedData, (self.MDNS, self.UDP_PORT))


    def close(self):
        self.sock.close()



strings = [
    "poopoo",
    "deadbeef",
    "bro",
    "onetwothreefourfivesizseveneightnine",
    "Python Networking",
    "String Packing Test"
]
    




if __name__ == "__main__":
    peer = UDPPeer()
    i = 0
    while (True):
        peer.processDin()
        peer.sendString(strings[i])
        i = (i + 1)%len(strings)
        time.sleep(1)
