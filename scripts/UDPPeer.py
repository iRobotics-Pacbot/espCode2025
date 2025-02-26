import socket
import struct
import time

class UDPPeer:

    def UDPPeer(self):
        self.TOF_COUNT = 6
        self.MDNS = 'uiucpacbot.local'
        self.MY_IP = "192.168.0.101"
        self.UDP_PORT = 8089
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.MY_IP, self.UDP_PORT))

    def recvMCL(self):
        data, addr = self.sock.recv(1024)
        fmt = f"{2 * self.TOF_COUNT + 6}"
        unpackedData = struct.unpack(fmt, data)
        distances = unpackedData[:self.TOF_COUNT]
        TOFstds = unpackedData[self.TOF_COUNT:2*self.TOF_COUNT]
        x, y, vx, vy, stdvx, stdvy = unpackedData[2*self.TOF_COUNT:]
        return (distances, TOFstds, x, y, vx, vy, stdvx, stdvy)

    def sendMCL(self, x, y, vx, vy, oldX, oldY):
        fmt = f"Bffffff"
        packedData = struct.pack(fmt, ord('a'), x, y, vx, vy, oldX, oldY)
        self.sock.sendto(packedData, (self.MDNS, self.UDP_PORT))
        
    def sendPath(self, x, y):
        fmt = f"Bff"
        packedData = struct.pack(fmt, ord('b'), x, y)
        self.sock.sendto(packedData, (self.MDNS, self.UDP_PORT))

    def close(self):
        self.sock.close()




    




# if __name__ == "__main__":
#     peer = UDPPeer()
#     while (True):
#         print(peer.recvMCL)
#         peer.sendMCL(1.11, 2.22, 3.33, 4.44, 5.55, 6.66)
#         peer.sendPath(1.22, 2.33)
#         time.sleep(1)
