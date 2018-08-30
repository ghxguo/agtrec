from socket import *
import time

address= ( '192.168.2.177', 5000) #define server IP and port
client_socket =socket(AF_INET, SOCK_DGRAM) #Set up the Socket
client_socket.settimeout(1) #Only wait 1 second for a response
steering = 0
pedal = 0
engineCut = 0
while(1):
    # steering += 1
    # pedal += 1
    pedal = 1900
    TXdata = "AS{0}P{1}C{2}".format(steering, pedal, engineCut)
    client_socket.sendto(TXdata,address)
    print TXdata
    time.sleep(0.1)
