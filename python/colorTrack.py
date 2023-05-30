import cv2
import urllib.request
import numpy as np
import socket
import struct 
import array
 
def nothing(x):
    pass

#indirizzo ip dell ESP
url='http://192.168.1.4/cam-lo.jpg' 
cv2.namedWindow("live transmission", cv2.WINDOW_AUTOSIZE)
serverAddress = ("192.168.1.4", 4210);

#comandi da inviare 
last_cmd = 1;
prefer_distance = 166;
cmd_aux= 0
distance =0
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # creazione della socket 
 
#impostazione del colore target in base allo spazio colori HUE-SATURATION-VALUE 
#hue -> COLORE 
# S  -> SATURAZIONE 
# V  -> indica la luminosita del colore 

l_h, l_s, l_v = 153, 50,50 # valori minimi 
u_h, u_s, u_v = 255, 255, 255 #valori massimi 
#la loro differenza indica la soglia di threshold il quale vengono ammessi colori seppur diversi 

while True:
    img_resp=urllib.request.urlopen(url)
    imgnp=np.array(bytearray(img_resp.read()),dtype=np.uint8)
    frame=cv2.imdecode(imgnp,-1)
    #_, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, l_b, u_b)
    k=0
    cnts, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area=cv2.contourArea(c)
        if area>1000:
            k=1;
            cv2.drawContours(frame,[c],-1,(255,0,0),3)
            M=cv2.moments(c)
            cx=int(M["m10"]/M["m00"])
            cy=int(M["m01"]/M["m00"])
            cv2.circle(frame,(cx,cy),7,(0,0,0),-1)
    if(k==0):
     cy=0
     cx=0
    print(cx,cy)
    #dataOut=array.array('L',[ last_cmd,prefer_distance,cmd_aux ,cx , cy ,distance]) #metodo adottato per avere 8 byte su socket
    dataOut=array.array('L',[cx , cy ]) 
    sock.sendto(bytes(dataOut), serverAddress) # manda il pacchetto 
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("live transmission", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("res", res)
    key=cv2.waitKey(5)
    if key==ord('q'):
        break
   
 
cv2.destroyAllWindows()
