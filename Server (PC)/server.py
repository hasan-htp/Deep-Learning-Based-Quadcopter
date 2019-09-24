#!/usr/bin/env python

import random
import socket, select
from time import gmtime, strftime
from random import randint
import numpy as np
import sys
import argparse
from yolo import YOLO, detect_video
from PIL import Image
import os
import cv2


yolo=YOLO()

counter = 0

connected_clients_sockets = []

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

HOST = 'Hasan-PC'
PORT = 1445
server_socket.bind((HOST, PORT))
server_socket.listen(10)

connected_clients_sockets.append(server_socket)
buffer_size=4096

while True:

    read_sockets, write_sockets, error_sockets = select.select(connected_clients_sockets, [], [])

    for sock in read_sockets:

        if sock == server_socket:

            sockfd, client_address = server_socket.accept()
            connected_clients_sockets.append(sockfd)

        else:
            try:

                data = sock.recv(buffer_size)
                print("gelen veri: ")
                
                txt = data.decode()

                #if data:
                if txt[0]=='R':
                    counter=counter+1
                    print (txt)
                    img1 = '\\\\192.168.1.4\\PiShare\\image' + str(counter) + '.jpg'
                    image1 = Image.open(img1)
                    #r_image,predicted_class,box,score = yolo.detect_image(image)
                    r_image,predicted_class,predicted_score= yolo.detect_image(image1)
                    r_image.show()
                    print(predicted_class)
                    print(predicted_score)
                    
                    mes1='Goruntu islenmistir'
                    sock.sendall(mes1.encode())

                    if len(predicted_class)>= 2:
                        max_index = (predicted_score==np.max(predicted_score))
                        maxElement_class = predicted_class[max_index]
                        print(maxElement_class)
                        maxElement_class=str(maxElement_class)
                        sock.sendall(maxElement_class.encode())
                    elif len(predicted_class)==1 : 
                        predicted_class=str(predicted_class)
                        sock.sendall(predicted_class.encode())
                    else:
                        mes2='Nesne Yok'
                        sock.sendall(mes2.encode())
     
                elif txt[0]=='B':
                    print (txt)
                    print("byeeeeeeeeeeeeeeeeeee")
                    sock.shutdown()

            except:
                sock.close()
                connected_clients_sockets.remove(sock)
                continue

server_socket.close()