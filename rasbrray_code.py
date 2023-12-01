#!/usr/bin/env python
# -*- coding: utf-8 -*-
#ver 1.5

import socket
import threading
import serial
import cv2
import numpy as np
import time

#모드값 대기(stanby)
mode = '3'
# Flask 웹 애플리케이션에서 클라이언트로부터 데이터를 받기 위한 변수
received_data = None

received_flag = 0
old_data = 3



# 시리얼 포트 설정
serial_port = serial.Serial(
    port="/dev/ttyAMA1",  # 라즈베리 파이의 시리얼 포트 경로
    baudrate=115200,      # 보드레이트 (STM32와 동일해야 함)
    timeout=1             # 타임아웃 설정
)

#라즈베리파이 서버 설정
def handle_client(client_socket):
    global received_data
    try:
        
        data = client_socket.recv(1024)
        
        decoded_data = data.decode()
        print("웹에서 받은 데이터:", decoded_data)
        if(decoded_data == '3') :
                camera_control(decoded_data)
        main(decoded_data)
    except Exception as e:
        print(f"클라이언트와의 연결이 종료되었습니다: {e}")
    finally:
        client_socket.close()

# MCU에서 모드송수신 확인을 위한 플래그
def read_data():
    if serial_port.in_waiting > 0:
        try:
            # received_data = serial_port.readline().decode('ISO-8859-1').strip()  # STM32에서 수신한 데이터 읽기
            # received_data = serial_port.readline().decode('utf-16').strip()  # STM32에서 수신한 데이터 읽기
            received_data = serial_port.readline().decode().strip()  # STM32에서 수신한 데이터 읽기
            if received_data:
                print(f"Received: {received_data}")
                if received_data[1] == '0':
                    print("MCU Mode")
                elif received_data[1] == "1":
                    print("RASPI: Line Tracing Mode")
                elif received_data[1] == "2":
                    print("RASPI: Color Tracing Mode")
        except UnicodeDecodeError as e:
            print("디코딩 오류:", e)

# MCU로 데이터 전송
def send_data(data):
    try:
        serial_port.write(data.encode())
        print(f"Sent: {data}")
    except Exception as e:
        print(f"Error while sending data: {str(e)}")

def camera_control(data):
    camera = cv2.VideoCapture(-1)
    camera.set(3, 360)
    camera.set(4, 240)

    while True:
        ret, frame = camera.read()

        if data == '1':
            # 라인 추적 모드
            crop_img = frame[100:200, 0:300]
            gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (9, 9), 0)
            ret, thresh1 = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY_INV)

            mask = cv2.erode(thresh1, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
                cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
                cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)

                print(cx)

                # 검정라인용
                if cx >= 100 and cx <= 180:
                    print("Go straight")
                    send_data("G1")
                elif cx > 180:
                    print("Turn right")
                    send_data("R1")
                elif cx < 80:
                    print("Turn Left")
                    send_data("L1")
                else:
                    print("Not found - Stop")
                    send_data("01")
            cv2.imshow('mask', mask)


        elif data == '2':
            # 컬러 추적 모드
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 70, 72])
            upper_red = np.array([7, 255, 255])

            mask = cv2.inRange(hsv, lower_red, upper_red)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if len(cnts) > 0:
                cnt = max(cnts, key=cv2.contourArea)
                (color_x, color_y), color_radius = cv2.minEnclosingCircle(cnt)

                if color_radius > 10:
                    cv2.circle(frame, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 2)
                    print(color_x)

                    if color_x >= 150 and color_x <= 210:
                        print("Go straight")
                        send_data("G2")
                    elif color_x > 210:
                        print("Turn right")
                        send_data("R2")
                    elif color_x < 150:
                        print("Turn Left")
                        send_data("L2")

                else:
                    print("Not found")
                    send_data("02")

            cv2.imshow("Color Tracking", frame)
        
        if data == '3':  
            print("카메라 릴리즈")
            break

        if cv2.waitKey(1) & 0xFF == 27:
            break

    camera.release()
    cv2.destroyAllWindows()
def MCU_mode():
    global received_flag
    camera = cv2.VideoCapture(-1)
    camera.set(3, 160)
    camera.set(4, 120)
    
    while camera.isOpened():
        ret, frame = camera.read()

        if not ret:
            break

        #read_data()
        cv2.imshow('normal', frame)        

        if cv2.waitKey(1) & 0xFF == 27:
            break

    camera.release()  # 카메라 릴리스
    cv2.destroyAllWindows()


def main(data):
    global old_data
    global received_flag
    if data == '0': # MCU 모드
        print("MCU")
        send_data("00")
        #MCU_mode()

    elif data == '1':   # 라인트레이싱 모드 
        print("라인트레이싱")
        send_data("01")
        camera_control(data)
        

    elif data == '2':   # 컬러트레이싱 모드

        print("컬러트레이싱")
        send_data("02")
        camera_control(data)
        
    elif data == '3':  # 대기모드
        print("Stand by")
        send_data("03")
        read_data()


if __name__ == '__main__':
    
     # 라즈베리파이 서버 설정
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('0.0.0.0', 5050)  # 모든 인터페이스에서 접속 허용
    server_socket.bind(server_address)
    server_socket.listen(5)
    print("연결 대기 중...")
    try:
        while True:
            client_socket, client_address = server_socket.accept()
            print(client_address, "와 연결됨")
            client_handler = threading.Thread(target=handle_client, args=(client_socket,))
            client_handler.start()
            
                
    except KeyboardInterrupt:
        print("서버 프로그램 종료")
    finally:
        server_socket.close()
    


    