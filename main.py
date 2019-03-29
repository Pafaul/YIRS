# -*- coding: cp1251 -*-
import initialize_vehicle
import temp_control
import temp_vision
import temp_signals
import temp_physics

import cv2
import numpy as np
import time


cap = None
video = None
res_video = None
log = open("log.txt", "w")
image_resolution = [0, 0]
color_red = (255, 0, 0)
printxy = [0, 0]

#инициализация камеры
def initialize_cam():
    cap = cv2.VedeoCapture(0)
    time.sleep(5)
    video = cv2.VedeoWriter("output_orig.avi", cv2.cv.CV_FOURCC(*'XVID'), 10, (image_resolution[0], image_resolution[1]))
    res_video = cv2.VedeoWriter("output_result.avi", cv2.cv.CV_FOURCC(*'XVID'), 10, (image_resolution[0], image_resolution[1]))
    flag, image = cap.read()
    image_resolution = image.shape[1::-1]

#вывод параметров на изображение
#центр искомого объекта, требуемые углы, дистанция до объекта, его скорость, высота над землёй
def print_results_img(results, img):
    for i in range(0, 1):
        printxy[i] = results[i]+resolution[i]/2
    cv2.circle(img, (printxy[0], printxy[1]), 5, color_red, 2)
    cv2.putText(img, "%d-%d" % (results[0], results[1]), (printxy[0]+10, printxy[1]-10),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "angles: %d %d %d" % (results[1], results[2], results[3]), (10, 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "altitude: %d" % (results[4]), (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "distance: %d %d" % (results[5], results[6]), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "speed: %d %d" % (results[7], results[8]), (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "req angles: %d, %d, %d" % (results[9], results[10], results[11]), (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    return img

#проверка на продолжение основного цикла
def is_running( param ):
    if param < 5*60:
        return True
    else return False

#основной цикл
def main():
    #инициализация камеры и устройства
    initialize_cam();
    vehicle = initialize_vehicle.startup();

    #время начала работы
    start_time = time.time();

    while is_running(time.time() - start_time):
        #получение времени начала цикла, необходимо для вычисления скорости движения цели
        current_time = time.time()
        #получение изображения с камеры
        flag, image = cap.read()
        video.write(image)
        #получение положения цели в пикселях
        pixels = temp_vision.process_image(image)
        #если не найден объект на изображении, то пропускаем остальную часть
        if (pixels[0] == -1) and (pixels[1] == -1):
            temp_controls.send_signal()
            continue
        
        #центрирование координат полученных с изображения
        for i in range(0, 1):
            pixels[i] = pixels[i]-image_resolution[i]/2

        #получение текущего состояния устройства
        vehicle_params = temp_signals.get_vehicle_status(vehicle)
        #вычисление расстояния до цели
        distance = temp_physics.calculate_distance(pixels, , vehicle_params[3])
        #вычисление скорости цели
        speed = temp_physics.calculate_speed(time.time() - current_time)
        #вычисление углов, на которые нужно повернуться
        angles = temp_physics.calculate_angles()
        #формирование и отправка управляющего сигнала
        control_params = [ angles[0], angles[1], angles[2], 0.60 ]
        temp_controls.send_signal( control_params )

        res_image = print_results_img(results, image)
        res_video.write(res_image)

    vehicle.close()
    cap.release()
    video.release()

                
        
        
        
        
        
