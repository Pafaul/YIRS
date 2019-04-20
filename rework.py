# -*- coding: utf-8 -*-
import math
import time
import dronekit
import cv2
import numpy as np

################################################
work_time = 60*5;

################################################
#videos
cap = None
video = None
res_video = None
log = open("log.txt", "w")
image_resolution = [0, 0]
color_red = (255, 0, 0)

################################################
#vehicle
vehicle=None

################################################
#physics
k = [ 1, 0.001, 0.001 ]
# первые два значения - текущее положение цели, последние два - предыдущее
distance_to_target = [ 0, 0, 0, 0 ]
# вычисляется на основе разницы расстояний и скорости
vehicle_speed = [ 0, 0 ]
# угол обзора камеры, полный
camera_angle = math.pi/3
# видимое расстояние, равно половине от всего
vision_radius = 0
# максимальное отклонение
max_angle = 20/180*math.pi

#для расстояния
dist_deadzone=0.5
#для скорости
velo_deadzone=0.2

#частота работы цикла
frequency = 5

################################################
#computer vision consts
hsv_min = np.array((0,  120, 120), np.unit8)
hsv_max = np.array((10, 220, 240), np.uint8)

################################################
#vehicle status
vehicle_angles = [ 0, 0, 0 ]
vehicle_height = 0
vehicle_channel_status = [ 0, 0 ]

################################################
#инициализация камеры
def initialize_cam():
    cap = cv2.VedeoCapture(0)
    flag, image = cap.read()
    image_resolution = image.shape[1::-1]
    return cap

def initialize_videos():
    video = cv2.VideoWriter("output_orig.avi", cv2.cv.CV_FOURCC(*'XVID'), 5, (image_resolution[0], image_resolution[1]))
    res_video = cv2.VideoWriter("output_result.avi", cv2.cv.CV_FOURCC(*'XVID'), 5, (image_resolution[0], image_resolution[1]))
    return [video, res_video]

################################################
#vehicle_initialization
def vehicle_initialization():
    vehicle=dronekit.connect('/dev/ttyACM0', baud=115200, wait_ready=True)
    while (not vehicle.is_armable):
        time.sleep(1)
    vehicle.mode = 'GUIDED'
    return vehicle

################################################
#physics
def calculate_distance(pixels, image_resolution, height, distance):
    distance = [ 0, 0 ]
    vision_radius = height*math.tan(camera_angle)
    for i in range(0, 1):
        distance[i+2] = distance[i]
        distance[i] = pixels[i]/image_resol[i]*vision_radius
        distance[i] = distance[i]

    return distance

def calculate_speed( distance, time ):
    velocity = [ 0, 0 ]
    for i in range(0, 1):
        velocity[i] = (distance[i] - distance[i+2])/time
    return velocity

def calculate_angles( distance, speed ):
    angles = [0, 0]
    for i in range(0, 1):
        angles[i] = k[0]*(k[1]*distance[i] + k[2]*speed[i])
        if (abs(angles[i]) > max_angle):
            angles[i] = max_angle(math.copysign(1, angles[i]))
            
    return angles

################################################
#computer_vision        

def process_image(image):
    img    = cv2.flip(img, 1)
    brg    = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    hsv    = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, hsv_min, hsv_max)

    moments = cv2.moments(thresh, 1)

    coords = None

    if moments['m00'] >= 10:
        coords = [ moments['m01']/moments['m00'], moments['m10']/moments['m00'] ]
    else:
        coords = [ -1, -1 ]

    return coords

################################################
#quad parameters

def get_attitude( vehicle ):
    #roll, pitch, yaw
    attitude = [0, 0, 0]
    attitude [0] = vehicle.attitude.roll
    attitude [1] = vehicle.attitude.pitch
    attitude [2] = vehicle.attitude.yaw

    return attitude

def get_altitude( vehicle ):
    return vehile.location.global_frame.alt

def get_vehicle_mode( vehicle ):
    return vehicle.mode

################################################
#управление дроном
def set_vehicle_attitude( vehicle, angles = [0, 0, 0], thrust = 0.5):
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, #time_boot_ms
        1, #target system
        1, #target component
        0b00000000, 
        to_quaternion( angles ), #quaternion
        0, #body roll rate in radian
        0, #body pitch rate in radian
        0, #body aw rate n radian/second
        thrust #thrust
    )
    vehicle.send_mavlink(msg)
    
################################################

def print_results_img( altitude, attitude, pixels, distance, velocity, angles, resolution, img ):
    printxy = [0, 0]
    for i in range(0, 1):
        printxy[i] = pixels[i] + resolution[i]/2
    cv2.circle(img, (printxy[0], printxy[1]), 5, color_red, 2)
    cv2.putText(img, "%d-%d" % (pixels[0], pixels[1]), (printxy[0]+10, printxy[1]-10),
                cv2.FONT_HERSHEY_SIMPLES, 1, color_red, 2)
    cv2.putText(img, "attitude: %d %d %d" %(attitude[0], attitude[1], attitude[2]), (10, 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "altitude: %d" %(altitude), (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "distance: %d %d" %(distance[0], distance[1]), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "velocity: %d %d" %(velocity[0], velocity[1]), (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    cv2.putText(img, "req angles: %d %d %d" %(angles[0], angles[1], angles[2]), (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
    return img

################################################
#main cycle

def main():
    #секция инициализации
    global cap
    global vehicle
    global video
    global res_video
    cap = initialize_cam()
    vehicle = vehicle_initialization()
    video, res_video = initialize_videos()
    start_height = get_altitude(vehicle)
    start_time = time.time()
    start_altitude = get_altitude(vehicle)
    start_attitude = get_attitude(vehicle)
    log.write('Start altitude: ' + str(start_altitude))
    log.write('Start attitude: ' + str(start_attitude))
    
    while (True):
        if vehicle.mode == 'GUIDED' or vehicle.mode == 'GuidedNoGps':
            current_time=time.time()
            flag, img = cap.read()
            resolution = img.shape[1::-1]
            video.write(img)
            #обработка изображения
            pixels = process_image(img)
            log.write('image pixel coords: ' + str(pixels))
            if ( pixels[0] != -1 ) and ( pixels[1] != -1 ):
                set_vehicle_attitude(vehicle)
                continue
            #"центровка" пиксельных координат
            for i in range(0, 1):
                pixels[i] = pixels[i] - resolution[i]/2
            #расчёт расстояния и скорости
            distance = calculate_distance( pixels, resolution, get_altitude( vehicle ), distance)
            velocity = calculate_speed( distance, time.time() - current_time )
            #проверка на попадание в мертвую зону
            for i in range(0, 1):
                if abs(distance[i]) < dist_deadzone:
                    distance[i] = 0
                if abs(velocity[i]) < velo_deadzone:
                    velocity[i] = 0
            #расчёт углов
            angles = calculate_angles( distance, speed )
            #задание необходимых наклонов и высоты
            set_vehicle_attitude( vehicle, angles, 0.5 )
            #получение изображения с результатами
            res_image = print_results_image(
                get_altitude(vehicle),
                get_attitude(vehicle),
                pixels,
                distance,
                velocity,
                angles,
                resolution,
                img )
            #запись результирующего видоса
            res_video.write(res_image)
            log.write('centered pixel coords: ' + str(pixels))
            log.write('distance to target: ' + str(distance))
            log.write('target speed: ' + str(velocity))
            log.write('required angles: ' + str(angles))
            log.write('attitude: ' + str(get_attitude(vehicle)))
            log.write('altitude: ' + str(get_altitude(vehicle)))
            #получение заданой частоты работы
            while (time.time() < current_time + 1/frequency):
                if vehicle.mode != 'GUIDED' || vehicle.mode != 'GuidedNoGps':
                    break
                
                
        else:
            if (time.time() - start_time  < work_time):
                if (vehicle.armed):
                    set_vehicle_attitude(vehicle)
                pass
            else:
                vehicle.close()
                video.release()
                res_video.release()
                cap.release()
                log.close()
################################################
#arm test
def arm_test():
    while ( not vehicle.armed ):
        vehicle.armed = True
        sleep(1)
    if (vehicle.armed):
        vehicle.armed = False

    vehicle.close()

#take off test
def take_off_test():
    global vehicle
    smooth_take_off=0.6
    fast_take_off=0.7
    smooth_landing=0.4
    vehicle = vehicle.initialization()
    start_height = get_altitude(vehicle)
    while ( not vehicle.armed ):
        vehicle.armed = True
        sleep(1)
    if (vehicle.armed):
        set_vehicle_attitude(vehicle, thrust = smooth_take_off)
        while (get_altitude(vehicle) - start_height < 0.95*2):
            if ( not safety_check(vehicle) ) break;
            
        set_vehicle_attitude(vehicle thrust = smooth_landing)
        while (get_altitude(vehicle) - start_height > 0.05*2):
            if ( not safety_check(vehicle) ) break;

        vehicle.armed = False

    vehicle.close()

#attitude changing test

def safety_check(vehicle, time = 0):
    if (time == 0):
        if vehicle.mode == 'GUIDED':
            return True
        else
            return False
    else:
        start_time = time.time()
        while (time.time() - start_time < time ):
            if vehicle.mode == 'GUIDED':
                pass
            else
                return False
        return True

def attitude_change_test():
    global vehicle
    smooth_take_off=0.6
    fast_take_off=0.7
    smooth_landing=0.4
    vehicle = vehicle.initialization()
    start_height = get_altitude(vehicle)
    stop = False
    while ( not vehicle.armed ):
        vehicle.arm()
        sleep(1)
    if (vehicle.armed):
        set_vehicle_attitude(vehicle, thrust = smooth_take_off)
        while (get_altitude(vehicle) - start_height < 0.95*2):
            if ( not safety_check(vehicle) ):
                stop = True
                break;
        
        if not stop:
            set_vehicle_attitude(vehicle, [2/180*math.pi, 0, 0])
        if ( not safety_check(vehicle, 1) ):
                stop = True
                break;
            

        if not stop:
            set_vehicle_attitude(vehicle, [-2/180*math.pi, 0, 0])
        if ( not safety_check(vehicle, 1) ):
                stop = True
                break;

        if not stop:
            set_vehicle_attitude(vehicle, [0, -2/180*math.pi, 0])
        if ( not safety_check(vehicle, 1) ):
                stop = True
                break;

        if not stop:
            set_vehicle_attitude(vehicle, [0, 2/180*math.pi, 0])
        if ( not safety_check(vehicle, 1) ):
                stop = True
                break;
        
        if not stop:
            set_vehicle_attitude(vehicle, thrust = smooth_landing)
        while (get_altitude(vehicle) - start_height > 0.05*2):
            if ( not safety_check(vehicle) ):
                stop = True
                break;
            
        vehicle.disarm()

    vehicle.close()

def check_thrust():
    global vehicle
    vehicle = initialize_vehicle()
    vehicle.arm()
    if (vehicle.armed):
        set_vehicle_attitude(vehicle, thrust = 0.5)
        sleep(1)
        set_vehicle_attitude(vehicle, thrust = 1)
        sleep(1)
        set_vehicle_attitude(vehicle, thrust = 0.1)
        sleep(1)

        vehicle.disarm()
    vehicle.close()
    
################################################
if __name__ == '__main__':
    main()
            
