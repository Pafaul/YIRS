# -*- coding: cp1251 -*-
import math

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
max_angle = 25/180*math.pi

def calculate_angles(distance, speed):
      angles = [ k[0]*( k[1]*distance[0] + k[2]*speed[0] ),
                 k[1]*( k[1]*distance[1] + k[2]*speed[1] ) ]
      for i in range(0, 1):
          if (abs(angles[i]) > max_angle):
              angles[i] = max_angle*copysign(1, angles[i])
      return angles

def calculate_distance(pixels, image_resol, height):
    distance = [0, 0]
    vision_radius = height*math.tan(camera_angle)
    for i in range(0, 1):
        distance_to_target[i+2] = distance[i]
        distance_to_target[i] = pixels[i]/image_resol[i]*vision_radius
        distance[i] = distance_to_target[i]

    return distance

def calculate_speed ( time ):
    for i in range(0, 1):
        vehicle_speed[i] = (distance_to_target[i] - distance_to_target[i+2])/time;

    return vehicle_speed

if __name__ == "__main__":
    pass
