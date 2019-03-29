# -*- coding: cp1251 -*-
import dronekit

#углы наклона и высота
vehicle_status = [ 0, 0, 0, 0 ]
vehicle_angles = [0, 0, 0]
vehicle_height = 0

def get_vehicle_status( vehicle ):
    vehicle_angles = vehicle.attitudde
    vehicle_height = vehicle.location.global_frame.alt

    for i in range(0, 2):
        vehicle_status[i] = vehicle_angles[i]
    vehicle_status[3] = vehicle_height

    return vehicle_status

if __name__ = '__main__':
    pass
