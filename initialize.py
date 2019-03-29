from __future__ import division
from __future__ import absolute_import

from dronekit import connect
from time import sleep

def startup():
    vehicle = connect('/dev/ttyAMA0', baud=57600)
    return vehicle

if __name__ == '__main__':
    vehicle = startup()
    print u"attitude: " + str(vehicle.attitude)
    print u"altitude: " + str(vehicle.location.global_frame.alt)
