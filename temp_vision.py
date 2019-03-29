import cv2
import numpy as np

hsv_min = np.array((0,  120, 120), np.unit8)
hsv_max = np.array((10, 220, 240), np.uint8)

def process_image(image):
    img    = cv2.flip(img, 1)
    brg    = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    hsv    = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, hsv_min, hsv_max)

    moments = cv2.moments(thresh, 1)

    coords = None

    if moments['m00'] >= 30:
        coords = [ moments['m01']/moments['m00'], moments['m10']/moments['m00'] ]
    else:
        coords = [ -1, -1 ]

    return coords

if __name__ == '__main__':
    pass
    
    
    
