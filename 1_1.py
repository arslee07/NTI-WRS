# import pymurapi as mur
# import cv2 as cv
# import numpy as np

# auv = mur.mur_init()

class Color:
    s_min, v_min = 50, 50
    s_max, v_max = 255, 255
    
    def __init__(self, h_min, h_max):
        if h_max > 180:
            raise ArithmeticError('h_max is too large')
        if h_min < 0:
            raise ArithmeticError('h_min is too small')
        if h_min > h_max:
            raise ArithmeticError('h_min is larger than h_max')
        if h_max < h_min:
            raise ArithmeticError('h_max is smaller than h_min')

        self.h_min = h_min
        self.h_max = h_max
      
    def __eq__(self, other_color):
        return (other_color.h_min >= self.h_min and other_color.h_max <= self.h_max)
    
    def __ne__(self, other_color):
        return (other_color.h_min < self.h_min or other_color.h_max > self.h_max)


COLOR_BLUE = Color(105, 135)
COLOR_PURPLE = Color(135, 165)
COLOR_GREEN = Color(45, 75)
COLOR_ORANGE = Color(0, 30)


print(COLOR_BLUE != Color(110, 120))

# while True:
#     print('Глубина: {:.2f}'.format(auv.get_depth()))
#     print('Курс: {:.2f}\n'.format(auv.get_yaw()))
    
#     image = auv.get_image_front()
#     print(image[100, 100])
    
    #auv.set_motor_power(0, 50)
    #auv.set_motor_power(1, -50)
    