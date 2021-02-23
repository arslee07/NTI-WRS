"""

Useful links:
- HSV circle colors https://is.gd/57BafN (divide the value by 2!)
- ...

"""

import math
import time

import cv2 as cv
import pymurapi as mur

auv = mur.mur_init()


class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p_gain(self, value):
        self._kp = value

    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return output


class Color:
    s_min, v_min = 50, 50
    s_max, v_max = 255, 255

    def __init__(self, h_min, h_max, s_min=50, s_max=255, v_min=50, v_max=255):
        if h_max > 180:
            raise ArithmeticError('h_max is too large')
        if h_min < 0:
            raise ArithmeticError('h_min is too small')
        if h_min > h_max:
            raise ArithmeticError('h_min is larger than h_max')
        if h_max < h_min:
            raise ArithmeticError('h_max is smaller than h_min')

        # Hue (color)
        self.h_min = h_min
        self.h_max = h_max
        # Saturation (value of white (bigger is more))
        self.s_min = s_min
        self.s_max = s_max
        # Value (value of black (bigger is less))
        self.v_min = v_min
        self.v_max = v_max

    def __eq__(self, other_color):
        return other_color.h_min >= self.h_min and other_color.h_max <= self.h_max

    def __ne__(self, other_color):
        return other_color.h_min < self.h_min or other_color.h_max > self.h_max


def clamp(v, min_val, max_val):
    if v < min_val:
        return min_val
    if v > max_val:
        return max_val
    return v


def keep_depth(depth_to_set):
    try:
        error = auv.get_depth() - depth_to_set
        output = keep_depth.regulator.process(error)
        output = clamp(int(output), -100, 100)
        auv.set_motor_power(2, output)
        auv.set_motor_power(3, output)
    except AttributeError:
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(20)
        keep_depth.regulator.set_d_gain(2)


def keep_yaw(yaw_to_set, speed):
    def clamp_to_180(angle):
        if angle > 180.0:
            return angle - 360
        if angle < - 180.0:
            return angle + 360
        return angle

    try:
        error = auv.get_yaw() - yaw_to_set
        error = clamp_to_180(error)
        output = keep_yaw.regulator.process(error)
        output = clamp(int(output), -100, 100)
        speed = clamp(int(speed), -100, 100)
        auv.set_motor_power(0, clamp(int(speed - output), -100, 100))
        auv.set_motor_power(1, clamp(int(speed + output), -100, 100))
    except AttributeError:
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(0.8)
        keep_yaw.regulator.set_d_gain(0.5)


def find_form(image, color_range):
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv_min = (color_range.h_min, color_range.s_min, color_range.v_min)
    hsv_max = (color_range.h_max, color_range.s_max, color_range.v_max)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ = cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 100:
                continue
            moments = cv.moments(c)
            try:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                return True, x, y

            except ZeroDivisionError:
                return False, 0, 0

    return False, 0, 0


def form_on_start(image, color_range):
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv_min = (color_range.h_min, color_range.s_min, color_range.v_min)
    hsv_max = (color_range.h_max, color_range.s_max, color_range.v_max)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ = cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 300:
                continue
            ((_, _), (w, h), _) = cv.minAreaRect(c)
            (_, _), radius = cv.minEnclosingCircle(c)

            rectangle_area = w * h
            circle_area = radius ** 2 * math.pi
            aspect_ratio = w / h

            if rectangle_area > circle_area:
                moments = cv.moments(c)
                try:
                    x = int(moments["m10"] / moments["m00"])
                    y = int(moments["m01"] / moments["m00"])
                    return "CIRCLE", x, y

                except ZeroDivisionError:
                    return False, 0, 0

            else:
                if area < 0.9 * rectangle_area:
                    moments = cv.moments(c)
                    try:
                        x = int(moments["m10"] / moments["m00"])
                        y = int(moments["m01"] / moments["m00"])
                        return "TRIANGLE", x, y

                    except ZeroDivisionError:
                        return False, 0, 0
                else:
                    moments = cv.moments(c)
                    try:
                        x = int(moments["m10"] / moments["m00"])
                        y = int(moments["m01"] / moments["m00"])
                        return "SQUARE", x, y

                    except ZeroDivisionError:
                        return False, 0, 0

    return False, 0, 0


def stab_on_form(image, color_range):
    found, x, y = find_form(image, color_range)
    if found:
        x_center = x - (320 / 2)
        y_center = y - (240 / 2)

        try:
            output_forward = stab_on_form.regulator_forward.process(y_center)
            output_side = stab_on_form.regulator_side.process(x_center)

            output_forward = clamp(output_forward, -50, 50)
            output_side = clamp(output_side, -50, 50)

            auv.set_motor_power(0, -output_forward)
            auv.set_motor_power(1, -output_forward)
            auv.set_motor_power(4, -output_side)

        except AttributeError:
            stab_on_form.regulator_forward = PD()
            stab_on_form.regulator_forward.set_p_gain(0.8)
            stab_on_form.regulator_forward.set_d_gain(0.5)

            stab_on_form.regulator_side = PD()
            stab_on_form.regulator_side.set_p_gain(0.8)
            stab_on_form.regulator_side.set_d_gain(0.5)


COLORS = {
    'blue': Color(105, 135),
    'purple': Color(135, 165),
    'green': Color(45, 75),
    'orange': Color(0, 30)
}