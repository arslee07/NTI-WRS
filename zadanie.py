
import pymurapi as mur
import cv2 as cv
import time
import math

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


def find_form(image,min,max):
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)    
    hsv_min = (min,50,50)
    hsv_max = (max,255,255)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ =cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 100:
                continue
            moments = cv.moments(c)
            try:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                return True, x , y
                
            except ZeroDivisionError:
                return False, 0, 0   
            
    return False, 0, 0




def form_on_start(image,min,max):
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv_min = (min,50,50)
    hsv_max = (max,255,255)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ =cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 300:
                continue
            ((_, _), (w,h), _) = cv.minAreaRect(c)
            (_, _), radius = cv.minEnclosingCircle(c)
            
            rectangle_area = w * h
            circle_area = radius ** 2 * math.pi
            aspect_ratio = w / h
            
            
            
            if rectangle_area > circle_area:
                moments = cv.moments(c)
                try:
                    x = int(moments["m10"] / moments["m00"])
                    y = int(moments["m01"] / moments["m00"])
                    return "circle", x , y
                    
                except ZeroDivisionError:
                    return False, 0, 
                
            else: 
                if  area < 0.9 * rectangle_area:
                    moments = cv.moments(c)
                    try:
                        x = int(moments["m10"] / moments["m00"])
                        y = int(moments["m01"] / moments["m00"])
                        return "treugolnik", x , y
                        
                    except ZeroDivisionError:
                        return False, 0, 0
                else:
                    moments = cv.moments(c)
                    try:
                        x = int(moments["m10"] / moments["m00"])
                        y = int(moments["m01"] / moments["m00"])
                        return "kvadrat", x , y
                        
                    except ZeroDivisionError:
                        return False, 0, 0                        
                        
                   
                   
            
    return False, 0, 0
    

def stab_on_form(image,min,max):
    found, x, y = find_form(image,min,max)
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



time.sleep(1)
now = time.time()
while True:
    
    image = auv.get_image_bottom()

    time.sleep(0.02)
    stab_on_form(image,120,160)
    if time.time() - now > 10:
        break
     
     

def kurs():
    while True:
        
        image = auv.get_image_bottom()
        found, x,y = form_on_start(image,120,160)
        time.sleep(0.02)
        
        if found == 'treugolnik':
            color = 40
        if found == 'kvadrat':
            color = 0           
        if found == 'circle':
            color = 90          
            
        
        found, x,y = find_form(image,color,color + 40)
        stab_on_form(image,120,160)
        
        print(x,y)
        
        auv.set_motor_power(0,-5)
        auv.set_motor_power(1,5)
        keep_depth(1.7)
       
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)

        if (158 < x < 162) and y < 120:
            kurs = auv.get_yaw()
            break 
       
    now = time.time()
    while True:
        
        image = auv.get_image_bottom()
        stab_on_form(image,120,160)
        
        time.sleep(0.02)  
        keep_yaw(kurs,0)
        keep_depth(1.7)
                
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)
        
        if time.time() - now > 10:
            break
    
kurs()




now = time.time()
while True:
    auv.set_motor_power(1,40)
    auv.set_motor_power(0,40)
    time.sleep(0.02)
    if time.time() - now > 5:
        break
    
   

now = time.time()    

while True:
    image = auv.get_image_bottom()
    found, x,y = form_on_start(image,15,30)
    time.sleep(0.02)

    
     
    if found == 'circle':        
        stab_on_form(image,15,30)
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)
        if time.time() - now > 25: 
            break      





auv.open_grabber() 





now = time.time()
while True:
    time.sleep(0.02)
    keep_depth(3.65)
    if time.time() - now > 15:
        
        break    

time.sleep(1)    
auv.close_grabber()        
time.sleep(1)


now = time.time()
while True:
    time.sleep(0.02)
    keep_depth(1)
    if time.time() - now > 5:
        
        break 







def find_rect(image,min,max):
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv_min = (min,50,50)
    hsv_max = (max,255,255)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ =cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 300:
                continue
            ((_, _), (w,h), _) = cv.minAreaRect(c)
            (_, _), radius = cv.minEnclosingCircle(c)
            
            rectangle_area = w * h
            circle_area = radius ** 2 * math.pi

            
            if 4 < w/h < 7:
                moments = cv.moments(c)
                try:
                    x = int(moments["m10"] / moments["m00"])
                    y = int(moments["m01"] / moments["m00"])
                    return True, x , y
                    
                except ZeroDivisionError:
                    return False, 0, 
                   
    return False, 0, 0    
    
    
 

def kurs2():   
    while True:
        
        image = auv.get_image_bottom()
        found, x,y = find_rect(image,0,180)
        time.sleep(0.02)
        keep_depth(1)
        stab_on_form(image,15,30)
        
        auv.set_motor_power(0,5)
        auv.set_motor_power(1,-5)
       
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)
    
        if (158 < x < 162) and y < 120:
            kurs = auv.get_yaw()
            break 
       
    now = time.time()
    
    
    now = time.time()
    while True:
        
        image = auv.get_image_bottom()
        stab_on_form(image,15,30)
        
        time.sleep(0.02)  
        keep_yaw(kurs,0)
        keep_depth(0.5)
                
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)
        
        if time.time() - now > 10:
            break


kurs2()



now = time.time()
while True:
    auv.set_motor_power(1,20)
    auv.set_motor_power(0,20)
    time.sleep(0.02)
    if time.time() - now > 3:
        break
    
   

now = time.time()    

while True:
    image = auv.get_image_bottom()
    found, x,y = form_on_start(image,120,135)
    time.sleep(0.02)

    
     
    if found == 'circle':        
        stab_on_form(image,120,135)
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)
        if time.time() - now > 15: 
            break    


time.sleep(1)
auv.open_grabber()



def color_rect():    
    color = 0
    time.sleep(2)
    while color < 170:
        image = auv.get_image_bottom()
        found, x, y = find_rect(image,color,color + 10)
        color += 1
        time.sleep(0.01)
        if found:
            return color
            break


color = color_rect()
print(color)


def find_pointer(image,min,max):
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv_min = (min,50,50)
    hsv_max = (max,255,255)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ =cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 50:
                continue
            
            
            
            
            if abs(area) < 800:
                moments = cv.moments(c)
                try:
                    x = int(moments["m10"] / moments["m00"])
                    y = int(moments["m01"] / moments["m00"])
                    return "circle", x , y
                    
                except ZeroDivisionError:
                    return False, 0, 
                
                
                        
                   
                   
            
    return False, 0, 0
    





def kurs3():
    while True:
        
        image = auv.get_image_bottom()
        found, x,y = form_on_start(image,120,135)
        time.sleep(0.02)
        
       
        
        found, x,y = find_pointer(image,color,color + 10)
        stab_on_form(image,120,135)
       
        
        auv.set_motor_power(0,-5)
        auv.set_motor_power(1,5)
        keep_depth(1)
       
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)
    
        if (158 < x < 162) and y < 120:
            kurs = auv.get_yaw()
            break 
       
    now = time.time()
    while True:
        
        image = auv.get_image_bottom()
        stab_on_form(image,120,135)
        
        time.sleep(0.02)  
        keep_yaw(kurs,0)
        keep_depth(1)
                
        cv.circle(image, (x,y), 4, (0,255,0), 4)
        cv.imshow("window", image)
        cv.waitKey(3)
        
        if time.time() - now > 10:
            break


kurs3()

def go_surface():
    
    now = time.time()
    while True:
        keep_depth(2)
        auv.set_motor_power(1,50)
        auv.set_motor_power(0,50)
        time.sleep(0.02)
        if time.time() - now > 5:
            break
    
    now = time.time()
    while True:
        image = auv.get_image_bottom()
        found,x,y = form_on_start(image,15,30)
        time.sleep(0.02)
        keep_depth(2)
        if found == 'circle' :
            stab_on_form(image,15,30)
            
            cv.circle(image, (x,y), 4, (0,255,0), 4)
            cv.imshow("window", image)
            cv.waitKey(3)            
            
            if time.time() - now > 15:
                break        
    
    now = time.time()   
    while True:
        time.sleep(0.02)
        keep_depth(0)
        if time.time() - now > 10:
            break         
        
        
go_surface()     
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        