# pyright: reportMissingImports=false
from spike import PrimeHub, LightMatrix, Button, ColorSensor, Motor, MotorPair, MotionSensor
import math
import time

class Navigation:

    def __init__(self, minRef, maxRef):
        
        # Motors
        self.lm = Motor('B')
        self.rm = Motor('C')

        self.pair = MotorPair('B','C')
        self.pair.set_motor_rotation(17.6 * math.pi, 'cm') # Sizes: Small (5.6cm) Large (17.6cm)

        
        # Sensors
        self.lls = ColorSensor('E')
        self.rls = ColorSensor('F')

        self.gyro = MotionSensor('A')
 
        # Constants
        self.minRef = minRef
        self.maxRef = maxRef
        
        
    def line_follow(self,
                    kp,
                    ki,
                    kd,
                    speed,
                    target = None,
                    right_edge = True,
                    rls = True,
                    forward = True,
                    scaling = True):    

        if target is None: raise ValueError('Missing Light Value')
        if scaling: target_light_intensity = (100 * (target - self.minRef ) / ( self.maxRef - self.minRef))          
        
        if rls: ls = self.rls
        else: ls = self.lls
        if right_edge: polarity = 1
        else: polarity = -1 
        
        if not forward:
            speed *= -1
            polarity *= 1

        last_error = error = integral = 0.0
        derivative = 0.0

        while True:
            if scaling:
                reflected_light_intensity = (100 * (ls.get_reflected_light() - self.minRef ) / ( self.maxRef - self.minRef))
            else:
                reflected_light_intensity = ls.get_reflected_light()
            error = target_light_intensity - reflected_light_intensity

            if error == 0:
                integral = 0
            else:
                integral = integral + error
            
            derivative = error - last_error
            last_error = error

            steering = (kp * error) + (ki * integral) + (kd * derivative)

            self.pair.start(steering = steering, speed = speed)

            # print('ref:{ref}, error:{err}, steering:{steer} '.format(ref = reflected_light_intensity, steer = steering, err = error))
        self.pair.stop()

    def line_align(self,
            start_spd,
            maxduration,
            marginoferror,
            target = None,
            black_white = True,
            correction_spd = None,
            beforeline = True,
            timeout = True):

        if beforeline: polarity = 1
        else: polarity = -1
        

        if target is None: raise ValueError('Missing Light Value')
        else:
            rls_target_light_intensity = target
            lls_target_light_intensity = target
                
        
        rls_target_black = rls_target_light_intensity - marginoferror
        rls_target_white = rls_target_light_intensity + marginoferror
        lls_target_black = lls_target_light_intensity - marginoferror
        lls_target_white = lls_target_light_intensity + marginoferror

        if correction_spd is None: correction_spd = start_spd
        
        start_spd = start_spd*polarity
        
        forward_spd = -correction_spd*polarity
        backward_spd = correction_spd*polarity

        # Coarse Adjustment

        while True:
            self.pair.start(steering = 0, speed = start_spd)

            if black_white:

                # Aligning towards black Line 

                if self.lls.get_reflected_light() <= lls_target_light_intensity:
                    self.lm.stop()
                    while not self.rls.get_reflected_light() <= rls_target_light_intensity:
                        self.rm.start(speed = start_spd)
                        # print(self.rls.get_reflected_light())
                    self.rm.stop()
                    break
            
                else:
                    if self.rls.get_reflected_light() <= rls_target_light_intensity:
                        self.rm.stop()
                        while not self.lls.get_reflected_light() <= lls_target_light_intensity:
                            self.lm.start(speed = start_spd)
                            # print(self.lls.get_reflected_light())
                        self.lm.stop()
                        break
                    else:
                        pass
            
            else:

                # Aligning towards White Line

                if self.lls.get_reflected_light() >= lls_target_light_intensity:
                    self.lm.stop()
                    while not self.rls.get_reflected_light() >= rls_target_light_intensity:
                        self.rm.start(speed = start_spd)
                        # print(self.rls.get_reflected_light())
                    self.rm.stop()
                    break
            
                else:
                    if self.rls.get_reflected_light() >= rls_target_light_intensity:
                        self.rm.stop()
                        while not self.lls.get_reflected_light() >= lls_target_light_intensity:
                            self.lm.start(speed = start_spd)
                            # print(self.lls.get_reflected_light())
                        self.lm.stop()
                        break
                    else:
                        pass

        # Fine Adjustment
        start = time.time()
        end = time.time()


        while not (end - start) > maxduration:

            if self.lls.get_reflected_light() < lls_target_light_intensity:
                self.lm.start(speed = forward_spd)
            else:
                if self.lls.get_reflected_light() > lls_target_light_intensity:
                    self.lm.start(speed = backward_spd)
                
                else:
                    self.lm.stop()
            
            if self.rls.get_reflected_light() < rls_target_light_intensity:
                self.rm.start(speed = forward_spd)
            else:
                if self.rls.get_reflected_light() > rls_target_light_intensity:
                    self.rm.start(speed = backward_spd)
                
                else:
                    self.rm.stop()
            
            if lls_target_black < self.lls.get_reflected_light() < lls_target_white and rls_target_black < self.rls.get_reflected_light() < rls_target_white:
                break
            
            if timeout:
                end = time.time()
            else:
                end = start
        
        self.rm.stop()
        self.lm.stop()

        # print(self.lls.get_reflected_light())
        # print(self.rls.get_reflected_light())

    def gyro_straight(self,
                    kp,
                    ki,
                    kd,
                    speed,
                    target = None,
                    forward = True): 

        if target is None: raise ValueError('Missing Gyro Value')
        if not forward:
            speed *= -1
        last_error = error = integral = 0.0
        derivative = 0.0

        while True:
            error = target - self.gyro.get_yaw_angle()

            if error == 0:
                integral = 0
            else:
                integral = integral + error
            
            derivative = error - last_error
            last_error = error

            steering = (kp * error) + (ki * integral) + (kd * derivative)

            self.pair.start(steering = steering, speed = speed)
        
        self.pair.stop()
    
    def gyro_turn(self,
                target):
        
        while True:
            correction = target - self.gyro.get_yaw_angle()
            if abs(correction) > 20:
                speed = abs(correction)/2
            else:
                speed = 10
            if correction > 0:
                self.pair.start(steering = 100, speed = speed)
            elif correction < 0:
                self.pair.start(steering = -100, speed = speed)
            else:
                break


        
# Run 1 

nav = Navigation(0,100)

lm = Motor('B')
rm = Motor('C')

pair = MotorPair('B','C')
pair.set_motor_rotation(17.6 * math.pi, 'cm') # Sizes: Small (5.6cm) Large (17.6cm)


# Sensors
lls = ColorSensor('E')
rls = ColorSensor('F')

gyro = MotionSensor('A')





raise SystemExit