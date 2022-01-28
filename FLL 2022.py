# pyright: reportMissingImports=false

from spike import Button, ColorSensor, LightMatrix, MotionSensor, Motor, MotorPair, PrimeHub
import math
import time

# Motors
lm = Motor('B')
rm = Motor('C')

pair = MotorPair('B','C')
pair.set_motor_rotation(17.6 * math.pi, 'cm') # Sizes: Small (5.6cm) Large (17.6cm)


# Sensors
lls = ColorSensor('E')
rls = ColorSensor('F')

gyro = MotionSensor('A')

# Constants
minRef = 0
maxRef = 100
    
    
def line_follow(kp,
                ki,
                kd,
                speed,
                deg,
                target = None,
                right_edge = True,
                rls = True,
                forward = True,
                scaling = True,
                track_right = True):    

    if target is None: raise ValueError('Missing Light Value')
    if scaling: target_light_intensity = (100 * (target - minRef ) / ( maxRef - minRef))          
    
    if rls: ls = rls
    else: ls = lls
    if right_edge: polarity = 1
    else: polarity = -1 
    
    if track_right: tracked_motor = rm
    else: tracked_motor = lm
    if not forward:
        speed *= -1
        polarity *= 1

    last_error = error = integral = 0.0
    derivative = 0.0
    
    rm.set_degrees_counted(0)
    lm.set_degrees_counted(0)

    while not abs(tracked_motor.get_degrees_counted()) > deg:
        if scaling:
            reflected_light_intensity = (100 * (ls.get_reflected_light() - minRef ) / ( maxRef - minRef ))
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

        pair.start(steering = steering, speed = speed)

        # print('ref:{ref}, error:{err}, steering:{steer} '.format(ref = reflected_light_intensity, steer = steering, err = error))
    pair.stop()

def line_align(start_spd,
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
        pair.start(steering = 0, speed = start_spd)

        if black_white:

            # Aligning towards black Line 

            if lls.get_reflected_light() <= lls_target_light_intensity:
                lm.stop()
                while not rls.get_reflected_light() <= rls_target_light_intensity:
                    rm.start(speed = start_spd)
                    # print(rls.get_reflected_light())
                rm.stop()
                break
        
            else:
                if rls.get_reflected_light() <= rls_target_light_intensity:
                    rm.stop()
                    while not lls.get_reflected_light() <= lls_target_light_intensity:
                        lm.start(speed = start_spd)
                        # print(lls.get_reflected_light())
                    lm.stop()
                    break
                else:
                    pass
        
        else:

            # Aligning towards White Line

            if lls.get_reflected_light() >= lls_target_light_intensity:
                lm.stop()
                while not rls.get_reflected_light() >= rls_target_light_intensity:
                    rm.start(speed = start_spd)
                    # print(rls.get_reflected_light())
                rm.stop()
                break
        
            else:
                if rls.get_reflected_light() >= rls_target_light_intensity:
                    rm.stop()
                    while not lls.get_reflected_light() >= lls_target_light_intensity:
                        lm.start(speed = start_spd)
                        # print(lls.get_reflected_light())
                    lm.stop()
                    break
                else:
                    pass

    # Fine Adjustment
    start = time.time()
    end = time.time()


    while not (end - start) > maxduration:

        if lls.get_reflected_light() < lls_target_light_intensity:
            lm.start(speed = forward_spd)
        else:
            if lls.get_reflected_light() > lls_target_light_intensity:
                lm.start(speed = backward_spd)
            
            else:
                lm.stop()
        
        if rls.get_reflected_light() < rls_target_light_intensity:
            rm.start(speed = forward_spd)
        else:
            if rls.get_reflected_light() > rls_target_light_intensity:
                rm.start(speed = backward_spd)
            
            else:
                rm.stop()
        
        if lls_target_black < lls.get_reflected_light() < lls_target_white and rls_target_black < rls.get_reflected_light() < rls_target_white:
            break
        
        if timeout:
            end = time.time()
        else:
            end = start
    
    rm.stop()
    lm.stop()

    # print(lls.get_reflected_light())
    # print(rls.get_reflected_light())

def gyro_straight(kp,
                ki,
                kd,
                speed,
                deg,
                target = None,
                forward = True,
                track_right = True): 

    if target is None: raise ValueError('Missing Gyro Value')
    if not forward:
        speed *= -1
    last_error = error = integral = 0.0
    derivative = 0.0

    if track_right: tracked_motor = rm
    else: tracked_motor = lm

    rm.set_degrees_counted(0)
    lm.set_degrees_counted(0)
    
    while not abs(tracked_motor.get_degrees_counted()) > deg:
        error = target - gyro.get_yaw_angle()

        if error == 0:
            integral = 0
        else:
            integral = integral + error
        
        derivative = error - last_error
        last_error = error

        steering = (kp * error) + (ki * integral) + (kd * derivative)

        pair.start(steering = steering, speed = speed)
    
    pair.stop()

def gyro_turn(target):
    
    while True:
        correction = target - gyro.get_yaw_angle()
        if abs(correction) > 20:
            speed = abs(correction)/2
        else:
            speed = 10
        if correction > 0:
            pair.start(steering = 100, speed = speed)
        elif correction < 0:
            pair.start(steering = -100, speed = speed)
        else:
            break


        
# Run 1 

gyro.reset_yaw_angle()
gyro_turn(90)
gyro_straight(1,0,2,30,300,target = 90)


# Run 2


raise SystemExit
