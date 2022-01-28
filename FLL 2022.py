# pyright: reportMissingImports=false
from spike import Button, ColorSensor, LightMatrix, MotionSensor, Motor, MotorPair, PrimeHub
import math
import time

# Hub
hub = PrimeHub()
hub.status_light.off()

# Motors
lm = Motor('A')
rm = Motor('B')

lm.set_stop_action('hold')
rm.set_stop_action('hold')
pair = MotorPair('A','B')
pair.set_motor_rotation(17.6 * math.pi, 'cm') # Sizes: Small (5.6cm) Large (17.6cm)

lmm = Motor('C')
rmm = Motor('D')

# Sensors
lls = ColorSensor('E')
rls = ColorSensor('F')

gyro = MotionSensor()

# Constants
minRef = 0
maxRef = 100


def pauseloop():
    hub.status_light.on('red')
    while not hub.left_button.is_pressed() and not hub.right_button.is_pressed():
        pass
    hub.status_light.on('green')

def reset_m():
    rm.set_degrees_counted(0)
    lm.set_degrees_counted(0)

def line_follow(speed,
                deg,
                target = None,
                right_edge = True,
                right_ls = True,
                forward = True,
                scaling = True,
                track_right = True):

    if target is None: raise ValueError('Missing Light Value')
    if scaling: target_light_intensity = (100 * (target - minRef ) / ( maxRef - minRef))

    if right_ls: ls = rls
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

    reset_m()

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

        steering = round((lkp * error) + (lki * integral) + (lkd * derivative))

        pair.start(steering = steering, speed = speed)

        print('ref:{ref}, error:{err}, steering:{steer} '.format(ref = reflected_light_intensity, steer = steering, err = error))
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
    start = time.ticks_ms()
    end = time.ticks_ms()


    while not (end - start) > maxduration/1000:

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
            end = time.ticks_ms()
        else:
            end = start
    
    rm.stop()
    lm.stop()

    # print(lls.get_reflected_light())
    # print(rls.get_reflected_light())

def gyro_straight(speed,
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

    reset_m()

    while not abs(tracked_motor.get_degrees_counted()) > deg:
        error = target - gyro.get_yaw_angle()

        if error == 0:
            integral = 0
        else:
            integral = integral + error

        derivative = error - last_error
        last_error = error

        steering = (gkp * error) + (gki * integral) + (gkd * derivative)

        pair.start(steering = round(steering), speed = speed)

    pair.stop()

def gyro_turn(speed, target):
    while not abs(gyro.get_yaw_angle()) > abs(target):
        if target > 0: pair.start(steering = 100, speed = speed)
        else: pair.start(steering = -100, speed = speed)


# Global Constants

## Run 1

# Run 1 Constants
gkp = 1
gki = 0
gkd = 0

lkp = 0
lki = 0 
lkd = 0

# Run 1 starts
gyro.reset_yaw_angle()

# line_align(10,1.5,2, target = 65, correction_spd = 20, timeout = True)

# line_follow(30, 1000000, target = 65, right_ls = True)


raise SystemExit

## Run 2

