# pyright: reportMissingImports=false

from spike import Button, ColorSensor, LightMatrix, MotionSensor, Motor, MotorPair, PrimeHub
import math
import time

# Hub
hub = PrimeHub()

# Motors
lm = Motor('B')
rm = Motor('A')

# Motor polarity
lm_polarity = -1
rm_polarity = 1

lm.set_stop_action('brake')
rm.set_stop_action('brake')

pair = MotorPair('B','A')
pair.set_motor_rotation(17.6 * math.pi, 'cm') # Sizes: Small (5.6cm) Large (17.6cm)


lmm = Motor('C')
rmm = Motor('D')

# Sensors
lls = ColorSensor('E')
rls = ColorSensor('F')

gyro = MotionSensor()

# Constants
minRef = 34
maxRef = 99

def cal_spd(speed, motor):

    # +ve speed: Forward
    # -ve speed: Backward

    if motor == 'rm': return(rm_polarity*speed)
    elif motor == 'lm': return(lm_polarity*speed)
    else: raise ValueError('Unspecified')

def pauseloop():
    hub.status_light.on('red')
    while not hub.left_button.is_pressed() and not hub.right_button.is_pressed():
        pass
    hub.status_light.on('green')

def calibrate_ls():

    minRef = 0
    maxRef = 100
    dataleft = []
    dataright = []

    # State
    state = 1
    hub.light_matrix.write(str(state))

    while True:
        if hub.left_button.is_pressed() and state < 3:
            time.sleep_ms(500)
            state += 1
            hub.light_matrix.write(str(state))

        elif hub.right_button.is_pressed() and state > 1:
            state -= 1
            time.sleep_ms(500)
            hub.light_matrix.write(str(state))

        dataleft.append(lls.get_reflected_light())
        dataright.append(rls.get_reflected_light())
        minRef = (min(dataleft)+min(dataright))/2
        maxRef = (max(dataleft)+max(dataright))/2

        if state == 1:
            print('l.ref: {lref}, r.ref: {rref}'.format(lref = lls.get_reflected_light(),
                                                        rref = rls.get_reflected_light()))
        elif state == 2:
            print('l.ref: {lref}, r.ref: {rref}'.format(lref = (100 * (lls.get_reflected_light() - minRef ) / ( maxRef - minRef)),
                                                        rref = (100 * (rls.get_reflected_light() - minRef ) / ( maxRef - minRef))))

        elif state == 3:
            print('minref: {minref}, maxref: {maxref}'.format(maxref = maxRef,
                                                            minref = minRef))





def reset_m():
    rm.set_degrees_counted(0)
    lm.set_degrees_counted(0)

def line_follow(speed,
                deg,
                target = None,
                right_edge = True,
                right_ls = True,
                forward = True,
                scale_ref = False,
                track_right = True):

    if target is None: raise ValueError('Missing Light Value')

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
        if scale_ref:
            reflected_light_intensity = (100 * (ls.get_reflected_light() - minRef ) / ( maxRef - minRef ))
        else:
            reflected_light_intensity = ls.get_reflected_light()
        error = target - reflected_light_intensity

        if error == 0:
            integral = 0
        else:
            integral = integral + error

        derivative = error - last_error
        last_error = error

        steering = round((lkp * error) + (lki * integral) + (lkd * derivative))
        steering *= polarity

        pair.start(steering = steering, speed = speed)

        print('ref:{ref}, error:{err}, steering:{steer} '.format(ref = reflected_light_intensity, steer = steering, err = error))
    pair.stop()

def dline_follow(speed,
                deg,
                right_edge = True,
                forward = True,
                scale_ref = False,
                track_right = True):

    if track_right: tracked_motor = rm
    else: tracked_motor = lm
    if not forward:
        speed *= -1

    last_error = error = integral = 0.0
    derivative = 0.0

    reset_m()

    while not abs(tracked_motor.get_degrees_counted()) > deg:
        if scale_ref:
            error = (100 * (lls.get_reflected_light() - minRef ) / ( maxRef - minRef )) - (100 * (rls.get_reflected_light() - minRef ) / ( maxRef - minRef ))
        else:
            error = lls.get_reflected_light() - rls.get_reflected_light()
    
        if error == 0:
            integral = 0
        else:
            integral = integral + error

        derivative = error - last_error
        last_error = error

        steering = round((lkp * error) + (lki * integral) + (lkd * derivative))

        pair.start(steering = steering, speed = speed)

        print('error:{err}, steering:{steer} '.format(steer = steering, err = error))
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
    # Forward: -ve
    # Backward: +ve

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
                    rm.start(speed = cal_spd(start_spd, 'rm'))
                    # print(rls.get_reflected_light())
                rm.stop()
                break

            else:
                if rls.get_reflected_light() <= rls_target_light_intensity:
                    rm.stop()
                    while not lls.get_reflected_light() <= lls_target_light_intensity:
                        lm.start(speed = cal_spd(start_spd, 'lm'))
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
                    rm.start(speed = cal_spd(start_spd, 'rm'))
                    # print(rls.get_reflected_light())
                rm.stop()
                break

            else:
                if rls.get_reflected_light() >= rls_target_light_intensity:
                    rm.stop()
                    while not lls.get_reflected_light() >= lls_target_light_intensity:
                        lm.start(speed = cal_spd(start_spd, 'lm'))
                        # print(lls.get_reflected_light())
                    lm.stop()
                    break
                else:
                    pass

    # Fine Adjustment
    start = time.ticks_ms()
    end = time.ticks_ms()


    while not (end - start) > maxduration*1000:

        if lls.get_reflected_light() < lls_target_light_intensity:
            lm.start(speed = cal_spd(forward_spd, 'lm'))
        else:
            if lls.get_reflected_light() > lls_target_light_intensity:
                lm.start(speed = cal_spd(backward_spd, 'lm'))

            else:
                lm.stop()

        if rls.get_reflected_light() < rls_target_light_intensity:
            rm.start(speed = cal_spd(forward_spd, 'rm'))
        else:
            if rls.get_reflected_light() > rls_target_light_intensity:
                rm.start(speed = cal_spd(backward_spd, 'rm'))

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

def gyro_turn(target, marginoferror, speed, sec, timeout = True):
    start = time.ticks_ms()
    end = time.ticks_ms()

    while not end-start > sec*1000:
        o_yaw = gyro.get_yaw_angle()
        if target - o_yaw < 0:
            pair.steering(speed = speed, steering = -100)
        elif target - o_yaw > 0:
            pair.steering(speed = speed, steering = 100)
        elif abs(target - o_yaw) <= marginoferror:
            break
        
        if timeout: end = time.ticks_ms()
    
    lm.stop()
    rm.stop()


# Global Constants

## Run 1

# Run 1 Constants
gkp = 3
gki = 0
gkd = 0.5

gtkp = 1

lkp = 0.45
lki = 0
lkd = 0.45

# Run 1 starts
gyro.reset_yaw_angle()

# line_align(10,1.5,2, target = 75, correction_spd = 10, timeout = True)

# line_follow(15, 100000, target = 62, right_ls = False, right_edge = False , scale_ref = True)

# gyro_straight(20, 10000, target = 0)

# calibrate_ls()


raise SystemExit

## Run 2