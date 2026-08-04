#!/usr/bin/env python
#
# Grove Library for I2C-Motor-Driver with Stepper Motor

from __future__ import division
from grove.i2c import Bus

__all__ = ["StepperMotor", "I2CStepperMotor"]

class StepperMotor(object):
    DC_SPEED_MAX         = 100
    _DIR_ANTI_CLKWISE    = 0
    _DIR_CLKWISE         = 1

    def __init__(self, arguments):
        # default Dirction
        self._dir = self._DIR_CLKWISE
        # Speed Varation Ratio, 1.0 is unapplicable
        # Most time it's reductor.
        self._var_ratio = arguments["var-ratio"]
        # Stride Angle
        self._stride_angle = arguments["stride-angle"]
        self._rpm_max      = arguments.get("rpm-max", 1.0)
        self._name         = arguments.get("name", "StepperMotor")

    @property
    def speed_max(self):
        return self._rpm_max

    @property
    def name(self):
        return self._name

    # To be derived
    # def _rotate(self, angle = None):
    #    return None

    def rotate(self, angle = None):
        "specify the stepper motor rotating angle,"
        " query the angle left if angle missed"
        "    angle  --- 1 cycle = 360 degrees, positive float or int acceptable"
        "    return angle left"
        return self._rotate(angle)

    # To be derived
    def _speed(self, rpm):
        pass

    def speed(self, rpm = None):
        "set or get stepper motor speed & direction"
        "    rpm    --- revolutions per minute, float or int acceptable"
        "               > 0 clockwise, < 0 anti-clockwise"
        "    return rpm value stored"
        if not rpm is None:
            self._rpm = rpm
            self._speed(rpm)
        return self._rpm

    # To be derived
    def _enable(self, en):
        pass

    def enable(self, en = None):
        "enable or disable the stepper motor"
        "    en    --- True for enable, False for disable"
        "    return disable or enable state stored"
        if not en is None:
            self._en = en
            self._enable(en)
        return self._en

    def _steps2angle(self, steps):
        angle = steps * self._stride_angle / self._var_ratio
        return angle

    def _angle2steps(self, angle):
        steps = angle * self._var_ratio / self._stride_angle
        return int(steps)



import time

class I2CStepperMotor(StepperMotor):
    __REG_GET_PID       = 0x00
    __REG_GET_VID       = 0x01
    __REG_GET_VER       = 0x02
    __REG_STP_EN        = 0x1A
    __REG_STP_DIS       = 0x1B
    __REG_STP_RUN       = 0x1C
    __REG_STP_INTERVAL  = 0x1D
    __REG_SEQ_LEN       = 0x20
    __REG_SEQ_XET       = 0x21
    __REG_SET_SPEED     = 0x82
    __REG_SET_FREQ      = 0x84
    __REG_SET_A         = 0xA1
    __REG_SET_B         = 0xA5
    __REG_SET_DIR       = 0xAA

    def __init__(self, arguments, address = 0x0F):
        super(I2CStepperMotor, self).__init__(arguments)
        self._addr = address
        self._bus = Bus()
        self._ang_left = 0
        self._load_seq(arguments["sequences"])

    def __del__(self):
        self.set_speed(0, 0)
        pass

    #Maps speed from 0-100 to 0-255
    def _map_vals(self,value, leftMin, leftMax, rightMin, rightMax):
        #http://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return int(rightMin + (valueScaled * rightSpan))
        
    #Set motor speed
    def set_speed(self, speed1 = 0, speed2 = 0):
        s1 = self._map_vals(speed1, 0, 100, 0, 255)
        s2 = self._map_vals(speed2, 0, 100, 0, 255)
        self._bus.write_i2c_block_data(self._addr, self.__REG_SET_SPEED, [s1, s2])
        time.sleep(.02)
    
    #Set motor direction
    def set_dir(self, clock_wise1 = True, clock_wise2 = True):
        dir1 = 0b10 if clock_wise1 else 0b01
        dir2 = 0b10 if clock_wise2 else 0b01
        dir = (dir2 << 2) | dir1
        self._bus.write_i2c_block_data(self._addr, self.__REG_SET_DIR, [dir, 0])
        time.sleep(.02)

    def _load_seq(self, seq):
        length = len(seq)
        self._bus.write_word_data(self._addr, self.__REG_SEQ_LEN, length)
        for i in range(len(seq)):
            self._bus.write_word_data(self._addr, self.__REG_SEQ_XET, seq[i])

    def _enable(self, en):
        cmd = self.__REG_STP_EN if en else self.__REG_STP_DIS
        if en:
            self.set_speed(self.DC_SPEED_MAX, self.DC_SPEED_MAX)
        else:
            self.set_speed(0, 0)
        self._bus.write_i2c_block_data(self._addr, cmd, [self._dir, 0])
        time.sleep(0.001)

    def _speed(self, rpm):
        self._dir = self._DIR_CLKWISE if rpm >= 0 else self._DIR_ANTI_CLKWISE
        # absolute angle per second
        aps = abs(rpm) * 360.0 / 60.0
        # steps per second, include reductor ratio.
        sps = self._angle2steps(aps)
        period = int(1000000 / sps) # us
        period = period // 10       # STP_INTERVAL, 10 us
        # print("period = %d us" % (period * 10))
        self._bus.write_word_data(self._addr, self.__REG_STP_INTERVAL, period)
        time.sleep(0.001)

    def _rotate(self, angle = None):
        if not angle is None:
            angle = abs(angle)
            self._ang_left = angle
            steps = self._angle2steps(angle)
            # print("steps set = {}".format(steps))
            self._bus.write_word_data(self._addr, self.__REG_STP_RUN, steps)
            time.sleep(0.001)
            return angle
        while True:
            # reading interface unstable when working
            try:
                steps = self._bus.read_word_data(self._addr, self.__REG_STP_RUN)
                # print("steps left = {}".format(steps))
                ang_left = self._steps2angle(steps)
                if ang_left > self._ang_left:
                    time.sleep(0.01)
                    continue
                self._ang_left = ang_left
                return ang_left
            except IOError:
                continue


def main():
    print("Make sure I2C-Motor-Driver inserted")
    print("  in one I2C slot of Grove-Base-Hat")

    arguments_28BYJ48 = {
        'var-ratio'   : 64,
        'stride-angle': 5.625,
        'rpm-max'     : 12,
        'sequences'   :
            # clockwise
            [ 0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001 ]
            # [ 0b1110, 0b1101, 0b1011, 0b0111 ]
            # anti-clockwise
            # [ 0b1001, 0b1000, 0b1100, 0b0100, 0b0110, 0b0010, 0b0011, 0b0001 ]
    }

    # motor = I2CStepperMotor(arguments_YH42BYGH40)
    motor = I2CStepperMotor(arguments_28BYJ48)
    ANGLE = 360 # rotate 360 degrees = 1 cycle
    motor.rotate(ANGLE)
    SPEED = 12 # speed set to 12 rpm, clockwise
    # anti-clockwise use negative value (-12)
    motor.speed(SPEED)
    # enable the motor, begin to run
    motor.enable(True)

    seconds = ANGLE / 360 / SPEED * 60
    print("Motor rotate {} degrees, time = {:.2f}".format(ANGLE, time.time()))
    print("      with speed {} RPM".format(SPEED))
    print("      will be stop after {:4.2f} seconds".format(seconds))

    while True:
        left = motor.rotate()
        if left < 1e-5:
            break
        print("Angle left {:6.2f} degrees, time = {:.2f}".format(left, time.time()))
        time.sleep(1.0)

    print("Motor run ended, time = {:.2f} !".format(time.time()))

if __name__ == '__main__':
    main()

