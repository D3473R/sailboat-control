RUDDER_SERVO_MIN = 220
RUDDER_SERVO_MAX = 450
RUDDER_SERVO_MIDDLE = RUDDER_SERVO_MIN + (RUDDER_SERVO_MAX - RUDDER_SERVO_MIN) / 2

SAIL_SERVO_MIN = 250
SAIL_SERVO_MAX = 500

def get_sail_angle(awa):
    """Get the sail angle from an apparent wind angle."""

    assert isinstance(awa, float) or isinstance(awa, int)
    if awa <= 180:
        sail_angle = awa / 2
    else:
        sail_angle = (360 - awa) / 2
    return round(sail_angle, 2)


def map_rudder_servo(heading_delta):
    """Maps the heading delta to a rudder servo value."""

    assert isinstance(heading_delta, float) or isinstance(heading_delta, int)

    if heading_delta <= 90:
        rudder_value = translate(heading_delta, 0, 90, RUDDER_SERVO_MIDDLE, RUDDER_SERVO_MAX)
    elif heading_delta <= 180:
        rudder_value = RUDDER_SERVO_MAX
    elif heading_delta <= 270:
        rudder_value = RUDDER_SERVO_MIN
    else:
        rudder_value = translate(heading_delta, 270, 360, RUDDER_SERVO_MIN, RUDDER_SERVO_MIDDLE)

    return round(rudder_value)


def map_sail_servo(sail_angle):
    """Maps the sail angle to a sail servo value."""

    assert isinstance(sail_angle, float) or isinstance(sail_angle, int)

    sail_value = translate(sail_angle, 0, 90, SAIL_SERVO_MAX, SAIL_SERVO_MIN)
    return round(sail_value)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)
