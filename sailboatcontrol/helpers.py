RUDDER_SERVO_MIN = 220
RUDDER_SERVO_MAX = 450
RUDDER_SERVO_MIDDLE = RUDDER_SERVO_MIN + (RUDDER_SERVO_MAX - RUDDER_SERVO_MIN) / 2
RUDDER_SERVO_RANGE = RUDDER_SERVO_MAX - RUDDER_SERVO_MIN
RUDDER_SERVO_RANGE_HALF = RUDDER_SERVO_RANGE / 2

SAIL_SERVO_MIN = 250
SAIL_SERVO_MAX = 500
SAIL_SERVO_MIDDLE = SAIL_SERVO_MIN + (SAIL_SERVO_MAX - SAIL_SERVO_MIN) / 2
SAIL_SERVO_RANGE = SAIL_SERVO_MAX - SAIL_SERVO_MIN
SAIL_SERVO_RANGE_HALF = SAIL_SERVO_RANGE / 2

HEADING_ANGLE_RANGE = 360
HEADING_ANGLE_RANGE_HALF = HEADING_ANGLE_RANGE / 2
HEADING_ANGLE_RANGE_QUARTER = HEADING_ANGLE_RANGE_HALF / 2


def get_sail_angle(awa):
    """Get the sail angle from an apparent wind angle."""

    assert isinstance(awa, float) or isinstance(awa, int)
    if awa <= HEADING_ANGLE_RANGE_HALF:
        sail_angle = awa * HEADING_ANGLE_RANGE_QUARTER / HEADING_ANGLE_RANGE_HALF
    else:
        sail_angle = (HEADING_ANGLE_RANGE - awa) \
                     * HEADING_ANGLE_RANGE_QUARTER / HEADING_ANGLE_RANGE_HALF
    return round(sail_angle, 2)


def map_rudder_servo(heading_delta):
    """Maps the heading delta to a rudder servo value."""

    assert isinstance(heading_delta, float) or isinstance(heading_delta, int)

    if heading_delta <= HEADING_ANGLE_RANGE_HALF:
        rudder_value = abs(heading_delta * RUDDER_SERVO_RANGE_HALF / HEADING_ANGLE_RANGE_HALF - RUDDER_SERVO_MIDDLE)
    else:
        rudder_value = abs((heading_delta - HEADING_ANGLE_RANGE_HALF)
                           * RUDDER_SERVO_RANGE_HALF / HEADING_ANGLE_RANGE_HALF - RUDDER_SERVO_MAX)
    return round(rudder_value)


def map_sail_servo(sail_angle):
    """Maps the sail angle to a sail servo value."""

    assert isinstance(sail_angle, float) or isinstance(sail_angle, int)

    if sail_angle <= HEADING_ANGLE_RANGE_HALF:
        sail_value = abs(sail_angle * SAIL_SERVO_RANGE_HALF / HEADING_ANGLE_RANGE_HALF - SAIL_SERVO_MIDDLE)
    else:
        sail_value = abs((sail_angle - HEADING_ANGLE_RANGE_HALF)
                         * SAIL_SERVO_RANGE_HALF / HEADING_ANGLE_RANGE_HALF - SAIL_SERVO_MAX)
    return round(sail_value)
