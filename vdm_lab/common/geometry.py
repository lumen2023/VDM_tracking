import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def pi_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
