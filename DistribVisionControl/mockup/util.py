import math


def angle_wrap180(a):
    return math.atan2(math.sin(a), math.cos(a))


def angle_wrap(a):
    return vec_dir((math.cos(a), math.sin(a)))


def vec_angle(a, b):
    return math.atan2(a[0] * b[1] - b[0] * a[1], a[0] * b[0] + a[1] * b[1])


def vec_from_angle(angle):
    return [math.cos(angle), math.sin(angle)]


def vec_dir(v):
    o = math.atan2(v[1], v[0])
    o += (math.pi * 2 if o < 0 else 0)
    return o
