from pyDatalog import pyDatalog as pd
import math

class Match(pd.Mixin):
    def __init__(self, name, prob, cx, cy, w, h):
        super().__init__()
        self.name = name  # класс найденной части
        self.prob = prob  # вероятнсть присутствия
        self.cx = cx  # координата X центра области
        self.cy = cy  # координата Y центра области
        self.w = w  # ширина области
        self.h = h  # выстоа области

    def __repr__(self):
        return f'Match: {self.name} cx={self.cx} cy={self.cy} w={self.w} h={self.h}'


pd.create_terms('distance, X, Y, D, math, min_bound, max_bound')
distance(X, Y, D) <= (math.sqrt((Match.cx[X] - Match.cx[Y]) ** 2 + (Match.cy[X] - Match.cy[Y]) ** 2) == D)

# #####################################################################################################################

pd.create_terms('wheels_1, wheels_2, tower_1, tower_2, tank, X, Y, B, Z, D, H, I')

wheels_1(X) <= (
    (Match.name[X] == "wheel") &
    (Match.h[X] < 1.5))

wheels_2(X, Y, D) <= (
    (Match.name[X] == "wheel") &
    distance(X, Y, D) &
    (D <= 1))

tower_1(X) <= (
    (Match.name[X] == "tower") &
    (Match.w[X] <= 4) &
    (Match.h[X] <= 2.5))

tower_2(X, Z) <= (
    (Match.name[X] == "wheel") &
    (Match.name[Z] == "tower") &
    ((Match.cx[X] if X.is_const() else min([t.cx for t in X.data])) < Match.cx[Z]) &
    (Match.cx[Z] < (Match.cx[X] if X.is_const() else max([t.cx for t in X.data]))))

tank(X, Y, Z, D) <= \
    wheels_1(X) & \
    wheels_1(Y) & \
    tower_1(Z) & \
    wheels_2(X, Y, D) & \
    tower_2(X, Z)


def is_tank():
    return len(tank(X, Y, Z, D)) > 0


def is_vehicle():
    return is_tank()

# ###################################################################################################################

pd.create_terms('X, Y, D, Z, I, M')

ex = 3
if ex == 1:
    M = [
        # tower: p=0.7806177139282227 c=485.0,165.0 s=570,330
        Match("tower", 0.78, 458/150, 165/150, 570/150, 330/150),

        # wheel: p=0.8195627927780151 c=495.0,330.0 s=390,180
        Match("wheel", 0.78, 495/150, 330/150, 390/150, 180/150)
    ]
elif ex == 2:
    '''
    wheel: p=0.82 c=2.75,3.05 s=1.3,1.3
    wheel: p=0.80 c=4.1,3.2 s=1.6,1.0
    wheel: p=0.84 c=7.1,3.2 s=1.0,1.0

    tower: p=0.79 c=1.9,1.25 s=3.8,2.5
    tower: p=0.79 c=4.9,1.4 s=3.2,2.8
    tower: p=0.79 c=7.6,0.95 s=2.0,1.9
    tower: p=0.79 c=3.7,1.7 s=2.0,1.0
    '''

    M = [
        Match("wheel", 0.82, 2.75, 3.05, 1.3, 1.3),
        Match("wheel", 0.80, 4.10, 3.20, 1.6, 1.0),
        Match("wheel", 0.84, 7.10, 3.20, 1.0, 1.0),
        Match("tower", 0.79, 1.9, 1.25, 3.8, 2.5),
        Match("tower", 0.79, 4.9, 1.4 , 3.2, 2.8),
        Match("tower", 0.79, 7.6, 0.95, 2.0, 1.9),
        Match("tower", 0.79, 3.7, 1.7 , 2.0, 1.0)
    ]

elif ex == 3:
    '''
    wheel: p=0.92 c=2.45,3.5 s=1.3,1.0
    wheel: p=0.94 c=5.75,3.5 s=1.3,1.0
    tower: p=0.80 c=2.2,1.0 s=2.0,2.0
    '''
    M = [
        Match("wheel", 0.92, 2.45, 3.5, 1.3, 1.0),
        Match("wheel", 0.94, 5.75, 3.5, 1.3, 1.0),
        Match("tower", 0.80, 2.2,  1.0, 2.0, 2.0),
    ]

print(tower_1(Z) & tower_2(X, Z))
