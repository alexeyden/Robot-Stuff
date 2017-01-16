import numpy as np
from shapely.geometry import Polygon, polygon


class WorldObject:
    def __init__(self, points, name):
        self.points = polygon.orient(Polygon(points))
        self.stands = []
        self.name = name

        pts = self.points.exterior.coords
        for p1, p2 in zip(pts, pts[1:]):
            pa = np.array(p2)
            pb = np.array(p1)
            v = pb - pa
            n = np.array([-v[1], v[0]])
            s = (pa + pb) * 0.5 + n / np.linalg.norm(n)
            self.stands.append(s)


class World:
    def __init__(self):
        self.robots = []
        self.objects = []

    def update(self, dt):
        for r in self.robots:
            r.update(dt)
