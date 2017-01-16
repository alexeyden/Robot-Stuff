import heapq
import numpy as np

from shapely.geometry import Polygon
from mockup import util


class Planner:
    d_xy = 1.2

    class Node:
        def __init__(self, f, g, prev, state):
            self.f = f; self.g = g; self.prev = prev
            self.state = state

        def __lt__(self, other):
            return self.f < other.f

    def __init__(self, obstacles, start):
        self.start = start
        self.obstacles = obstacles
        self.use_alpha = False
        self.report_on = 100

    def plan(self, goal):
        open_ = []
        closed = set()

        gx, gy, ga = goal

        node = self.Node(self._h(self.start, goal), 0, None, self.start)

        heapq.heappush(open_, node)

        i = 0

        while len(open_) > 0:
            cur = heapq.heappop(open_)  # type: Planner.Node
            cx,cy,ca = cur.state

            state_round = (round(cx, 2), round(cy, 2), round(ca, 2))

            if np.linalg.norm([gx - cx, gy - cy]) < 0.5:
                print(util.angle_wrap180(ca - ga) * 180/np.pi)

            if np.linalg.norm([gx - cx, gy - cy]) < 0.5 and (not self.use_alpha or abs(util.angle_wrap180(ca - ga)) < np.pi/6.0):
                yield self._build_path(cur)
                break

            for succ in self._succ(cur, goal):
                heapq.heappush(open_, succ)

            closed.add(state_round)

            i += 1
            if i % self.report_on == 0:
                status = 'open = {}\tD = {}\tf = {}\txya = {:.3f},{:.3f},{:3f}'.format(
                    len(open_),
                    np.linalg.norm([cx-gx, cy-gy]),
                    cur.f,
                    *state_round)
                print(status)
                yield self._build_path(cur)

        return None

    @staticmethod
    def _build_path(dst : Node):
        path = []
        while dst is not None:
            path.insert(0, dst.state)
            dst = dst.prev

        return path

    def _succ(self, node: Node, goal):
        cx,cy,ca = node.state

        for d_a in [-np.pi/6, 0.0, np.pi/6]:
            next_pos = np.array(util.vec_from_angle(ca + d_a)) * self.d_xy + [cx, cy]

            if any([p.intersects(self._robot_poly(*next_pos, ca + d_a)) for p in self.obstacles]):
                continue

            if next_pos[0] > 30 or next_pos[1] > 30 or next_pos[0] < -10 or next_pos[1] < -10:
                continue

            next_state = (*next_pos, util.angle_wrap180(ca + d_a))
            h = self._h(next_state, goal)

            g = node.g + np.linalg.norm([next_pos[0] - cx, next_pos[1] - cy])

            yield Planner.Node(h + g, g, node, next_state)

    def _h(self, src, dst):
        sx, sy, sa = src
        dx, dy, da = dst

        if self.use_alpha:
            a = (util.angle_wrap(da - sa))/np.pi
            return np.linalg.norm([dx - sx, dy - sy, a * 10])
        else:
            return np.linalg.norm([dx - sx, dy - sy])

    @staticmethod
    def _robot_poly(r_x, r_y, r_alpha):
        width = 0.6
        height = 1.0
        k = 1

        c = np.array([r_x, r_y])
        o = r_alpha

        m = np.array([[np.cos(o), -np.sin(o)], [np.sin(o), np.cos(o)]])

        points = [
            c*k + np.dot(m, [-width/2 * k, height/2 * k]),
            c*k + np.dot(m, [+width/2 * k, height/2 * k]),
            c*k + np.dot(m, [+width/2 * k, -height/2 * k]),
            c*k + np.dot(m, [-width/2 * k, -height/2 * k]),
        ]

        return Polygon(points)
