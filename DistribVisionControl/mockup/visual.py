import tkinter as tk
import numpy as np
import multiprocessing as mp
import random
import colorsys

from .util import *
from .world import *
from .planner import *
from .robot import *
from .dispatch import ActionDispatcher


class RobotView:
    def __init__(self, body, arrow, camera, wheels, label):
        self.body = body
        self.arrow = arrow
        self.camera = camera
        self.wheels = wheels
        self.plan = None
        self.plan_goal = None
        self.label = label
        self.domain = []


class ObjectView:
    def __int__(self, body):
        pass


class WorldRenderer:
    def __init__(self, world : World):
        self.world = world

        self.visual_pipe, self.server_pipe = mp.Pipe()
        self.server_dispatcher = ActionDispatcher(self.world, self)

        self.robots = {}
        self.objects = {}

        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=800, height=600, background='white')
        self.canvas.pack()

        self.scale = 40

        self.root.bind('<Button-1>', self._click)

        self.update()

    def _random_color(self):
        rgb = colorsys.hsv_to_rgb(random.uniform(0, 1), 0.5, 0.83)
        return '#{:02x}{:02x}{:02x}'.format(*[int(x * 255) for x in rgb])

    def update(self):
        dt = 30/1000.0
        self.world.update(dt)
        self._update_robots(dt)

        if self.visual_pipe.poll():
            req = self.visual_pipe.recv()
            resp = self.server_dispatcher.dispatch(req)
            self.visual_pipe.send(resp)

        self.root.after(30, self.update)

    def run(self):
        self.root.mainloop()

    def calc_plan(self, robot, long, lat):
        x = long * self.scale
        y = lat * self.scale

        view = self.robots[robot]

        if view.plan_goal:
            self.canvas.delete(view.plan_goal)

        sp = np.array(vec_from_angle(robot.state[-1])) * 0.5 + robot.state[-3:-1]
        p = Planner([o.points for o in self.world.objects], (*sp, robot.state[-1]))
        p.use_alpha = True

        def plan_iter(gen):
            line = None
            final_plan = None
            for plan in gen:
                if line:
                    self.canvas.delete(line)

                flat_plan = [p * self.scale for x,y,_ in plan for p in (x,y)]
                line = self.canvas.create_line(*flat_plan)

                final_plan = plan
                self.root.update()
            return final_plan, line

        robot_plan, pline = plan_iter(p.plan((long, lat, np.pi)))

        if robot_plan:
            view.plan = pline
            view.plan_goal = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=self.canvas.itemcget(view.body, 'fill'))
            robot.plan = robot_plan

    def _click(self, ev):
        self.canvas.delete('click-pos')
        self.canvas.create_text(10, 10, anchor=tk.NW,
                                text='lat = {0}, long = {1}'.format(ev.x / self.scale, ev.y / self.scale), tags=('click-pos',))

    def _create_robot_view(self) -> RobotView:
        wheel_fl = self.canvas.create_line([0, 0, 0, 0], width=6)
        wheel_fr = self.canvas.create_line([0, 0, 0, 0], width=6)

        fill = self._random_color()
        body = self.canvas.create_polygon([0, 0, 0, 0], fill=fill, outline='black')

        arrow = self.canvas.create_line([0, 0, 0, 0], arrow=tk.FIRST)

        camera = self.canvas.create_line([0, 0, 0, 0], fill='gray')

        name = self.canvas.create_text(0, 0, anchor=tk.E)

        return RobotView(body, arrow, camera, [wheel_fl, wheel_fr], name)

    def _update_robot_view(self, robot: Robot, view: RobotView):
        body = self._build_body(robot.position, robot.width, robot.height, robot.angle)
        arrow = self._build_arrow(robot.position, robot.width, robot.height, robot.angle)
        camera = self._build_frustum(robot.position, robot.angle + robot.cam_angle, robot.cam_width)
        wheel_fl = self._build_wheel(robot.position, [-robot.width/2, robot.height/2],
                                     robot.angle, robot.wheels_angle/2)
        wheel_fr = self._build_wheel(robot.position, [+robot.width/2, robot.height/2],
                                     robot.angle, robot.wheels_angle/2)

        self.canvas.coords(view.body, body)
        self.canvas.coords(view.arrow, arrow)
        self.canvas.coords(view.camera, camera)

        self.canvas.coords(view.wheels[0], wheel_fl)
        self.canvas.coords(view.wheels[1], wheel_fr)

        self.canvas.coords(view.label, [robot.position[0] * self.scale, robot.position[1] * self.scale])
        self.canvas.itemconfigure(view.label, text=robot.name)

        dom_lines = self._build_domain(robot)
        if len(view.domain) != len(dom_lines):
            for dl in view.domain:
                self.canvas.delete(dl)
            view.domain = []
            for _ in dom_lines:
                view.domain.append(self.canvas.create_line(0, 0, 10, 10, dash=(4, 4)))

        for coords, line in zip(dom_lines, view.domain):
            self.canvas.coords(line, *coords)

        if not robot.plan:
            self.canvas.delete(view.plan)
            self.canvas.delete(view.plan_goal)
            view.plan = None
            view.plan_goal = None

    def _create_object_view(self, object_: WorldObject):
        point_list = [x*self.scale for p in object_.points.exterior.coords for x in p]
        poly = self.canvas.create_polygon(point_list, fill='#d0d0d0', outline='black')

        stands = [self.canvas.create_oval(x*self.scale-2, y*self.scale-2, x*self.scale+2, y*self.scale+2)
                  for x, y in object_.stands]
        label = self.canvas.create_text(object_.points.centroid.x * self.scale, object_.points.centroid.y * self.scale,
                                        text=object_.name, anchor=tk.CENTER)
        return poly, stands, label

    def _build_domain(self, robot):
        lines = []
        for name in robot.domain:
            r = self._robot_by_name(name)
            lines.append([x * self.scale for x in [*robot.position, *r.position]])
        return lines

    def _build_arrow(self, center, width, height, alpha):
        k = self.scale
        c = np.array(center)
        o = alpha - np.pi/2

        m = np.array([[np.cos(o), -np.sin(o)], [np.sin(o), np.cos(o)]])

        points = [
            c*k + np.dot(m, [0, height/2 * k]),
            c*k + np.dot(m, [0, 0])
        ]

        return [x for p in points for x in p]

    def _build_frustum(self, center, alpha, angle):
        k = self.scale
        c = np.array(center)
        o = alpha - np.pi/2

        m = np.array([[np.cos(o), -np.sin(o)], [np.sin(o), np.cos(o)]])

        points = [
            c*k,
            c*k + np.dot(m, [1.5 * np.tan(angle/2.0) * k, 1.5 * k]),
            c*k + np.dot(m, [-1.5 * np.tan(angle/2.0) * k, 1.5 * k]),
            c*k
        ]

        return [x for p in points for x in p]

    def _build_body(self, center, width, height, alpha):
        k = self.scale
        c = np.array(center)
        o = alpha - np.pi/2

        m = np.array([[np.cos(o), -np.sin(o)], [np.sin(o), np.cos(o)]])

        points = [
            c*k + np.dot(m, [-width/2 * k, height/2 * k]),
            c*k + np.dot(m, [+width/2 * k, height/2 * k]),
            c*k + np.dot(m, [+width/2 * k, -height/2 * k]),
            c*k + np.dot(m, [-width/2 * k, -height/2 * k]),
            ]

        return [x for p in points for x in p]

    def _build_wheel(self, center, pos, alpha, w_alpha):
        k = self.scale
        c = np.array(center)
        o = alpha - np.pi/2
        o2 = alpha + w_alpha - np.pi/2

        m1 = np.array([[np.cos(o), -np.sin(o)], [np.sin(o), np.cos(o)]])
        m2 = np.array([[np.cos(o2), -np.sin(o2)], [np.sin(o2), np.cos(o2)]])

        p = np.dot(m1, pos)
        points = [
            c*k + p*k + np.dot(m2, [0, 0.15 * k]),
            c*k + p*k + np.dot(m2, [0, -0.15 * k])
        ]

        return [x for p in points for x in p]

    def _update_robots(self, dt):
        if len(self.world.objects) != len(self.objects):
            for object_ in self.world.objects:
                if object_ not in self.objects:
                    self.objects[object_] = self._create_object_view(object_)
        if len(self.world.robots) != len(self.robots):
            for robot in self.world.robots:
                if robot not in self.robots:
                    self.robots[robot] = self._create_robot_view()

        for robot, view in self.robots.items():
            self._update_robot_view(robot, view)

    def _robot_by_name(self, name):
        matches = [robot for robot in self.robots if robot.name == name]
        return matches[0] if len(matches) > 0 else None
