from .world import World
from .robot import Robot

import math


class ActionDispatcher:
    def __init__(self, world: World, view):
        self.world = world
        self.view = view

    def dispatch(self, req):
        action = req['action']

        def robots_by_name(name):
            return [r for r in self.world.robots if r.name == name]

        if action == 'robot-connect':
            if robots_by_name(req['name']):
                return {'status': 'already exists'}

            robot = Robot((req['long'], req['lat']), 0)
            robot.name = req['name']
            self.world.robots.append(robot)
            return {'status': 'ok'}
        elif action == 'robot-disconnect':
            self.world.robots.remove(robots_by_name(req['name'])[0])
            return {'status': 'ok'}
        elif action == 'robot-status':
            robots = robots_by_name(req['name'])
            if not robots:
                return {'status': 'not found'}
            robot = robots[0]

            long, lat = robot.position
            dir_ = robot.angle
            plan = len(robot.plan)

            return {'status': 'ok', 'long': long, 'lat': lat, 'dir': dir_, 'plan': plan}
        elif action == 'robot-goto':
            robots = robots_by_name(req['name'])
            if not robots:
                return {'status': 'not found'}
            robot = robots[0]
            self.view.calc_plan(robot, req['long'], req['lat'])

            return {'status': 'ok'}
        elif action == 'robot-domain':
            robots = robots_by_name(req['name'])
            if not robots:
                return {'status': 'not found'}
            robots[0].domain = req['domain']
            return {'status': 'ok'}
        elif action == 'robot-cam-turn':
            robots = robots_by_name(req['name'])
            if not robots:
                return {'status': 'not found'}
            robot = robots[0]
            robot.cam_angle = req['angle'] * math.pi / 180

            return {'status': 'ok'}
        else:
            return {'status': 'no such action'}
