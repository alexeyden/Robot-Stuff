import vision.es


class Task:
    MODE_STAT = 1
    MODE_DYN = 2

    def __init__(self):
        self.osm_id = None
        self.long_lat = None
        self.robots = []
        self.sensors = []
        self.mode = None
        self.done = False
        self.cancelled = False


class Server:
    def __init__(self):
        self.port = None
        self.task = Task()
        self.es = vision.es.ExpertSystem(None)
        pass

    def start(self):
        pass

    def task(self):
        pass

    def cancel(self):
        pass

    def set_task(self, req):
        pass

    def set_order(self, req):
        pass

    def _handler(self, json):
        pass