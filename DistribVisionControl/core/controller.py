from server.server import Server


class Controller:
    def __init__(self):
        self.server = Server()

    def process_master(self, task):
        pass

    def process_slave(self, order):
        pass
