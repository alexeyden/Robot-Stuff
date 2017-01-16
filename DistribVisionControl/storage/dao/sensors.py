import couchdb

from .base import DaoBase


class Sensors(DaoBase):
    DB_NAME = 'sensor'

    def __init__(self, storage: couchdb.Server):
        super().__init__(storage)
        self._ensure_db(self.DB_NAME)

    def get_by_robot_id(self, robot_id):
        pass


class SensorTypes(DaoBase):
    DB_NAME = 'sensor_type'

    def __init__(self, storage: couchdb.Server):
        super().__init__(storage)
        self._ensure_db(self.DB_NAME)
