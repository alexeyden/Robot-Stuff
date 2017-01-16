import couchdb

from .base import DaoBase


class Observations(DaoBase):
    DB_NAME = 'observ'

    def __init__(self, storage: couchdb.Server):
        super().__init__(storage)
        self._ensure_db(self.DB_NAME)

    def get_by_series_id(self, series_id):
        pass

    def get_by_sensor_id(self, sensor_id):
        pass

    def fetch_image_by_id(self, observ_id):
        pass


class ObservationSeries(DaoBase):
    DB_NAME = 'observ_series'

    def __init__(self, storage: couchdb.Server):
        super().__init__(storage)
        self._ensure_db(self.DB_NAME)

    def get_by_osm(self, osm_id):
        pass


class ObservationObjects(DaoBase):
    DB_NAME = 'observ_object'

    def __init__(self, storage: couchdb.Server):
        super().__init__(storage)
        self._ensure_db(self.DB_NAME)
