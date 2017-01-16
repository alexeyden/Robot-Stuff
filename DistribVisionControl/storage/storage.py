import couchdb

from .dao.observations import *
from .dao.sensors import *
from .dao.vision import *

class Storage:
    def __init__(self, host):
        self.server = couchdb.Server(host)

        self.decision_nodes = DecisionNode(self.server)
        self.classifier_data = ClassifierDataRev(self.server)
        self.sensors = Sensors(self.server)
        self.sensor_types = SensorTypes(self.server)
        self.observ_series = ObservationSeries(self.server)
        self.observations = Observations(self.server)
        self.observed_objects = ObservationObjects(self.server)
