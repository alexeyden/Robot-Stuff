import couchdb

from .base import DaoBase


class DecisionNode(DaoBase):
    DB_NAME = 'decision_tree'

    def __init__(self, storage: couchdb.Server):
        super().__init__(storage)
        self._ensure_db(self.DB_NAME)

    def get_tree(self):
        pass

    def get_subtree(self, node_id):
        pass

    def get_by_subclass(self, subclass):
        pass


class ClassifierDataRev(DaoBase):
    DB_NAME = 'classifier_data'

    def __init__(self, storage: couchdb.Server):
        super().__init__(storage)
        self._ensure_db(self.DB_NAME)

    def get_by_source(self, robot_id):
        pass