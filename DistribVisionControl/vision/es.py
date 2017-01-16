from .bow import *
from storage import storage


class DecisionNode:
    def __init__(self):
        self.label = None
        self.expr = None
        self.props = None
        self.parent = DecisionNode()
        self.children = [DecisionNode(), DecisionNode()]


class ExpertSystem:
    def __init__(self, s: storage.Storage):
        self.tree = DecisionNode()
        self.storage = storage.Storage()
        self.bow = BOW(None)

    def train(self, data):
        pass

    def save_tree(self):
        pass

    def get_class(self, image_or_obj_id):
        pass

    def get_id(self, obj_id):
        pass

    def part_classes(self):
        pass