

class DaoBase:
    def __init__(self, storage):
        self.storage = storage
        self.db = None

    def _ensure_db(self, name):
        if name not in self.storage:
            self.db = self.storage.create(name)
        else:
            self.db = self.storage[name]

    def get_all(self):
        return self.db.view('_all_docs')

    def get_by_id(self, id_):
        return self.db[id_]

    def save(self):
        pass

    def delete(self, id_):
        pass

    def conflicts(self):
        pass

