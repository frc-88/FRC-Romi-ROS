import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging

class RomiServer:
    def __init__(self):
        logging.basicConfig(level=logging.DEBUG)

        self.base_name = "romi"

        NetworkTables.initialize()
        self.table = NetworkTables.getTable("romi")

    def push(self, root, msg):
        for key in msg:
            subpath = root + "/" + key
            self.table.putNumber(subpath, msg[key])

    def pull(self, root, default=0.0):
        return self.table.getNumber(root, default)

