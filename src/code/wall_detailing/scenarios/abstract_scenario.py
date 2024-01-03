from abc import ABC, abstractmethod


class Scenario(ABC):
    def __init__(self):
        self.walls = []

    def load(self):
        self.walls = self.get_walls()

    @abstractmethod
    def get_walls(self):
        return []
