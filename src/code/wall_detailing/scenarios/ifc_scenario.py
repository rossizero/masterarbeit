from wall_detailing.importer.ifc_importer import IfcImporter
from wall_detailing.scenarios.abstract_scenario import Scenario


class IFCScenario(Scenario):
    def __init__(self, ifc_file):
        self.ifc_file = ifc_file
        self.ifc_importer = IfcImporter(self.ifc_file)

        super().__init__()

    def get_walls(self):
        return self.ifc_importer.get_walls()
