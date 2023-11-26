from typing import List
from owlready2 import *

from wall_detailing.masonry.brick import Brick, Neighbor

# TODO this returns nothing: Brick and (dependsOn min 0 PlacedBrick) and not(dependsOn some PlaceableBrick)
# TODO                       Brick and dependsOn some PlacedBrick and not(dependsOn some PlaceableBrick)
class BrickToOntology:
    def __init__(self, bricks: List[Brick]):
        self.original_file = "file:///home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/brick_deduction.rdf"
        self.working_file = "/home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/temporary_working_env.rdf"

        if os.name == 'nt':  # windows
            owlready2.JAVA_EXE = "C:\\Program Files\\Java\\jre-1.8\\bin\\java.exe"
            self.original_file = "E:\\Git\\masterarbeit\\src\\ontologies\\brick_deduction.rdf"
            self.working_file = "E:\\Git\\masterarbeit\\src\\ontologies\\temporary_working_env.rdf"

        self.bricks = bricks
        self.onto = get_ontology(self.original_file).load()
        print(list(self.onto.classes()))

        # set ids
        idd = 1
        dic = {}
        for b in bricks:
            brick = self.onto.ConcreteBrick("brick_" + str(idd))
            #brick.is_a.append(self.onto.ConcreteBrick)
            brick.hasID = [idd]
            brick.hasBeenSet = False
            b.id = idd
            idd += 1
            dic[b.id] = (b, brick)

        building = self.onto.Building("building_1")
        empty = self.onto.PlacedBrick("empty_brick")
        for b in self.onto.ConcreteBrick.instances():
            if b == empty:
                continue
            local_brick = dic[b.hasID[0]][0]
            b.hasBottomNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.BOTTOM]]
            b.hasTopNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.TOP]]
            b.hasLeftNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.LEFT]]
            b.hasRightNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.RIGHT]]
            b.hasFrontNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.FRONT]]
            b.hasBackNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.BACK]]
            b.placed = False
            b.dependsOn = [empty]#, dic[1][1]]
            b.dependsOn.extend(b.hasBottomNeighbor)
            building.hasBrick.append(b)
            close_world(b, Properties=[self.onto.dependsOn])

        with self.onto:
            pass
            #rule = Imp(1)
            #rule.set_as_rule("""Brick(?b), hasBottomNeighbor(?b, ?n) -> dependsOn(?b, ?n)""")
            #rule.set_as_rule("""Brick(?b) -> hasBeenSet(?b, false)""")
            sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
            #print(str(rule))

        print("all", self.onto.Brick.instances())
        print("placeable", self.onto.PlaceableBrick.instances())
        print("placed", self.onto.PlacedBrick.instances())

        self.onto.save(file=self.working_file, format="rdfxml")
