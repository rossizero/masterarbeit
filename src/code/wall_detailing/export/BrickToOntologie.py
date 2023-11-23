from typing import List
from owlready2 import *

from wall_detailing.masonry.brick import Brick, Neighbor


class BrickToOntology:
    def __init__(self, bricks: List[Brick]):
        self.bricks = bricks
        self.onto = get_ontology("file:///home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/brick_deduction.rdf").load()
        print(list(self.onto.classes()))

        # set ids
        idd = 1
        dic = {}
        for b in bricks:
            brick = self.onto.Brick("brick_" + str(idd))
            brick.hasID = [idd]
            brick.dependsOn = []
            # brick.hasBeenSet = [False]

            b.id = idd
            idd += 1
            dic[b.id] = (b, brick)

        building = self.onto.Building("building_1")
        for b in self.onto.Brick.instances():
            local_brick = dic[b.hasID[0]][0]
            b.hasBottomNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.BOTTOM]]
            b.hasTopNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.TOP]]
            b.hasLeftNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.LEFT]]
            b.hasRightNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.RIGHT]]
            b.hasFrontNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.FRONT]]
            b.hasBackNeighbor = [dic[obj.id][1] for obj in local_brick.neighbors[Neighbor.BACK]]
            building.hasBrick.append(b)

        with self.onto:
            rule = Imp(1)
            #rule.set_as_rule("""Brick(?b), hasBottomNeighbor(?b, ?n) -> dependsOn(?b, ?n)""")
            rule.set_as_rule("""Brick(?b) -> hasBeenSet(?b, false)""")
            sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
            print(str(rule))

        print(self.onto.Brick.instances())
        print(self.onto.PlaceableBrick.instances())
        self.onto.save(file="/home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/brick_deduction_filled.rdf", format="rdfxml")
