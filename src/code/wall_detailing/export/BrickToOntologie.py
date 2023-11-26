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
            b.id = idd
            idd += 1
            dic[b.id] = (b, brick)

        building = self.onto.Building("building_1")
        empty = self.onto.PlacedBrick("empty_brick")
        empty.dependsOn = []
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
            b.hasBeenSet = False
            b.dependsOn = [empty]
            b.dependsOn.extend(b.hasBottomNeighbor)
            building.hasBrick.append(b)

        #sync_reasoner_pellet(self.onto, infer_property_values=True)




        with self.onto:
            def sparql():
                sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
                tmp = self.onto.world.sparql("""
                                        PREFIX b: <http://www.semanticweb.org/rosrunner/ontologies/2023/10/brick_deduction#>
                                        INSERT {
                                            ?brick b:dependsOn ?n .
                                        }
                                        WHERE {
                                            ?brick a b:ConcreteBrick .
                                            ?brick b:hasBottomNeighbor ?n .
                                        }
                                    """)
                print("lalala",tmp)
                tmp = self.onto.world.sparql("""
                                        PREFIX b: <http://www.semanticweb.org/rosrunner/ontologies/2023/10/brick_deduction#>
                                        INSERT {
                                             ?test_brick_1 rdf:type b:ConcreteBrick .
                                            ?test_brick_1 b:hasBeenSet true .
                                        }
                                        WHERE {
                                            ?test_brick_1 rdfs:subClassOf b:ConcreteBrick .
                                        }
                                        """)
                #print("lalala",tmp)
            def swrl():
                pass
                #rule = Imp(1)
                #rule.set_as_rule("""hasBottomNeighbor(?b, ?n) -> dependsOn(?b, ?n)""")
                #sync_reasoner_hermit(self.onto, infer_property_values=True)

            for b in self.onto.ConcreteBrick.instances():
                if b != empty:
                    close_world(b, Properties=[self.onto.dependsOn])

            sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
            one = dic[7][1]
            print(one.get_properties())

        tmp = list(self.onto.world.sparql("""
                                            PREFIX b: <http://www.semanticweb.org/rosrunner/ontologies/2023/10/brick_deduction#>
                                            SELECT ?brick
                                            WHERE { 
                                                ?brick a b:ConcreteBrick .
                                                ?brick b:hasBottomNeighbor ?top .
                                            }
                                        """))
        print("select", len(list(tmp)), list(tmp))
        print("all", self.onto.Brick.instances())
        print("placeable", self.onto.PlaceableBrick.instances())
        print("next", self.onto.NextBrick.instances())
        print("placed", self.onto.PlacedBrick.instances())

        self.onto.save(file=self.working_file, format="rdfxml")
