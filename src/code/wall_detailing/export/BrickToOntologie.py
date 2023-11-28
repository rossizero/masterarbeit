from typing import List
from owlready2 import *

from wall_detailing.masonry.brick import Brick, Neighbor

# TODO this returns nothing: Brick and (dependsOn min 0 PlacedBrick) and not(dependsOn some PlaceableBrick)
# TODO                       Brick and dependsOn some PlacedBrick and not(dependsOn some PlaceableBrick)


class RuleSet:
    def __init__(self, ontology: Ontology):
        self.rules = []
        self.onto = ontology

    def add_rule(self, rule):
        self.rules.append(rule)


class Rule:
    """
    PropertyA(BrickA, BrickB) partOf PropertyB(BrickA, BrickB) = For each BrickA that has PropertyA, it will have PropertyB applied for a BrickB too
    PropertyA(BrickA) max/min number = BrickA can only have max/min number of PropertyA to be placeable
    """
    def __init__(self, name, rule):
        self.name = name
        self.rule = rule

        self.effectedPropertyName = None

    def apply(self, ontology: Ontology):
        if ontology.holdPropertyByName in self.rule.get_properties():
            prop = ontology.holdPropertyByName[self.rule]
            self.effectedPropertyName = prop[0]

        if ontology.ApplyObjectPropertyRule in self.rule.is_a:
            if ontology.applyObjectPropertyTo not in self.rule.get_properties():
                return None

            prop = ontology.applyObjectPropertyTo[self.rule]

            if len(prop) == 0:
                return None

            holder = prop[0]
            if ontology.holdPropertyByName not in holder.get_properties():
                return None

            property2 = ontology.holdPropertyByName[holder][0]
            print(self.effectedPropertyName, "applyObjectPropertyTo", property2)

            if (ontology[self.effectedPropertyName].domain != ontology[property2].domain
                    or ontology[self.effectedPropertyName].range != ontology[property2].range):
                print("[WARNING] domain or range mismatch between", self.effectedPropertyName, "and", property2,
                      ": (D", ontology[self.effectedPropertyName].domain, "vs", ontology[property2].domain,
                      ") and (R", ontology[self.effectedPropertyName].range, "vs", ontology[property2].range, ")")

            for brick in ontology.NamedBrick.instances():
                if ontology[self.effectedPropertyName] in brick.get_properties():
                    for neighbor in ontology[self.effectedPropertyName][brick]:
                        ontology[property2][brick].append(neighbor)

        elif ontology.CardinalityRule in self.rule.is_a:
            print("I am a cardinality rule")
        else:
            print("I am a " + self.name + " rule")


class BrickToOntology:
    def __init__(self, bricks: List[Brick]):
        self.original_file = "file:///home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/brick_deduction.rdf"
        self.working_file = "/home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/temporary_working_env.rdf"

        if os.name == 'nt':  # windows
            owlready2.JAVA_EXE = "C:\\Program Files\\Java\\jre-1.8\\bin\\java.exe"
            self.original_file = "E:\\Git\\masterarbeit\\src\\ontologies\\brick_deduction.rdf"
            self.working_file = "E:\\Git\\masterarbeit\\src\\ontologies\\temporary_working_env.rdf"

        self.bricks = bricks
        # self.world = World()

        self.onto = get_ontology(self.original_file).load()
        self.namespace = self.onto.get_namespace(self.original_file).name

        # Extract rules
        rules = []
        for ruleset in self.onto.Ruleset.instances():
            rs = RuleSet(self.onto)
            for rule in ruleset.hasRule:
                r = Rule(rule.name, rule)
                rs.add_rule(r)
            rules.append(rs)

        # Create bricks for building
        building = self.onto.Building("building_1")
        empty = self.onto.PlacedBrick("empty_brick")

        idd = 1
        dic = {}

        for b in bricks:
            brick = self.onto.NamedBrick("brick_" + str(idd))
            brick.hasID = [idd]
            b.id = idd
            idd += 1
            dic[b.id] = (b, brick)

        for b in self.onto.NamedBrick.instances():
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
            building.hasBrick.append(b)

        with self.onto:
            def sparql_tests():
                sync_reasoner_hermit(infer_property_values=True) # sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
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
                print("lalala", tmp)
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
                tmp = list(self.onto.world.sparql("""
                            PREFIX b: <http://www.semanticweb.org/rosrunner/ontologies/2023/10/brick_deduction#>
                            SELECT ?brick
                            WHERE { 
                                ?brick a b:NamedBrick .
                                ?brick b:hasTopNeighbor ?top .
                            }
                        """))
                print("select", len(list(tmp)), list(tmp))
            def swrl_tests():
                pass
                #rule = Imp(1)
                #rule.set_as_rule("""ConcreteBrick(?b), hasBottomNeighbor(?b, ?n) -> dependsOn(?b, ?n)""")
                #sync_reasoner_hermit(self.onto, infer_property_values=True)

            sync_reasoner_hermit(infer_property_values=True)#, infer_data_property_values=True)

            for ruleset in rules:
                for rule in ruleset.rules:
                    rule.apply(self.onto)

            for b in self.onto.NamedBrick.instances():
                if b != empty:
                    pass
                    close_world(b, Properties=[self.onto.dependsOn])

            sync_reasoner_hermit(infer_property_values=True) #, infer_data_property_values=True)

        print("test", self.onto["hasBottomNeighbor"][self.onto.NamedBrick.instances()[8]])

        print("all", self.onto.Brick.instances())
        print("placeable", self.onto.PlaceableBrick.instances())
        print("next", len(self.onto.NextBrick.instances()), self.onto.NextBrick.instances())
        print("placed", self.onto.PlacedBrick.instances())

        self.onto.save(file=self.working_file, format="rdfxml")
