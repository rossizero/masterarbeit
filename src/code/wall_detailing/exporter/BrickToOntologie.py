import random
from typing import List
from owlready2 import *

from wall_detailing.masonry.brick import Brick, Neighbor


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
        """
        Apply the rule to the ontology. Depending on the RuleType different actions are taken.
        :param ontology: The ontology to apply the rule to.

        """
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
    def __init__(self, bricks: List[Brick], building_name: str = "building_1"):
        self.original_file = "file:///home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/brick_deduction.rdf"
        self.working_file = "/home/rosrunner/Desktop/repos/masterarbeit/src/ontologies/temporary_working_env.rdf"
        self.use_pellet = True

        if os.name == 'nt':  # windows
            owlready2.JAVA_EXE = "C:\\Program Files\\Java\\jre-1.8\\bin\\java.exe"
            self.original_file = "E:\\Git\\masterarbeit\\src\\ontologies\\brick_deduction.rdf"
            self.working_file = "E:\\Git\\masterarbeit\\src\\ontologies\\temporary_working_env.rdf"

        self.onto = get_ontology(self.original_file).load()
        self.namespace = self.onto.get_namespace(self.original_file).name

        self.building_name = building_name
        self.brick_to_ontology_dictionary = {}  # because I am too lazy to retrieve the bricks by id from the ontology
        self.bricks = bricks

        # setup data step by step
        self.rules = self.load_rules()  # load rules from ontology
        self.fill_ontology()  # fill ontology with bricks
        self.apply_rules()  # apply rules to bricks
        self.onto.save(file=self.working_file, format="rdfxml")  # save ontology

    def load_rules(self):
        """
        Loads the rules from the ontology and returns them as a list of RuleSets
        """
        rules = []
        # Extract rules
        for ruleset in self.onto.Ruleset.instances():
            rs = RuleSet(self.onto)
            for rule in ruleset.hasRule:
                r = Rule(rule.name, rule)
                rs.add_rule(r)
            rules.append(rs)
        return rules

    def fill_ontology(self):
        """
        Fills the ontology with bricks
        """
        # Create bricks for building
        building = self.onto.Building(self.building_name)
        self.empty = self.onto.PlacedBrick("empty_brick")

        idd = 1

        for b in self.bricks:
            brick = self.onto.NamedBrick("brick_" + str(idd))
            brick.hasID = [idd]
            b.id = idd
            idd += 1
            self.brick_to_ontology_dictionary[b.id] = (b, brick)

        for b in self.onto.NamedBrick.instances():
            if b == self.empty:
                continue
            local_brick = self.brick_to_ontology_dictionary[b.hasID[0]][0]
            b.hasBottomNeighbor = [self.brick_to_ontology_dictionary[obj.id][1] for obj in local_brick.neighbors[Neighbor.BOTTOM]]
            b.hasTopNeighbor = [self.brick_to_ontology_dictionary[obj.id][1] for obj in local_brick.neighbors[Neighbor.TOP]]
            b.hasLeftNeighbor = [self.brick_to_ontology_dictionary[obj.id][1] for obj in local_brick.neighbors[Neighbor.LEFT]]
            b.hasRightNeighbor = [self.brick_to_ontology_dictionary[obj.id][1] for obj in local_brick.neighbors[Neighbor.RIGHT]]
            b.hasFrontNeighbor = [self.brick_to_ontology_dictionary[obj.id][1] for obj in local_brick.neighbors[Neighbor.FRONT]]
            b.hasBackNeighbor = [self.brick_to_ontology_dictionary[obj.id][1] for obj in local_brick.neighbors[Neighbor.BACK]]
            b.hasBeenSet = False
            b.dependsOn = [self.empty]
            building.hasBrick.append(b)

    def status(self):
        """
        Prints the status of the ontology
        """
        if len(self.onto.NamedBrick.instances()) == 0:
            print("No brick individuals in ontology")
            return
        print("test", self.onto["hasBottomNeighbor"][self.onto.NamedBrick.instances()[8]])
        print("all", self.onto.Brick.instances())
        print("placeable", self.onto.PlaceableBrick.instances())
        print("next", len(self.onto.NextBrick.instances()), self.onto.NextBrick.instances())
        print("placed", self.onto.PlacedBrick.instances())

    def apply_rules(self):
        """
        Applies the rules of the ontology to the bricks
        """
        with self.onto:
            for ruleset in self.rules:
                for rule in ruleset.rules:
                    rule.apply(self.onto)

            for b in self.onto.NamedBrick.instances():
                if b != self.empty:
                    pass
                    close_world(b, Properties=[self.onto.dependsOn])

    def deduct_building_plan(self):
        """
        Deducts a building plan by using the ontology and its reasoner to infer what bricks to place next
        Right now it just chooses random buildable bricks to proof the concept
        """
        ret = []
        for i in range(len(self.onto.NamedBrick.instances())):
            # create a new world and load a fresh ontology
            new_world = World()
            new_onto = new_world.get_ontology(self.working_file).load()

            # run selected reasoner
            if self.use_pellet:
                sync_reasoner_pellet(new_onto, infer_property_values=True, infer_data_property_values=True)
            else:
                sync_reasoner_hermit(new_onto, infer_property_values=True)

            # choose the next brick to be placed
            next_brick = random.choice(new_onto.NextBrick.instances())

            # look up next brick in dictionary (lazy)
            idd = next_brick.hasID[0]
            if idd in self.brick_to_ontology_dictionary.keys():
                self.brick_to_ontology_dictionary[idd][1].hasBeenSet = True
                ret.append(self.brick_to_ontology_dictionary[idd][0])
            self.onto.save(file=self.working_file, format="rdfxml")
        return ret

    def __sparql_tests(self):
        """
        Some sparql tests
        """
        sync_reasoner_hermit(
            infer_property_values=True)  # sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
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

    def __swrl_tests(self):
        """
        Some swrl tests
        """
        pass
        # rule = Imp(1)
        # rule.set_as_rule("""ConcreteBrick(?b), hasBottomNeighbor(?b, ?n) -> dependsOn(?b, ?n)""")
        # sync_reasoner_hermit(self.onto, infer_property_values=True)
