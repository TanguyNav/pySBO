# -*- coding: utf-8 -*-
"""Trunk.py: create scene of the Trunk and the controler to actuate it.
"""

__authors__ = "emenager, tnavez"
__contact__ = "etienne.menager@inria.fr, tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Jun 29 2022"


import sys
import pathlib
import os
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))
MeshPath = os.path.dirname(os.path.abspath(__file__))+'/Mesh/'

from BaseController import BaseController

class Controller(BaseController):
    """See BaseController for a detailed description.
    """
    def __init__(self, *args, **kwargs):
        super(Controller,self).__init__(*args, **kwargs)
    def get_actuators_state(self):
        return [float(cable.cableLength.value) for cable in self.list_actuators]
    def get_effectors_state(self):
        n_constraint = self.get_n_effectors_constraints()
        return [0 for _ in range(n_constraint)]
    def get_effectors_positions(self):
        return [effector_MO.position.value for effector_MO in self.list_effectors_MO]
    def apply_actions(self, values):
        assert len(self.list_actuators) == len(values)
        for actuator, value in zip(self.list_actuators, values):
            actuator.value.value = [value]



def add_goal_node(parent, position = [0, 0, 0], showObject = True):
    """Add goal node and MO in the scene.
    Parameters:
    ----------
        parent: Sofa.Node
            The parent of the goal node in scene tree.
        position: list of loat or list of list of float
            The position(s) of the goal.
        showObject: bool
            Flag to indicate if we want to display the goals.
    Outputs:
    -------
        goal_mo: Sofa.MechanicalObject
            The MechanicalObject of the goal.
    """
    goal = parent.addChild("Goal")
    goal_mo = goal.addObject('MechanicalObject', name='GoalMO', showObject=showObject, showObjectScale=35,
                             showColor=[0.5, 0.5, 0.5, 1.], position=position)
    return goal_mo

class Diamond():
    """ This class is implementing a parallel soft robot.
        The robot is entirely soft and actuated with 4 cables. This robot is composed
        of a mechanical model for the deformable structure.
        Parameters:
        -----------
            youngModulus: float
                The Young Modulus of the Trunk.
            poissonRatio: float
                The Poisson ratio of the Trunk.
            totalMass: float
                The total mass of the Trunk.
            inverseMode: bool
                If we use the Trunk in inverse mode.
    """

    def __init__(self, parentNode, youngModulus=3000, poissonRatio=0.45, totalMass=0.5, inverseMode=False, is_force=False):
        """Classical initialization of the class.

        Parameters:
        -----------
            parentNode: Sofa.Node
                The parent of the Trunk node in scene tree.
            youngModulus: float
                The Young Modulus of the Trunk.
            poissonRatio: float
                The Poisson ratio of the Trunk.
            totalMass: float
                The total mass of the Trunk.
            inverseMode: bool
                If we use the Trunk in inverse mode.
            is_force: bool
                Wether actuators are controlled in force or displacement.
                

        """

        self.inverseMode = inverseMode
        self.is_force = is_force
        self.node = parentNode.addChild('Diamond')

        self.node.addObject('MeshVTKLoader', name="loader", filename=MeshPath+'siliconeV0.vtu')
        self.node.addObject('MeshTopology', src="@loader")
        self.node.addObject('MechanicalObject', name="tetras", template="Vec3", showIndices=False, showIndicesScale=4e-5, rx=90, dz=35)


        self.node.addObject('UniformMass', totalMass=totalMass)
        self.node.addObject('TetrahedronFEMForceField', youngModulus=youngModulus, poissonRatio=poissonRatio)

        self.cables = self._addCables()


    def _addCables(self):
        """Private method to add cables in the Trunk.

        Outputs:
        --------
            The list of the cables.
        """
        actuators = self.node.addChild('Actuators')
        actuators.addObject('MechanicalObject', name="actuatedPoints", template="Vec3",
                    position=[[0, 0, 125], [0, 97, 45], [-97, 0, 45], [0, -97, 45], [97, 0, 45], [0, 0, 115]])
        
        valueType = "force" if self.is_force else "displacement"
        
        if self.inverseMode:
            north = actuators.addObject('CableActuator', template="Vec3", name="north" , indices=1, pullPoint=[0, 10, 30], maxPositiveDisp=20, minForce=0)
            west = actuators.addObject('CableActuator', template="Vec3", name="west", indices=2, pullPoint=[-10, 0, 30], maxPositiveDisp=20, minForce=0)
            south = actuators.addObject('CableActuator', template="Vec3", name="south", indices=3, pullPoint=[0, -10, 30], maxPositiveDisp=20, minForce=0)
            east = actuators.addObject('CableActuator', template="Vec3", name="east", indices=4, pullPoint=[10, 0, 30], maxPositiveDisp=20, minForce=0)
        else:
            north = actuators.addObject('CableConstraint', template="Vec3", name="north", indices=1, pullPoint=[0, 10, 30], valueType = valueType)
            west = actuators.addObject('CableConstraint', template="Vec3", name="west",  indices=2, pullPoint=[-10, 0, 30], valueType = valueType)
            south = actuators.addObject('CableConstraint', template="Vec3", name="south", indices=3, pullPoint=[0, -10, 30], valueType = valueType)
            east = actuators.addObject('CableConstraint', template="Vec3", name="east",  indices=4, pullPoint=[10, 0, 30], valueType = valueType)

        actuators.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

        return [north, west, south, east]

    def fixExtremity(self):
        """Fix the base of the Diamond.
        """
        self.node.addObject('BoxROI', name="boxROI", box=[-15, -15, -40,  15, 15, 10], drawBoxes=True)
        self.node.addObject('FixedConstraint', indices="@boxROI.indices")

    def addEffectors(self, target, position=[[0., 0., 195.]]):
        """Add a position effector in the Trunk.
        Parameters:
        ----------
            target:
            position: list of float or list of list of float
                The position of the effector(s) in the Trunk.
        """
        effectors = self.node.addChild("Effectors")
        effectors.addObject("MechanicalObject", position=position)
        effectors.addObject('PositionEffector', template='Vec3', indices = [i for i in range(len(position))], effectorGoal=target.position.getLinkPath())
        effectors.addObject("BarycentricMapping", mapForces=False, mapMasses=False)

def createScene(rootNode, classConfig):
    config = classConfig.get_scene_config()
    rootNode.addObject("RequiredPlugin", name="SoftRobots")
    rootNode.addObject("RequiredPlugin", name="SofaSparseSolver")
    rootNode.addObject("RequiredPlugin", name="SofaPreconditioner")
    rootNode.addObject("RequiredPlugin", name="SofaPython3")
    rootNode.addObject('RequiredPlugin', name='SofaOpenglVisual')
    rootNode.addObject('RequiredPlugin', name="SofaMiscCollision")
    rootNode.addObject("RequiredPlugin", name="SofaBoundaryCondition")
    rootNode.addObject("RequiredPlugin", name="SofaConstraint")
    rootNode.addObject("RequiredPlugin", name="SofaEngine")
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name="SofaSimpleFem")
    rootNode.addObject('RequiredPlugin', name="SofaDeformable")
    rootNode.addObject('RequiredPlugin', name="SofaGeneralLoader")
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')

    source = config["source"]
    target = config["target"]

    rootNode.addObject("LightManager")
    spotLoc = [2*source[0], 0, 0]
    rootNode.addObject("SpotLight", position=spotLoc, direction=[-np.sign(source[0]), 0.0, 0.0])
    rootNode.addObject("InteractiveCamera", name='camera', position=source, lookAt=target, zFar=500)

    rootNode.addObject('VisualStyle', displayFlags='showCollision showVisualModels showForceFields '
                                                   'showInteractionForceFields hideCollisionModels '
                                                   'hideBoundingCollisionModels hideWireframe')

    rootNode.addObject("DefaultVisualManagerLoop")

    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('FreeMotionAnimationLoop')
    if config["inverseMode"]:
        rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
        rootNode.addObject("QPInverseProblemSolver")
    else:
        constraint_solver = rootNode.addObject('GenericConstraintSolver', tolerance=1e-6, maxIterations=1000)

    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(0.3), name='Response',
                               response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)

    rootNode.gravity.value = [0., 0, -9810.]
    rootNode.dt.value = 0.01

    

    simulation = rootNode.addChild("Simulation")
    simulation.addObject('EulerImplicitSolver', name='odesolver', firstOrder= 0, rayleighMass=0.1,  rayleighStiffness=0.1)
    # simulation.addObject('ShewchukPCGLinearSolver', name='linearSolver', iterations=500, tolerance=1e-18,
    #                          preconditioners="precond")
    # simulation.addObject('SparseLDLSolver', name='precond', template = "CompressedRowSparseMatrixd")
    simulation.addObject('SparseLDLSolver', name='precond', template = "CompressedRowSparseMatrixd")
    simulation.addObject('GenericConstraintCorrection', solverName="precond")

    diamond = Diamond(simulation, inverseMode=config["inverseMode"], is_force=config["is_force"], totalMass = 0.0)
    rootNode.diamond = diamond
    diamond.fixExtremity()

    goal_mo = add_goal_node(rootNode, position = config["goalPos"], showObject = True)
    if config["inverseMode"]:
        diamond.addEffectors(goal_mo, position = [[0., 0., 125.]])
    else:        
        effectors = diamond.node.addChild("Effectors")
        effectors.addObject("MechanicalObject", position=[[0., 0., 125.]])
        effectors.addObject('PositionConstraint', template='Vec3', indices = [0], effectorGoal=goal_mo.position.getLinkPath(), valueType = "force", imposedValue = [0,0,0])
        effectors.addObject("BarycentricMapping", mapForces=False, mapMasses=False)

        rootNode.addObject(Controller(name="Controller", constraint_solver = constraint_solver, list_actuators = diamond.cables,  list_effectors = [effectors.PositionConstraint], list_effectors_MO = [effectors.MechanicalObject]))

    return rootNode
