import Sofa
import Sofa.Core as SC
import SofaRuntime
from source.nodes.cylinder import CylinderNode

def MainHeader(parent):

    rootNode = SC.Node("root")
    rootNode.dt = 0.02
    rootNode.gravity = [0, 0, 0]

    rootNode.addObject('RequiredPlugin',
                       pluginName='SofaRigid SoftRobots SofaPython3 SofaSparseSolver SofaOpenglVisual SofaConstraint SofaGeneralTopology SofaImplicitOdeSolver SofaSimpleFem SofaLoader SofaGeneralLoader SofaEngine SofaGeneralSimpleFem SofaDeformable SofaMiscCollision SofaBoundaryCondition SofaGeneralEngine SofaGeneralDeformable SofaGeneralObjectInteraction SofaMeshCollision ')
    
    # Not found: MultiThreading
    SofaRuntime.importPlugin("SofaComponentAll")

    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels showBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields hideInteractionForceFields showWireframe')


    rootNode.addObject('ParallelBruteForceBroadPhase')
    rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('CollisionPipeline', depth=5, draw=0, verbose=1)
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=0.01, contactDistance=0.0, angleCone=0.0)
    rootNode.addObject('DefaultContactManager', name='defaultContactManager1', response='PenalityContactForceField')
    rootNode.addObject('CollisionGroup', name='Group')
    return parent

def CylinderNavigation(parent, gridSize, axis, n_cylinders, position, radii, lengths):

    for i in range(n_cylinders):
        CylinderNode(parent, name="Cylinder_"+str(i), gridSize=gridSize, axis=axis, 
                    length=lengths[i], radius=radii[i], translation=position[i])
    return parent

def addController(parent, controller):
    parent.addObject(controller(name="Controller", node=parent))
    return parent