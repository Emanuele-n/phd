def add_cube(rootNode, position):
        # Cube model
        cube = rootNode.addChild('cube')
        cube.addObject('EulerImplicitSolver', name='odesolver')
        cube.addObject('SparseLDLSolver', name='linearSolver')
        cube.addObject('MechanicalObject', template='Rigid3', position=position)
        cube.addObject('UniformMass', totalMass=0.001)
        cube.addObject('UncoupledConstraintCorrection')

        cubeScale = 5
        # Collision
        cubeCollis = cube.addChild('cubeCollis')
        cubeCollis.addObject('MeshOBJLoader', name='cube_loader', filename='data/mesh/smCube27.obj', triangulate=True, scale=cubeScale)
        cubeCollis.addObject('MeshTopology', src='@cube_loader')
        cubeCollis.addObject('MechanicalObject')
        cubeCollis.addObject('TriangleCollisionModel')
        cubeCollis.addObject('LineCollisionModel')
        cubeCollis.addObject('PointCollisionModel')
        cubeCollis.addObject('RigidMapping')

        # Visualization
        cubeVisu = cube.addChild('cubeVisu')
        cubeVisu.addObject('MeshOBJLoader', name='cube_loader', filename='data/mesh/smCube27.obj')
        cubeVisu.addObject('OglModel', name='Visual', src='@cube_loader', color=[0.0, 0.1, 0.5], scale=cubeScale)
        cubeVisu.addObject('RigidMapping')

        # Constraint 
        #cube.addObject('FixedConstraint', indices='0')

        #print("\nDEBUG: added cube\n")


def add_floor(rootNode, translation, rotation):
        # Floor model
        planeNode = rootNode.addChild('Plane')
        planeNode.addObject('MeshOBJLoader', name='plane_loader', filename='data/mesh/floorFlat.obj', triangulate=True, rotation=rotation, scale=10, translation=translation)
        planeNode.addObject('MeshTopology', src='@plane_loader')
        planeNode.addObject('MechanicalObject', src='@plane_loader')
        planeNode.addObject('TriangleCollisionModel', simulated=False, moving=False)
        planeNode.addObject('LineCollisionModel', simulated=False, moving=False)
        planeNode.addObject('PointCollisionModel', simulated=False, moving=False)
        planeNode.addObject('OglModel', name='Visual_plane', src='@plane_loader', color=[1, 0, 0, 1])

        #print("\nDEBUG: added floor\n")


def add_wall(rootNode, translation, rotation):
        # Wall model
        wall = rootNode.addChild('wall')
        wall.addObject('EulerImplicitSolver', name='odesolver')
        wall.addObject('SparseLDLSolver', name='linearSolver')
        wall.addObject('MechanicalObject', template='Rigid3d', translation=translation, rotation=rotation)
        wall.addObject('UniformMass', totalMass=100)
        wall.addObject('UncoupledConstraintCorrection')

        wallScale = 5
        # Collision
        wallCollis = wall.addChild('wallCollis')
        wallCollis.addObject('MeshOBJLoader', name='wall_loader', filename='data/mesh/wall.obj', triangulate=True, scale=wallScale)
        wallCollis.addObject('MeshTopology', src='@wall_loader')
        wallCollis.addObject('MechanicalObject')
        #CudaRigid2d, CudaRigid2f, CudaRigid3d, CudaRigid3f, CudaVec1d, CudaVec1f, CudaVec2d, CudaVec2f, CudaVec3d, CudaVec3d1, CudaVec3f, CudaVec3f1, CudaVec6d, CudaVec6f, 
        # Rigid2d, Rigid3d, Vec1d, Vec2d, Vec3d, Vec6d
        wallCollis.addObject('TriangleCollisionModel')
        wallCollis.addObject('LineCollisionModel')
        wallCollis.addObject('PointCollisionModel')
        wallCollis.addObject('RigidMapping')

        # Visualization
        wallVisu = wall.addChild('wallVisu')
        wallVisu.addObject('MeshOBJLoader', name='wall_loader', filename='data/mesh/wall.obj')
        wallVisu.addObject('OglModel', name='Visual', src='@wall_loader', color=[0.0, 0.1, 0.5, 0.5], scale=wallScale)
        wallVisu.addObject('RigidMapping')

        # Constraint 
        wall.addObject('FixedConstraint', indices='0')

        #print("\nDEBUG: added wall\n")


def add_organ(rootNode, path, translation, rotation, gpu):
        # Organ model
        organ = rootNode.addChild('organ')
        organ.addObject('EulerImplicitSolver', name='odesolver')
        organ.addObject('SparseLDLSolver', name='linearSolver')
        organ.addObject('MechanicalObject', template='Rigid3d', translation=translation, rotation=rotation)
        organ.addObject('UniformMass', totalMass=100)
        organ.addObject('UncoupledConstraintCorrection')

        organScale = 100
        # Collision
        organCollis = organ.addChild('organCollis')
        organCollis.addObject('MeshOBJLoader', name='organ_loader', filename=path, triangulate=True, scale=organScale)
        organCollis.addObject('MeshTopology', src='@organ_loader')
        organCollis.addObject('MechanicalObject')
        #CudaRigid2d, CudaRigid2f, CudaRigid3d, CudaRigid3f, CudaVec1d, CudaVec1f, CudaVec2d, CudaVec2f, CudaVec3d, CudaVec3d1, CudaVec3f, CudaVec3f1, CudaVec6d, CudaVec6f, 
        # Rigid2d, Rigid3d, Vec1d, Vec2d, Vec3d, Vec6d
        organCollis.addObject('TriangleCollisionModel')
        organCollis.addObject('LineCollisionModel')
        organCollis.addObject('PointCollisionModel')
        organCollis.addObject('RigidMapping')

        # Visualization
        organVisu = organ.addChild('organVisu')
        organVisu.addObject('MeshOBJLoader', name='organ_loader', filename=path)
        organVisu.addObject('OglModel', name='Visual', src='@organ_loader', color=[1.0, 0.0, 0.0, 1], scale=organScale)
        organVisu.addObject('RigidMapping')

        # Constraint 
        organ.addObject('FixedConstraint', indices='0')

        #print("\nDEBUG: added organ\n")
        

def add_spring(rootNode, translationSpring, anglesSpring, youngModulusSpring, youngModulusStiffLayerSpring, gpu):
        # Spring model
        spring = rootNode.addChild('spring')
        spring.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        spring.addObject('SparseLDLSolver', name='preconditioner')
        spring.addObject('MeshVTKLoader', name='loader', filename='data/mesh/1dof/spring.vtk', scale=10, translation=translationSpring, rotation=anglesSpring)
        spring.addObject('MeshTopology', src='@loader', name='container')
        spring.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
        spring.addObject('UniformMass', totalMass=0.04)
        if gpu:                
                spring.addObject('ParallelTetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=youngModulusSpring)
        else:
                spring.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=youngModulusSpring)
        
        spring.addObject('BoxROI', name='boxROI', box=[20, 15, -10, -18, 35, 10], doUpdate=False, drawBoxes=True)
        spring.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12)
        spring.addObject('LinearSolverConstraintCorrection')

        # Sub topology
        modelSubTopoSpring = spring.addChild('modelSubTopo')
        modelSubTopoSpring.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI.tetrahedraInROI', name='container')
        if gpu:
                modelSubTopoSpring.addObject('ParallelTetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=youngModulusStiffLayerSpring - youngModulusSpring)
        else:
                modelSubTopoSpring.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=youngModulusStiffLayerSpring - youngModulusSpring)
                
        # Collision
        collisionSpring = spring.addChild('collisionSpring')
        collisionSpring.addObject('MeshSTLLoader', name='loader', filename='data/mesh/1dof/spring.stl', scale=10, translation=translationSpring, rotation=anglesSpring)
        collisionSpring.addObject('MeshTopology', src='@loader', name='topo')
        collisionSpring.addObject('MechanicalObject', name='collisMech')
        collisionSpring.addObject('TriangleCollisionModel', selfCollision=False)
        collisionSpring.addObject('LineCollisionModel', selfCollision=False)
        collisionSpring.addObject('PointCollisionModel', selfCollision=False)
        collisionSpring.addObject('BarycentricMapping')

        # Visualization
        springVisu = spring.addChild('visu')
        springVisu.addObject('MeshSTLLoader', name='loader', filename='data/mesh/1dof/spring.stl', scale=10, translation=translationSpring, rotation=anglesSpring)
        springVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6])
        springVisu.addObject('BarycentricMapping') # it slows down the loading by a lot, understand if it is necessary. It's necessary to avoid: [WARNING] [SparseLDLSolver(preconditioner)] Invalid Linear System to solve. Please insure that there is enough constraints (not rank deficient).


        #print("\nDEBUG: added spring\n")


def add_catheter(rootNode, translationCatheter, anglesCathter, youngModulusCatheters, youngModulusStiffLayerCatheters, gpu):
        
        cathScale = 20
        
        # Catheter Model
        catheter = rootNode.addChild('catheter')
        catheter.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        catheter.addObject('SparseLDLSolver', name='preconditioner')
        catheter.addObject('MeshVTKLoader', name='loader', filename='data/mesh/1dof/cylinder_with_cavity.vtk', scale=cathScale, translation=translationCatheter, rotation=anglesCathter)
        catheter.addObject('MeshTopology', src='@loader', name='container')
        catheter.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
        catheter.addObject('UniformMass', totalMass=0.04)
        if gpu:
                catheter.addObject('ParallelTetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=youngModulusCatheters)
        else:
                catheter.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=youngModulusCatheters)
        catheter.addObject('BoxROI', name='boxROI', box=[20, 15, -10, -18, 35, 10], doUpdate=False, drawBoxes=False)
        #catheter.addObject('BoxROI', name='boxROI', box=[5, -0.4, -0.4, 7, 0.4, 0.4], doUpdate=False, drawBoxes=True)
        #catheter.addObject('BoxROI', name='boxROISubTopo', box=[-118, 22.5, 0, -18, 28, 8], strict=False, drawBoxes=False)
        catheter.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12)
        catheter.addObject('LinearSolverConstraintCorrection')

        # Sub topology
        #modelSubTopo = catheter.addChild('modelSubTopo')
        #modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI', name='container')
        #modelSubTopo.addObject('ParallelTetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=youngModulusStiffLayerCatheters - youngModulusCatheters)

        # Constraint
        cavity = catheter.addChild('cavity')
        cavity.addObject('MeshSTLLoader', name='loader', filename='data/mesh/1dof/cavity.stl', scale=cathScale, translation=translationCatheter, rotation=anglesCathter)
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity')
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0001, triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

        # Collision	
        collisionCatheter = catheter.addChild('collisionCatheter')
        collisionCatheter.addObject('MeshSTLLoader', name='loader', filename='data/mesh/1dof/cylinder_with_cavity.stl', scale=cathScale, translation=translationCatheter, rotation=anglesCathter)
        collisionCatheter.addObject('MeshTopology', src='@loader', name='topo')
        collisionCatheter.addObject('MechanicalObject', name='collisMech')
        collisionCatheter.addObject('TriangleCollisionModel', selfCollision=False)
        collisionCatheter.addObject('LineCollisionModel', selfCollision=False)
        collisionCatheter.addObject('PointCollisionModel', selfCollision=False)
        collisionCatheter.addObject('BarycentricMapping')

        # Visualization	
        modelVisu = catheter.addChild('visu')
        modelVisu.addObject('MeshSTLLoader', name='loader', filename='data/mesh/1dof/cylinder_with_cavity.stl', scale=cathScale, translation=translationCatheter, rotation=anglesCathter)
        modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6])
        modelVisu.addObject('BarycentricMapping')

        #print("\nDEBUG: added catheter\n")