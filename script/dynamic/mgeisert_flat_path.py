## Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.rbprmbuilder import Builder

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer
import time
import math
import omniORB.any
from configs.mgeisert_flat import *

from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
from hpp.corbaserver.rbprm.tools.time_out import *

class Robot (Parent):
	rootJointType = 'freeflyer'
	packageName = 'hpp-rbprm-corba'
	meshPackageName = 'hpp-rbprm-corba'
	urdfName = 'hrp2_trunk_flexible'
	urdfSuffix = ""
	srdfSuffix = ""
	def __init__ (self, robotName, load = True):
		Parent.__init__ (self, robotName, self.rootJointType, load)
		self.tf_root = "base_footprint"
		self.client.basic = Client ()
		self.load = load
		


rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hrp2_trunk_arms_flexible'
urdfNameRoms =  [rLegId,lLegId,rArmId,lArmId]
urdfSuffix = ""
srdfSuffix = ""


# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)



# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom'])
rbprmBuilder.setAffordanceFilter('hrp2_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hrp2_rleg_rom', ['Support'])
vMax = 0.2;
aMax = 0.1;
extraDof = 6

#rbprmBuilder.setJointBounds ("base_joint_xyz", [-1,2.5,0.5 ,3, 0.6, 0.6])
#rbprmBuilder.setJointBounds ("base_joint_xyz", [-1,2.5,0.5 ,3, 0.6, 0.6])
base_bound =[-2.5,2.5,-2.5 ,2.5, 0.4, 0.7] 
rbprmBuilder.setJointBounds ("base_joint_xyz", base_bound)
rbprmBuilder.setJointBounds('CHEST_JOINT0',[-0.05,0.05])
rbprmBuilder.setJointBounds('CHEST_JOINT1',[-0.05,0.05])
# We also bound the rotations of the torso. (z, y, x)
#rbprmBuilder.boundSO3([-3.14,3.14,-0.65,0.65,-0.2,0.2])
#rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
#rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-1,1,-1,1,-2,2,-2,2,-2,2,-2,2])
rbprmBuilder.boundSO3([-math.pi,math.pi,-0.0,0.0,-0.0,0.0])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-1.,1.,-1.,1.,-2.,2.,0,0,0,0,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.basic.robot.getDimensionExtraConfigSpace()


# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )
ps.client.problem.setParameter("aMax",omniORB.any.to_any(aMax))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(vMax))
ps.client.problem.setParameter("orientedPath",omniORB.any.to_any(0.))
ps.client.problem.setParameter("friction",omniORB.any.to_any(MU))
ps.client.problem.setParameter("sizeFootX",omniORB.any.to_any(0.24))
ps.client.problem.setParameter("sizeFootY",omniORB.any.to_any(0.14))


r = Viewer (ps)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
#afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.2])
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.05])
#afftool.loadObstacleModel (packageName, ENV_NAME, ENV_PREFIX, r, reduceSizes=[0.2,0])
afftool.loadObstacleModel (packageName, ENV_NAME, ENV_PREFIX, r, reduceSizes=[0.14,0])
#r.loadObstacleModel (packageName, "ground", "planning")
afftool.visualiseAffordances('Support', r,r.color.lightYellow)
r.addLandmark(r.sceneName,1)

# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig ();
q_init[3:7] = [1.,0.,0.,0.]
q_init [0:3] = [0., 0., 0.6]; r (q_init)
#q_init [0:3] = [0.5, 2.5, 0.6]; r (q_init)


x = np.random.normal(size=2)
x = x/np.linalg.norm(x)
u = np.random.random()
v_init = x*u*vMax/2.

q_init[-6:-3] = [0.0, 0.05, 0.] #[v_init[0],v_init[1],0.]


rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]

x = np.random.normal(size=2)
x = x/np.linalg.norm(x)
u = np.random.random()
v_goal = x*u*vMax/2.

x_goal = np.random.random(2)

q_goal [0:3] = [1.5, 1., 0.6] #[x_goal[0]*4.-2., x_goal[1]*4.-2., 0.6] 
q_goal[-6:-3] = [0.0, 0.0, 0.] #[v_goal[0],v_goal[1],0.]
r(q_goal)
#q_goal [0:3] = [1.83, 0.86, 1.13]; r(q_goal)


# Choosing a path optimizer
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
#ps.client.problem.selectPathValidation("RbprmPathValidation",0.01)
ps.client.problem.selectPathValidation("RbprmDynamicPathValidation",0.05)
#ps.selectPathProjector('Progressive',0.05)
# Choosing kinodynamic methods : 
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
ps.selectPathPlanner("DynamicPlanner")
ps.addPathOptimizer("RandomShortcutDynamic")
ps.addPathOptimizer("OrientedPathOptimizer")

#solve the problem :
r(q_init)

import IPython
IPython.embed()
#r.solveAndDisplay("rm",1,radiusSphere=0.01)

#ps.client.problem.setTimeOutPathPlanning(10)
try:
  t = ps.solve()
except Exception, exc:
  print exc
  ErrorCode = 1
  raise exc





from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=0.03
pp.displayVelocityPath(2)

#import IPython
#IPython.embed()

#r.client.gui.setVisibility("path_2_root","ALWAYS_ON_TOP")




q_far = q_init[::]
q_far[2] = -3
r(q_far)




"""
camera = [0.6293167471885681,
 -9.560577392578125,
 10.504343032836914,
 0.9323806762695312,
 0.36073973774909973,
 0.008668755181133747,
 0.02139890193939209]
r.client.gui.setCameraTransform(0,camera)
"""


