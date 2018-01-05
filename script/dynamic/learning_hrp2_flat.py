## Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.rbprmbuilder import Builder

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer
import time
import math


from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

class Robot (Parent):
	rootJointType = 'freeflyer'
	packageName = 'hpp-rbprm-corba'
	meshPackageName = 'hpp-rbprm-corba'
	# URDF file describing the trunk of the robot HyQ
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
urdfNameRoms =  ['hrp2_lleg_rom','hrp2_rleg_rom']
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
rbprmBuilder.setJointBounds ("base_joint_xyz", [-3,4.5,-2 ,2.5, 0.55, 0.65])
rbprmBuilder.setJointBounds('CHEST_JOINT0',[-0.05,0.05])
rbprmBuilder.setJointBounds('CHEST_JOINT1',[-0.05,0.05])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-math.pi,math.pi,-0.1,0.1,-0.1,0.1])

# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )


r = Viewer (ps)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel ('hpp-rbprm-corba', "bauzil_stairs", "planning", r)
#r.loadObstacleModel (packageName, "ground", "planning")
afftool.visualiseAffordances('Support', r, r.color.lightBrown)
r.addLandmark(r.sceneName,1)

# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig ();
q_init[3:7] = [1,0,0,0]
q_init [0:3] = [-1, 1, 0.58]; r (q_init)
q_init[-6]=0.1

rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]


q_goal [0:3] = [-1, -1.5, 0.58]; r(q_goal)
q_goal[-6]=0
q_goal[-5]=-0.1
r (q_goal)


# Choosing a path optimizer
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.01)




#solve the problem :
r(q_init)


# Fait une seul iteration du planning (pour tester), a remplacer par ps.solve() quand c'est bon
ps.client.problem.prepareSolveStepByStep()
ps.client.problem.executeOneStep()


global i_sphere
i_sphere=0

def displaySpheres(q,color=r.color.black,size=0.03):
  global i_sphere
  for i in range(0,len(q)):
    r.client.gui.addSphere("s"+str(i+i_sphere),size,color)
    r.client.gui.applyConfiguration("s"+str(i+i_sphere),q[i]+[1,0,0,0])
    r.client.gui.setVisibility("s"+str(i+i_sphere),"ALWAYS_ON_TOP")
    r.client.gui.addToGroup("s"+str(i+i_sphere),r.sceneName)
  i_sphere += len(q)
  r.client.gui.refresh()




q_far = q_init[::]
q_far[2] = -3
r(q_far)






