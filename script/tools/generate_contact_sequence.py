import pinocchio as se3
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
from config import *
from spline import bezier
import inspect

import locomote
from locomote import WrenchCone,SOC6,ControlType,IntegratorType,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
global i_sphere 

def pinnochioQuaternion(q):
    assert len(q)>6, "config vector size must be superior to 7"
    w = q[3]
    q[3:6] = q[4:7]
    q[6] = w
    return q

def addContactLandmark(M,color,viewer):
    global i_sphere
    try:
      gui = viewer.client.gui
      name = 's'+str(i_sphere)
      i_sphere += 1    
      gui.addSphere(name,0.01,color)
      #gui.setVisibility(name,"ALWAYS_ON_TOP")
      gui.addToGroup(name,viewer.sceneName)
      p = M.translation.transpose().tolist()[0]
      rot = Quaternion(M.rotation)
      p +=  [rot.w]
      p += rot.coeffs().transpose().tolist()[0][0:3]
      gui.applyConfiguration(name,p)
      gui.addLandmark(name,0.03) 
      #print "contact altitude : "+str(p[2])
    except:
      pass
    
def displayContactsFromPhase(phase,viewer):
  try:
    if phase.LF_patch.active:
        addContactLandmark(phase.LF_patch.placement*MLsole_display,viewer.color.red ,viewer)
    if phase.RF_patch.active:
        addContactLandmark(phase.RF_patch.placement*MRsole_display,viewer.color.green ,viewer)  
    if phase.LH_patch.active:
        addContactLandmark(phase.LH_patch.placement*MLhand_display,viewer.color.yellow ,viewer)
    if phase.RH_patch.active:
        addContactLandmark(phase.RH_patch.placement*MRhand_display,viewer.color.blue ,viewer)                
    viewer.client.gui.refresh() 
  except:
    pass
    
        



def generateContactSequence(fb,configs,beginId,endId,viewer=None, curves_initGuess = [], timings_initGuess = []):
    print "generate contact sequence from planning : "
    global i_sphere
    i_sphere = 0
    #print "MR offset",MRsole_offset
    #print "ML offset",MLsole_offset  
    n_double_support = len(configs)
    # config only contains the double support stance
    n_steps = n_double_support*2 -1 
    # Notice : what we call double support / simple support are in fact the state with all the contacts and the state without the next moving contact
    if len(curves_initGuess) == (len(configs)-1) and len(timings_initGuess) == (len(configs)-1):
        init_guess_provided = True
    else : 
        init_guess_provided = False
    print "generate CS, init guess provided : "+str(init_guess_provided)
    
    cs = ContactSequenceHumanoid(n_steps)
    unusedPatch = cs.contact_phases[0].LF_patch.copy()
    unusedPatch.placement = SE3.Identity()
    unusedPatch.active= False

    
    # for each contact state we must create 2 phase (one with all the contact and one with the next replaced contact(s) broken)
    for stateId in range(beginId,endId):
        # %%%%%%%%%  all the contacts : %%%%%%%%%%%%%
        cs_id = (stateId-beginId)*2
        config_id=stateId-beginId
        phase_d = cs.contact_phases[cs_id]
        fb.setCurrentConfig(configs[config_id])
        init_guess_for_phase = init_guess_provided
        if init_guess_for_phase:
            c_init_guess = curves_initGuess[config_id]
            t_init_guess = timings_initGuess[config_id]
            init_guess_for_phase = isinstance(c_init_guess,bezier)
            if init_guess_for_phase:
                print "bezier curve provided for config id : "+str(config_id)

        
        # compute MRF and MLF : the position of the contacts
        q_rl = fb.getJointPosition(rfoot)
        q_ll = fb.getJointPosition(lfoot)
        q_rh = fb.getJointPosition(rhand)
        q_lh = fb.getJointPosition(lhand)
        
        
        # feets
        MRF = SE3.Identity()
        MLF = SE3.Identity()
        MRF.translation = np.matrix(q_rl[0:3]).T
        MLF.translation = np.matrix(q_ll[0:3]).T
        if not FORCE_STRAIGHT_LINE : 
            rot_rl = Quaternion(q_rl[3],q_rl[4],q_rl[5],q_rl[6])
            rot_ll = Quaternion(q_ll[3],q_ll[4],q_ll[5],q_ll[6])
            MRF.rotation = rot_rl.matrix()
            MLF.rotation = rot_ll.matrix()
        
        # apply the transform ankle -> center of contact
        MRF *= MRsole_offset
        MLF *= MLsole_offset
        
        # hands
        MRH = SE3()
        MLH = SE3()
        MRH.translation = np.matrix(q_rh[0:3]).T
        MLH.translation = np.matrix(q_lh[0:3]).T
        rot_rh = Quaternion(q_rh[3],q_rh[4],q_rh[5],q_rh[6])
        rot_lh = Quaternion(q_lh[3],q_lh[4],q_lh[5],q_lh[6])
        MRH.rotation = rot_rh.matrix()
        MLH.rotation = rot_lh.matrix()   
        
        MRH *= MRhand_offset
        MLH *= MLhand_offset    
        
        phase_d.RF_patch.placement = MRF
        phase_d.LF_patch.placement = MLF
        phase_d.RH_patch.placement = MRH
        phase_d.LH_patch.placement = MLH
        
        
        # initial state : Set all new contacts patch (either with placement computed below or unused)
        if stateId==beginId:
            # FIXME : for loop ? how ?
            if fb.isLimbInContact(rLegId,stateId):
                phase_d.RF_patch.active = True
            else:
                phase_d.RF_patch.active = False
            if fb.isLimbInContact(lLegId,stateId):
                phase_d.LF_patch.active = True
            else:
                phase_d.LF_patch.active = False
            if fb.isLimbInContact(rArmId,stateId):
                phase_d.RH_patch.active = True
            else:
                phase_d.RH_patch.active = False
            if fb.isLimbInContact(lArmId,stateId):
                phase_d.LH_patch.active = True
            else:
                phase_d.LH_patch.active = False
        else:   
            # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement)
            phase_d.RF_patch = phase_s.RF_patch
            phase_d.RF_patch.active = fb.isLimbInContact(rLegId,stateId)
            phase_d.LF_patch = phase_s.LF_patch
            phase_d.LF_patch.active = fb.isLimbInContact(lLegId,stateId)
            phase_d.RH_patch = phase_s.RH_patch
            phase_d.RH_patch.active = fb.isLimbInContact(rArmId,stateId)
            phase_d.LH_patch = phase_s.LH_patch
            phase_d.LH_patch.active = fb.isLimbInContact(lArmId,stateId)
            
            # now we change the contacts that have moved : 
            variations = fb.getContactsVariations(stateId-1,stateId)
            #assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
            for var in variations:     
                # FIXME : for loop in variation ? how ?
                if var == lLegId:
                    phase_d.LF_patch.placement = MLF
                if var == rLegId:
                    phase_d.RF_patch.placement = MRF
                if var == lArmId:
                    phase_d.LH_patch.placement = MLH
                if var == rArmId:
                    phase_d.RH_patch.placement = MRH
                    
        # retrieve the COM position for init and final state (equal for double support phases)
        if init_guess_for_phase :
            dc_init_guess = c_init_guess.compute_derivate(1)
            ddc_init_guess = dc_init_guess.compute_derivate(1)
            if stateId != beginId: # not the first state. Take the trajectory for p2 of the previous curve and merge it with p0 of the current curve : 
                c_prev = curves_initGuess[config_id-1]
                dc_prev = c_prev.compute_derivate(1)
                ddc_prev = dc_prev.compute_derivate(1)                
                timings_prev = timings_initGuess[config_id-1]
                # generate init state : 
                init_state = [0]*9
                init_state[0:3] = c_prev(timings_prev[0] + timings_prev[1]).transpose().tolist()[0]
                init_state[3:6] = dc_prev(timings_prev[0] + timings_prev[1]).transpose().tolist()[0]
                init_state[6:9] = ddc_prev(timings_prev[0] + timings_prev[1]).transpose().tolist()[0]   
                init_state = np.matrix(init_state).transpose()   
            else : # first traj
                # generate init state : 
                init_state = [0]*9
                init_state[0:3] = c_init_guess(0).transpose().tolist()[0]
                init_state[3:6] = dc_init_guess(0).transpose().tolist()[0]
                init_state[6:9] = ddc_init_guess(0).transpose().tolist()[0]   
                init_state = np.matrix(init_state).transpose()            
            # generate final state : 
            final_state = [0]*9            
            final_state[0:3] = c_init_guess(t_init_guess[0]).transpose().tolist()[0]
            final_state[3:6] = dc_init_guess(t_init_guess[0]).transpose().tolist()[0]
            final_state[6:9] = ddc_init_guess(t_init_guess[0]).transpose().tolist()[0] 
            final_state = np.matrix(final_state).transpose()
            # set timing :
            t_tot = t_init_guess[0]
            if stateId != beginId:
                t_prev = (timings_prev[2]) 
                t_tot += t_prev
            phase_d.time_trajectory.append(t_tot)
            # generate init guess traj for state and control from bezier curve and it's derivative : 
            if config_id == 0:
                num_nodes = NUM_NODES_INIT
            else :
                num_nodes = NUM_NODES_DS
            time = np.linspace(0,t_tot,num=num_nodes,endpoint=False)  
            #print time
            for t in time:
                state = [0]*9
                if stateId == beginId:
                    state[0:3] = c_init_guess(t).transpose().tolist()[0]
                    state[3:6] = dc_init_guess(t).transpose().tolist()[0]
                    state[6:9] = ddc_init_guess(t).transpose().tolist()[0]
                else:
                    if t < t_prev : 
                        state[0:3] = c_prev(timings_prev[0]+timings_prev[1]+t).transpose().tolist()[0]
                        state[3:6] = dc_prev(timings_prev[0]+timings_prev[1]+t).transpose().tolist()[0]
                        state[6:9] = ddc_prev(timings_prev[0]+timings_prev[1]+t).transpose().tolist()[0]  
                    else : 
                        state[0:3] = c_init_guess(t-t_prev).transpose().tolist()[0]
                        state[3:6] = dc_init_guess(t-t_prev).transpose().tolist()[0]
                        state[6:9] = ddc_init_guess(t-t_prev).transpose().tolist()[0]                        
                u = [0]*6
                u [0:3] = state[6:9]
                #print t
                #print np.matrix(u).transpose()
                #print np.matrix(state).transpose()
                phase_d.control_trajectory.append(np.matrix(u).transpose())
                phase_d.state_trajectory.append(np.matrix(state).transpose())
        else :
            init_state = phase_d.init_state.copy()
            init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
            init_state[3:9] = np.matrix(configs[config_id][-6:]).transpose()
            final_state = init_state.copy()
            phase_d.time_trajectory.append((fb.getDurationForState(stateId))*DURATION_n_CONTACTS/SPEED)
        phase_d.init_state=init_state
        phase_d.final_state=final_state
        phase_d.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[config_id][:-6])).T)
        #print "done for double support"
        if DISPLAY_CONTACTS and viewer:
            displayContactsFromPhase(phase_d,viewer)
        
        
        # %%%%%% simple support : %%%%%%%% 
        phase_s = cs.contact_phases[cs_id + 1]
        # copy previous placement :
        phase_s.RF_patch = phase_d.RF_patch
        phase_s.LF_patch = phase_d.LF_patch
        phase_s.RH_patch = phase_d.RH_patch
        phase_s.LH_patch = phase_d.LH_patch 
        # find the contact to break : 
        variations = fb.getContactsVariations(stateId,stateId+1)
        for var in variations:
            if var == lLegId:
                phase_s.LF_patch.active = False
            if var == rLegId:
                phase_s.RF_patch.active = False
            if var == lArmId:
                phase_s.LH_patch.active = False
            if var == rArmId:
                phase_s.RH_patch.active = False
        # retrieve the COM position for init and final state 
         
        phase_s.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[config_id][:-6])).T)
        if init_guess_for_phase:
            # generate init state : 
            init_state = [0]*9
            init_state[0:3] = c_init_guess(t_init_guess[0]).transpose().tolist()[0]
            init_state[3:6] = dc_init_guess(t_init_guess[0]).transpose().tolist()[0]
            init_state[6:9] = ddc_init_guess(t_init_guess[0]).transpose().tolist()[0]
            init_state = np.matrix(init_state).transpose()            
            # generate final state : 
            final_state = [0]*9
            final_state[0:3] = c_init_guess(t_init_guess[1]+t_init_guess[0]).transpose().tolist()[0]
            final_state[3:6] = dc_init_guess(t_init_guess[1]+t_init_guess[0]).transpose().tolist()[0]
            final_state[6:9] = ddc_init_guess(t_init_guess[1]+t_init_guess[0]).transpose().tolist()[0]
            final_state = np.matrix(final_state).transpose()            
            
            # set timing : 
            phase_s.time_trajectory.append(t_init_guess[1]) 
            time = np.linspace(t_init_guess[0],t_init_guess[1]+t_init_guess[0],num=NUM_NODES_SS,endpoint=False)  
            for t in time:
                state = [0]*9
                state[0:3] = c_init_guess(t).transpose().tolist()[0]
                state[3:6] = dc_init_guess(t).transpose().tolist()[0]
                state[6:9] = ddc_init_guess(t).transpose().tolist()[0]
                u = [0]*6
                u [0:3] = state[6:9]                
                phase_s.control_trajectory.append(np.matrix(u).transpose())
                phase_s.state_trajectory.append(np.matrix(state).transpose()) 
        else :
            init_state = phase_d.init_state.copy()
            final_state = phase_d.final_state.copy()
            fb.setCurrentConfig(configs[config_id+1])
            final_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
            final_state[3:9] = np.matrix(configs[config_id+1][-6:]).transpose()              
            phase_s.time_trajectory.append((fb.getDurationForState(stateId))*(1-DURATION_n_CONTACTS)/SPEED) 
        phase_s.init_state=init_state.copy()
        phase_s.final_state=final_state.copy()
        #print "done for single support"
        if DISPLAY_CONTACTS and viewer:
            displayContactsFromPhase(phase_s,viewer)        
        
    # add the final double support stance : 
    phase_d = cs.contact_phases[n_steps-1]
    fb.setCurrentConfig(configs[n_double_support - 1])
    # compute MRF and MLF : the position of the contacts
    q_rl = fb.getJointPosition(rfoot)
    q_ll = fb.getJointPosition(lfoot)
    q_rh = fb.getJointPosition(rhand)
    q_lh = fb.getJointPosition(lhand)

    # feets
    MRF = SE3.Identity()
    MLF = SE3.Identity()
    MRF.translation = np.matrix(q_rl[0:3]).T
    MLF.translation = np.matrix(q_ll[0:3]).T
    if not FORCE_STRAIGHT_LINE :  
        rot_rl = Quaternion(q_rl[3],q_rl[4],q_rl[5],q_rl[6])
        rot_ll = Quaternion(q_ll[3],q_ll[4],q_ll[5],q_ll[6])
        MRF.rotation = rot_rl.matrix()
        MLF.rotation = rot_ll.matrix()
    # apply the transform ankle -> center of contact
    MRF *= MRsole_offset
    MLF *= MLsole_offset

    # hands
    MRH = SE3()
    MLH = SE3()
    MRH.translation = np.matrix(q_rh[0:3]).T
    MLH.translation = np.matrix(q_lh[0:3]).T
    rot_rh = Quaternion(q_rh[3],q_rh[4],q_rh[5],q_rh[6])
    rot_lh = Quaternion(q_lh[3],q_lh[4],q_lh[5],q_lh[6])
    MRH.rotation = rot_rh.matrix()
    MLH.rotation = rot_lh.matrix()   
    
    MRH *= MRhand_offset
    MLH *= MLhand_offset    
    
    # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement
    phase_d.RF_patch = phase_s.RF_patch
    phase_d.RF_patch.active = fb.isLimbInContact(rLegId,endId)
    phase_d.LF_patch = phase_s.LF_patch
    phase_d.LF_patch.active = fb.isLimbInContact(lLegId,endId)
    phase_d.RH_patch = phase_s.RH_patch
    phase_d.RH_patch.active = fb.isLimbInContact(rArmId,endId)
    phase_d.LH_patch = phase_s.LH_patch
    phase_d.LH_patch.active = fb.isLimbInContact(lArmId,endId)
    
    # now we change the contacts that have moved : 
    variations = fb.getContactsVariations(endId-1,endId)
    #assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
    for var in variations:     
        # FIXME : for loop in variation ? how ?
        if var == lLegId:
            phase_d.LF_patch.placement = MLF
        if var == rLegId:
            phase_d.RF_patch.placement = MRF
        if var == lArmId:
            phase_d.LH_patch.placement = MLH
        if var == rArmId:
            phase_d.RH_patch.placement = MRH
    # retrieve the COM position for init and final state (equal for double support phases)    
    phase_d.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[-1][:-6])).T)
    if init_guess_for_phase:
        # generate init state : 
        init_state = [0]*9
        init_state[0:3] = c_init_guess(t_init_guess[1]+t_init_guess[0]).transpose().tolist()[0]
        init_state[3:6] = dc_init_guess(t_init_guess[1]+t_init_guess[0]).transpose().tolist()[0]
        init_state[6:9] = ddc_init_guess(t_init_guess[1]+t_init_guess[0]).transpose().tolist()[0] 
        init_state = np.matrix(init_state).transpose()
        # generate final state : 
        final_state = [0]*9        
        final_state[0:3] = c_init_guess(t_init_guess[1]+t_init_guess[0]+t_init_guess[2]).transpose().tolist()[0]
        final_state[3:6] = dc_init_guess(t_init_guess[1]+t_init_guess[0]+t_init_guess[2]).transpose().tolist()[0]
        final_state[6:9] = ddc_init_guess(t_init_guess[1]+t_init_guess[0]+t_init_guess[2]).transpose().tolist()[0] 
        final_state = np.matrix(final_state).transpose()
        # set timing :         
        phase_d.time_trajectory.append(t_init_guess[2])
        time = np.linspace(t_init_guess[0]+t_init_guess[1],t_init_guess[2]+t_init_guess[1]+t_init_guess[0],num=NUM_NODES_FINAL,endpoint=True)  
        for t in time:
            t = min(t,c_init_guess.max())
            state = [0]*9
            state[0:3] = c_init_guess(t).transpose().tolist()[0]
            state[3:6] = dc_init_guess(t).transpose().tolist()[0]
            state[6:9] = ddc_init_guess(t).transpose().tolist()[0]
            u = [0]*6
            u [0:3] = state[6:9]            
            phase_d.control_trajectory.append(np.matrix(u).transpose())
            phase_d.state_trajectory.append(np.matrix(state).transpose()) 
    else :
        init_state = phase_d.init_state
        init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        init_state[3:9] = np.matrix(configs[-1][-6:]).transpose()
        final_state = init_state.copy()
        phase_d.time_trajectory.append(0.)
    phase_d.init_state=init_state
    phase_d.final_state=final_state   
    #print "done for last state"
    if DISPLAY_CONTACTS and viewer:
        displayContactsFromPhase(phase_d,viewer)
        
        
        
        
    # assign contact models :
    MContactAction = MRsole_offset.copy()
    t = MContactAction.translation
    t[:2] = 0.
    MContactAction.translation = t  
    for k,phase in enumerate(cs.contact_phases):
        RF_patch = phase.RF_patch
        cm = RF_patch.contactModel
        cm.mu = MU_FOOT
        cm.ZMP_radius = ZMP_RADIUS
        RF_patch.contactModelPlacement = MContactAction
        LF_patch = phase.LF_patch
        cm = LF_patch.contactModel
        cm.mu = MU_FOOT
        cm.ZMP_radius = ZMP_RADIUS
        LF_patch.contactModelPlacement = MContactAction
        LH_patch = phase.LH_patch
        cm = LH_patch.contactModel
        cm.mu = MU_HAND
        cm.ZMP_radius = ZMP_RADIUS
        LH_patch.contactModelPlacement = MContactAction
        RH_patch = phase.RH_patch            
        cm = RH_patch.contactModel
        cm.mu = MU_HAND
        cm.ZMP_radius = ZMP_RADIUS
        RH_patch.contactModelPlacement = MContactAction
                
        
    return cs


def generateContactSequenceWithInitGuess(ps,fb,configs,beginId,endId,viewer=None):
    curves_initGuess = []
    timings_initGuess = []
    for id_state in range(beginId,endId):
        print "id_state = ",str(id_state)
        pid = fb.isDynamicallyReachableFromState(id_state,id_state+1,True,numPointsPerPhases=0)
        if len(pid) != 4:
            print "Cannot compute qp initial guess for state "+str(id_state)
            return generateContactSequence(fb,configs,beginId,endId,viewer)
        c_qp = fb.getPathAsBezier(int(pid[0]))
        t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]
        curves_initGuess.append(c_qp)
        timings_initGuess.append(t_qp)
        
    return generateContactSequence(fb,configs,beginId,endId,viewer, curves_initGuess, timings_initGuess)
    
    
