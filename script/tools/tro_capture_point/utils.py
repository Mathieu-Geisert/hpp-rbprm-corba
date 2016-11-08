# -*- coding: utf-8 -*-
"""
Created on Thu Oct  6 14:50:16 2016

@author: adelpret
"""

import numpy as np
from numpy.linalg import norm
from math import sqrt
from pinocchio.utils import zero as mat_zeros
from pinocchio.utils import rand as mat_rand
from pinocchio_inv_dyn.acc_bounds_util import computeVelLimits
from pinocchio_inv_dyn.sot_utils import solveLeastSquare
from pinocchio_inv_dyn.multi_contact.stability_criterion import StabilityCriterion

EPS = 1e-5

class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds);

    def __str__(self):
        res = "";
        for (key,value) in self.__dict__.iteritems():
            if (isinstance(value, np.ndarray) and len(value.shape)==2 and value.shape[0]>value.shape[1]):
                res += " - " + key + ": " + str(value.T) + "\n";
            else:
                res += " - " + key + ": " + str(value) + "\n";
        return res[:-1];

def select_contact_to_make(x_com, dx_com, robot, name_contact_to_break, contacts, contact_candidates, mu, gravity, mass, viewer=None, verb=0):
    ncp = np.sum([np.matrix(c['P']).shape[0] for c in contacts.itervalues()]);
    p = mat_zeros((3,ncp));  # contact points in world frame
    N = mat_zeros((3,ncp));  # contact normals in world frame
    N[2,:] = 1.0;
    stabilityCriterion = StabilityCriterion("Stab_crit", x_com, dx_com, p.T, N.T, mu, gravity, mass, verb=verb);
    c_pred = mat_zeros((3,len(contact_candidates)));
    dc_pred = mat_zeros((3,len(contact_candidates)));
    v_pred = mat_zeros(len(contact_candidates));
    t_pred = mat_zeros(len(contact_candidates));
    
    # compute contact points for all contacts except the one to move
    i = 0;
    for (contact_name, PN) in contacts.iteritems():
        if(contact_name==name_contact_to_break):
            continue;
        oMi = robot.framePosition(robot.model.getFrameId(contact_name));
        Pi = np.matrix(PN['P']).T;
        Ni = np.matrix(PN['N']).T;
        for j in range(Pi.shape[1]):
            p[:,i] = oMi.act(Pi[:,j]);
            N[:,i] = oMi.rotation * Ni[:,j];
            i += 1;
    c_id=0;
    P_cand = np.matrix(contacts[name_contact_to_break]['P']).T;  # contact points of contact to break in local frame
    N_cand = np.matrix(contacts[name_contact_to_break]['N']).T;  # contact normals of contact to break in local frame
    for (c_id,oMi) in enumerate(contact_candidates):
        # compute new position of contact points
        for j in range(Pi.shape[1]):
            p[:,i+j] = oMi.act(P_cand[:,j]);
            N[:,i+j] = oMi.rotation * N_cand[:,j]; 

        if(viewer is not None):
            # update contact points in viewer
            for j in range(p.shape[1]):
                viewer.addSphere("contact_point"+str(j), 0.005, p[:,j], (0.,0.,0.), (1, 1, 1, 1));
                viewer.updateObjectConfigRpy("contact_point"+str(j), p[:,j]);

        # check whether the system can stop with this contacts
        stabilityCriterion.set_contacts(p.T, N.T, mu);
        try:
            stab_res = stabilityCriterion.can_I_stop(x_com, dx_com);
            c_pred[:,c_id] = np.asmatrix(stab_res.c).T;
            dc_pred[:,c_id] = np.asmatrix(stab_res.dc).T;
            v_pred[c_id] = norm(stab_res.dc);
            t_pred[c_id] = stab_res.t;
            if(verb>0):
                print "[select_contact_to_make] Candidate %d, predicted com vel in %.3f s = %.3f"%(c_id, stab_res.t, norm(stab_res.dc));
#                raw_input();
        except Exception as e:
            print "ERROR while computing stability criterion:", e;
        
    best_candidate_ids = np.where(abs(v_pred-np.min(v_pred))<EPS)[0];
    if(best_candidate_ids.shape[0]>1):
        # if multiple contacts result in the same final com velocity (typically that would be 0), 
        # pick the one who does it faster
        best_candidate_id = best_candidate_ids[np.argmin(t_pred[best_candidate_ids])];
    else:
        best_candidate_id = best_candidate_ids[0];

    return Bunch(index=best_candidate_id, c=c_pred[:,best_candidate_id], 
                 dc=dc_pred[:,best_candidate_id], t=t_pred[best_candidate_id,0]);


def select_contact_to_break(x_com, dx_com, robot, mass, contacts, mu, gravity, step_time):
    ''' Select which contact to break
    '''
    ncp = np.sum([np.matrix(c['P']).shape[0] for c in contacts.itervalues()]);
    # assume all contacts have the same number of contact points
    ncp -= np.matrix(contacts.itervalues().next()['P']).shape[0];
    p   = mat_zeros((3,ncp));
    N   = mat_zeros((3,ncp));
    N[2,:] = 1.0;
    stabilityCriterion = StabilityCriterion("Stab_crit", x_com, dx_com, p.T, N.T, mu, gravity, mass);
    t_pred  = mat_zeros(len(contacts));
    t_min  = mat_zeros(len(contacts));
    v_pred  = mat_zeros(len(contacts));
    c_pred  = mat_zeros((3,len(contacts)));
    dc_pred = mat_zeros((3,len(contacts)));
    dc_min = mat_zeros((3,len(contacts)));
    for (contactId, name_of_contact_to_break) in enumerate(contacts):
        # compute the contact points without name_of_contact_to_break
        i = 0;
        for (contact_name, PN) in contacts.iteritems():    
            if(contact_name==name_of_contact_to_break):
                continue;
            fid = robot.model.getFrameId(contact_name);
            oMi = robot.framePosition(fid);
            Pi = np.matrix(PN['P']).T;
            Ni = np.matrix(PN['N']).T;
            for j in range(Pi.shape[1]):
                p[:,i] = oMi.act(Pi[:,j]);
                N[:,i] = oMi.rotation * Ni[:,j];
                i += 1;
        # predict future com state supposing to break name_of_contact_to_break
        stabilityCriterion.set_contacts(p.T, N.T, mu);
        try:
            res = stabilityCriterion.predict_future_state(step_time);
            t_pred[contactId] = res.t;
            t_min[contactId] = res.t_min;
            c_pred[:,contactId] = np.asmatrix(res.c).T;
            dc_pred[:,contactId] = np.asmatrix(res.dc).T;
            dc_min[:,contactId] = np.asmatrix(res.dc_min).T;
            v_pred[contactId] = norm(res.dc);
#            print "Without contact %s robot will be able to maintain the contact for %.3f s"%(name_of_contact_to_break,t);
#            print "  Predicted com state = (", c_t.T, dc_t.T, "), norm(dc)=%.3f"%norm(dc_t);
        except Exception as e:
            print "ERROR while computing stability criterion:", e;
    
    t_pred_sorted = sorted(t_pred.A.squeeze());
    if(t_pred_sorted[-1] > t_pred_sorted[-2]+EPS):
        id_contact_to_break = np.argmax(t_pred);
    else:
        id_contact_to_break = np.argmin(v_pred);
    name_of_contact_to_break = contacts.keys()[id_contact_to_break];
    return Bunch(name=name_of_contact_to_break, c=c_pred[:,id_contact_to_break], 
                 dc=dc_pred[:,id_contact_to_break], t=t_pred[id_contact_to_break,0],
                 t_min=t_min[id_contact_to_break,0], dc_min=dc_min[:,id_contact_to_break]);

''' Solve a QP to compute initial joint velocities that satisfy contact constraints and optionally other
    specified constraints, such as having the capture point inside the convex hull of the contact points.
    @param q Joint configuration
    @param invDynForm An instance of invDynFormUtil
    @param ZERO_INITIAL_ANGULAR_MOMENTUM True if initial state must have zero initial angular momentum
    @param ZERO_INITIAL_VERTICAL_COM_VEL True if initial state must have zero initial vertical com velocity
    @param ENSURE_INITIAL_CAPTURE_POINT_OUT True if initial capture point must be outside the support polygon
    @param ENSURE_INITIAL_CAPTURE_POINT_IN True if initial capture point must be inside the support polygon
    @param MAX_INITIAL_COM_VEL The maximum norm of the initial com velocity
    @param MAX_ITER Maximum number of iterations
    @return (success, v), where success is a boolean flag and v contains the joint velocities.
'''    
def compute_initial_joint_velocities(q, invDynForm, ZERO_INITIAL_ANGULAR_MOMENTUM, ZERO_INITIAL_VERTICAL_COM_VEL,
                                     ENSURE_INITIAL_CAPTURE_POINT_OUT, ENSURE_INITIAL_CAPTURE_POINT_IN, 
                                     MAX_INITIAL_COM_VEL, MAX_ITER):
    nv = invDynForm.nv;
    na = invDynForm.na;
    (B_ch, b_ch) = invDynForm.getSupportPolygon();
    
    k = invDynForm.Jc.shape[0];
    n_in = k+3 if ZERO_INITIAL_ANGULAR_MOMENTUM else k;
    n_in +=  1 if ZERO_INITIAL_VERTICAL_COM_VEL else 0;
    A = np.matrix.copy(invDynForm.J_com);
    A_in = mat_zeros((n_in,nv));
    lb_in = mat_zeros(n_in);
    ub_in = mat_zeros(n_in);
    A_in[:k,:] = invDynForm.Jc;
    if(ZERO_INITIAL_ANGULAR_MOMENTUM):
        A_in[k:k+3,:] = invDynForm.M[3:6,:];
    if(ZERO_INITIAL_VERTICAL_COM_VEL):
        A_in[-1,:] = invDynForm.J_com[2,:];
    # compute joint velocity bounds for viability
    v_lb = mat_zeros(nv)-1e10;
    v_ub = mat_zeros(nv)+1e10;
    for j in range(na):
        (v_lb[6+j], v_ub[6+j]) = computeVelLimits(q[7+j], invDynForm.qMin[7+j], invDynForm.qMax[7+j], invDynForm.dqMax[6+j], invDynForm.MAX_MIN_JOINT_ACC);
    # sample desired capture point position
    p = np.matlib.copy(invDynForm.contact_points);
    cp_ub = np.matrix([np.max(p[:,0]), np.max(p[:,1])]).T;
    cp_lb = np.matrix([np.min(p[:,0]), np.min(p[:,1])]).T;
    dx_com_des = mat_zeros(3);
    
    initial_state_generated = False;
    counter = 0;
    v0 = mat_zeros(3);
    while(not initial_state_generated):
        counter += 1;        
        if(counter > MAX_ITER):
            return (False, v0);
        
        cp_des = cp_lb + np.multiply(mat_rand(2), (cp_ub-cp_lb));
        cp_ineq = np.min(np.dot(B_ch, cp_des) + b_ch);
        if((ENSURE_INITIAL_CAPTURE_POINT_OUT and cp_ineq>=0.0) or (ENSURE_INITIAL_CAPTURE_POINT_IN and cp_ineq<0.0)):
            continue;
        
        # compute com vel corresponding to desired capture point        
        dx_com_des[:2] = sqrt(9.81/invDynForm.x_com[2]) * (cp_des - invDynForm.x_com[:2]);
        if(norm(dx_com_des)>=MAX_INITIAL_COM_VEL):
            continue;

        # solve QP to find joint velocities
        (imode,v0) = solveLeastSquare(A.A, dx_com_des.A, v_lb.A, v_ub.A, A_in.A, lb_in.A, ub_in.A, regularization=1e-4);
        if(imode==0):
            dx_com_0 = np.dot(invDynForm.J_com, v0);
            cp_0     = invDynForm.x_com[:2] + dx_com_0[:2] / sqrt(9.81/invDynForm.x_com[2,0]);
            cp_ineq = np.min(np.dot(B_ch, cp_0) + b_ch);
            
            if((ENSURE_INITIAL_CAPTURE_POINT_OUT and cp_ineq<0.0) or
               (ENSURE_INITIAL_CAPTURE_POINT_IN and cp_ineq>=0.0) or
               (ENSURE_INITIAL_CAPTURE_POINT_OUT==False and ENSURE_INITIAL_CAPTURE_POINT_IN==False)):
                initial_state_generated = True;
        
    return (True, v0);

''' Solve a QP to compute initial joint velocities that satisfy contact constraints and optionally other
    specified constraints, such as multi-contact stability/unstability. Stability is computed using the 
    multi-contact stability criterion, as implemented in the function can_I_stop.
    @param q Joint configuration
    @param invDynForm An instance of invDynFormUtil
    @param mu Contact friction coefficient
    @param ZERO_INITIAL_ANGULAR_MOMENTUM True if initial state must have zero initial angular momentum
    @param ZERO_INITIAL_VERTICAL_COM_VEL True if initial state must have zero initial vertical com velocity
    @param ENSURE_STABILITY True if initial state must be stable
    @param ENSURE_UNSTABILITY True if initial state must be unstable
    @param MAX_INITIAL_COM_VEL The maximum norm of the initial com velocity
    @param MAX_ITER Maximum number of iterations
    @return (success, v), where success is a boolean flag and v contains the joint velocities.
    @note It assumes that the initial com position allows for static equilibrium. If this is not
          the case it returns False.
'''       
def compute_initial_joint_velocities_multi_contact(q, invDynForm, mu, ZERO_INITIAL_ANGULAR_MOMENTUM, 
                                                   ZERO_INITIAL_VERTICAL_COM_VEL,
                                                   ENSURE_STABILITY, ENSURE_UNSTABILITY, 
                                                   MAX_INITIAL_COM_VEL, MAX_ITER):
    nv = invDynForm.nv;
    na = invDynForm.na;
    
    k = invDynForm.Jc.shape[0];
    n_in = k+3 if ZERO_INITIAL_ANGULAR_MOMENTUM else k;
    n_in +=  1 if ZERO_INITIAL_VERTICAL_COM_VEL else 0;
    A = np.matrix.copy(invDynForm.J_com);
    A_in = mat_zeros((n_in,nv));
    lb_in = mat_zeros(n_in);
    ub_in = mat_zeros(n_in);
    A_in[:k,:] = invDynForm.Jc;
    if(ZERO_INITIAL_ANGULAR_MOMENTUM):
        A_in[k:k+3,:] = invDynForm.M[3:6,:];
    if(ZERO_INITIAL_VERTICAL_COM_VEL):
        A_in[-1,:] = invDynForm.J_com[2,:];
    # compute joint velocity bounds for viability
    v_lb = mat_zeros(nv)-1e10;
    v_ub = mat_zeros(nv)+1e10;
    for j in range(na):
        (v_lb[6+j], v_ub[6+j]) = computeVelLimits(q[7+j], invDynForm.qMin[7+j], invDynForm.qMax[7+j], invDynForm.dqMax[6+j], invDynForm.MAX_MIN_JOINT_ACC);
    # sample desired capture point position
    P = np.matlib.copy(invDynForm.contact_points);
    N = np.matlib.copy(invDynForm.contact_normals);
    dx_com_des = mat_zeros(3);
    
    initial_state_generated = False;
    counter = 0;
    v0 = mat_zeros(3);

    stab_criterion = StabilityCriterion("default", invDynForm.x_com, dx_com_des, P.T, N.T, mu, np.array([0,0,-9.81]), invDynForm.M[0,0])    
    try:
        # check initial state is in static equilibrium
        res = stab_criterion.can_I_stop();
        if(not res.is_stable):
            print "ERROR: initial configuration is not in static equilibrium", res.c, res.dc;
            return (False, v0);
    except Exception as e:
        print "[compute_initial_joint_velocities_multi_contact] Error while testing static equilibrium. %s"%str(e);
        
    while(not initial_state_generated):
        counter += 1;        
        if(counter > MAX_ITER):
            return (False, v0);
        
        dx_com_des = MAX_INITIAL_COM_VEL * mat_rand(3) / sqrt(3.0);
        if(ENSURE_STABILITY or ENSURE_UNSTABILITY):
            try:
                res = stab_criterion.can_I_stop(invDynForm.x_com, dx_com_des);
                if((ENSURE_STABILITY and not res.is_stable) or (ENSURE_UNSTABILITY and res.is_stable)):
                    continue;
            except Exception as e:
                print "[compute_initial_joint_velocities_multi_contact] Error while checking stability of desired com velocity. %s"%str(e);
                continue;
        
        # solve QP to find joint velocities
        (imode,v0) = solveLeastSquare(A.A, dx_com_des.A, v_lb.A, v_ub.A, A_in.A, lb_in.A, ub_in.A, regularization=1e-4);
        if(imode==0):
            dx_com_0 = np.dot(invDynForm.J_com, v0);
            try:
                res = stab_criterion.can_I_stop(invDynForm.x_com, dx_com_0);
                if((ENSURE_UNSTABILITY and not res.is_stable) or (ENSURE_STABILITY and res.is_stable) or
                   (not ENSURE_STABILITY and not ENSURE_UNSTABILITY)):
                    initial_state_generated = True;
            except Exception as e:
                print "[compute_initial_joint_velocities_multi_contact] Error while checking stability of com velocity. %s"%str(e);
        
    return (True, v0);
