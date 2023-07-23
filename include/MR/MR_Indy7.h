#ifndef MR_INDY7_H
#define MR_INDY7_H

#include "jsoncpp/json/json.h"
#include "iostream"
#include "modern_robotics.h"

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace mr;
class MR_Indy7 {
public:
    MR_Indy7();  // Constructor
    mr::ScrewList Slist;
    mr::ScrewList Blist;
    mr::SE3 M;

	vector<mr::Matrix6d> Glist;	
	vector<mr::SE3> Mlist;	
    mr::JVec q;
    mr::JVec dq;
    mr::JVec ddq;

    mr::JVec q_des;
    mr::JVec dq_des;
    mr::JVec ddq_des;
    

    mr::Vector3d g;
    mr::JVec  torq;

    mr::Matrix6d Kp;
    mr::Matrix6d Kv;
    mr::Matrix6d Ki;

    mr::Matrix6d Hinf_Kp;
    mr::Matrix6d Hinf_Kv;
    mr::Matrix6d Hinf_Ki;
    mr::Matrix6d Hinf_K_gamma;
    mr::Matrix6d Imp_A;
    mr::Matrix6d invImp_A;
    mr::Matrix6d Imp_D;
    mr::Matrix6d Imp_K;
    

    void MRSetup();

    mr::Matrix6xn Mmat(JVec q);
    mr::JVec Cvec(JVec q, JVec dq);
    mr::JVec Gvec(JVec q);
    mr::Jacobian J_s(JVec q);
    mr::Jacobian J_b(JVec q);
    Jacobian Jdot_b(mr::Jacobian J_b,JVec qdot);
    mr::SE3 T_s(JVec q);
    mr::SE3 T_b(JVec q);

    mr::JVec Gravity( JVec q);
    mr::JVec ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des);
    void saturationMaxTorque(JVec &torque, JVec MAX_TORQUES);
    mr::JVec ComputedTorquePIDControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec& eint);
    mr::JVec HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec& eint);
    JVec ImpedanceControl( JVec q,JVec qdot,Vector6d Ftip, SE3 X, Jacobian Jb, Jacobian Jbdot,SE3 X_des, JVec V_des,JVec Vdot_des,Vector6d F_des);
    JVec VelQuadraticForces(const  JVec q,JVec dq);
    MassMat MassMatrix(const  JVec q);
    JVec ForwardDynamics(const JVec q,const JVec qdot,const JVec tau,const Vector6d Ftip);
    
    
};

#endif // MR_INDY7_H
