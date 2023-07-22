#ifndef MR_INDY7_DUALARM_H
#define MR_INDY7_DUALARM_H
#include <jsoncpp/json/json.h>
#include "iostream"
#include "MR_Indy7.h"

#include "modern_robotics.h"
#include "modern_robotics_relative.h"

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace mr;
#define RELJOINTNUM 12
class MR_Indy7_DualArm {
public:
    MR_Indy7_DualArm();  // Constructor
    typedef Eigen::Matrix<double, RELJOINTNUM, 1> relJVec;
    typedef Eigen::Matrix<double, 6, RELJOINTNUM> relScrewList;
    MR_Indy7* L;
    MR_Indy7* R;
    unsigned int jointnum;
    relmr::ScrewList Slist;
    relmr::ScrewList Blist;

    mr::SE3 M;
    mr::SE3 T0l;
    mr::SE3 T0r;
    mr::SE3 Tbr;
    mr::SE3 Tbl;
    mr::SE3 Tbr0;
    mr::SE3 Tbl0;


    relJVec q;
    relJVec dq;
    relJVec ddq;


    relJVec q_des;
    relJVec dq_des;
    relJVec ddq_des;

    mr::Vector3d g;
    mr::JVec  torq;

    mr::Matrix6d Kp;
    mr::Matrix6d Kv;
    mr::Matrix6d Ki;

    relmr::Matrixnxn Hinf_Kp;
    relmr::Matrixnxn Hinf_Kv;
    relmr::Matrixnxn Hinf_Ki;
    relmr::Matrixnxn Hinf_K_gamma;

    relmr::Matrixnxn HinfSim_Kp;
    relmr::Matrixnxn HinfSim_Kv;
    relmr::Matrixnxn HinfSim_Ki;
    relmr::Matrixnxn HinfSim_K_gamma;

    void MRSetup();
    void setq(mr::JVec q_l, mr::JVec q_r);
    void setdq(mr::JVec dq_l, mr::JVec dq_r);
    void getq(mr::JVec& q_l, mr::JVec& q_r);
    void getdq(mr::JVec& dq_l, mr::JVec& dq_r);
    relmr::JVec GravityForces(const relmr::JVec q);
    relmr::JVec VelQuadraticForces(const relmr::JVec q,const relmr::JVec qdot);
    relmr::MassMat MassMatrix(const relmr::JVec q);
    relmr::JVec ForwardDynamics(const relmr::JVec q,const relmr::JVec qdot,const relmr::JVec tau,const Vector6d Ftip_r,const Vector6d Ftip_l);
    relmr::JVec HinfControlSim(const relmr::JVec q,const relmr::JVec qdot,const relmr::JVec q_des,const relmr::JVec qdot_des,const relmr::JVec qddot_des,relmr::JVec& eint);
    void FKinBody(const relmr::JVec q);
    relmr::JVec get_q_rel(relmr::JVec q);
    relmr::JVec get_qdot_rel(relmr::JVec qdot);
};

#endif // MR_INDY7_DUALARM_H
