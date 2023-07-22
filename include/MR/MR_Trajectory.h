#ifndef MR_TRAJECTORY_H
#define MR_TRAJECTORY_H

#include "iostream"
#include "modern_robotics.h"
#include "modern_robotics_relative.h"
#include "jsoncpp/json/json.h"
#include <fstream>
//#include "Scurve.h"
#include "ScurveGenerator.h"
#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace mr;
class MR_Trajectory {
public:
    MR_Trajectory();  // Constructor
    vector<SE3> Xd_list;
    vector<Vector6d> Vd_list;
    vector<Vector6d> dVd_list;
    vector<double> t_list;
    vector<JVec> q_list;
    vector<JVec> qdot_list;
    vector<JVec> qddot_list;
    vector<relmr::JVec> q_rel_list;
    vector<relmr::JVec> qdot_rel_list;
    vector<relmr::JVec> qddot_rel_list;    

    double Tf;
    double N;
    double timegap;
    void setJointTrajectory(const mr::JVec& q0,const mr::JVec& qT,const mr::JVec& qdot0,const mr::JVec& qdotT,const mr::JVec& qddot0,const mr::JVec& qddotT,double t0,double Tf,int N);
    void setRelativeJointTrajectory(const relmr::JVec& q0,const relmr::JVec& qT,const relmr::JVec& qdot0,const relmr::JVec& qdotT,const relmr::JVec& qddot0,const relmr::JVec& qddotT,double Tf,double dt,int trajType);
    void addRelativeJointTrajectory(const relmr::JVec& q0,const relmr::JVec& q1,const relmr::JVec& qdot0,const relmr::JVec& qdotT,const relmr::JVec& qddot0,const relmr::JVec& qddotT,double t0,double t1,double dt,int trajType);
    void addScurveRelativeJointTrajectory(const relmr::JVec& q0,const relmr::JVec& q1,const relmr::JVec& qdot0,const relmr::JVec& qdot1,const relmr::JVec& qddot0,const relmr::JVec& qddot1,const relmr::JVec& qdot_max,const relmr::JVec& qddot_max,double t0,double t1,double dt,int trajType);
    void addLieScrewTrajectory(mr::SE3 X0,mr::SE3 X1,mr::Vector6d V0,mr::Vector6d V1,mr::Vector6d Vd0,mr::Vector6d Vd1,double t0,double t1,double dt,int trajType);
    void getJointTrajectory(double now_t ,mr::JVec& q_des,mr::JVec& qdot_des,mr::JVec& qddot_des );
    void getRelativeJointTrajectory(double now_t ,relmr::JVec& q_des,relmr::JVec& qdot_des,relmr::JVec& qddot_des );
    void saveJointTrajectory(const std::string& filename);
};

#endif // MR_TRAJECTORY_H
