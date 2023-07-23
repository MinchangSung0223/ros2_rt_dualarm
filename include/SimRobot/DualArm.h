#ifndef DAULARM_SETUP_H
#define DAULARM_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "modern_robotics.h"
#include "modern_robotics_relative.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include "Indy7.h"

using namespace Eigen;
using namespace std;
class DualArm
{
	Indy7* R;
	Indy7* L;
	int leftId;
	int rightId;

public:
	relmr::JVec HOME;
	relmr::JVec ZERO;
	DualArm(class b3RobotSimulatorClientAPI_NoDirect* sim,int rightId,int leftId);
	relmr::JVec get_q(class b3RobotSimulatorClientAPI_NoDirect* sim);
	relmr::JVec get_qdot(class b3RobotSimulatorClientAPI_NoDirect* sim);
	relmr::JVec get_q_rel(class b3RobotSimulatorClientAPI_NoDirect* sim);
	relmr::JVec get_qdot_rel(class b3RobotSimulatorClientAPI_NoDirect* sim);
	relmr::Vector6d get_FT(class b3RobotSimulatorClientAPI_NoDirect* sim,int flag);
	void apply_FT(class b3RobotSimulatorClientAPI_NoDirect* sim,int flag,relmr::Vector6d Fapply);
	void reset_q(class b3RobotSimulatorClientAPI_NoDirect* sim,relmr::JVec q);
	void set_torque(class b3RobotSimulatorClientAPI_NoDirect* sim, relmr::JVec torque,relmr::JVec  max_torques);
	virtual ~DualArm();

};
#endif  //DAULARM_SETUP_H
