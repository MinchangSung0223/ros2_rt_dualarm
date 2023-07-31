/*
 * RTIndy7Client.cpp
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */


#ifndef __XENO__
#define __XENO__
#endif
#include "../RTIndy7Client.h"
#include "MR_Indy7.h"
#include "MR_DualArm.h"
#include "MR_Indy7_DualArm.h"
#include "modern_robotics_relative.h"
#include "modern_robotics.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"

#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "Utils/b3Clock.h"
#include "Indy7.h"
#include "DualArm.h"

#include "MR_Trajectory.h"

//
JointInfo right_info;
JointInfo left_info;
JointInfo info;


MR_Indy7 mr_indy7;
MR_Indy7 mr_indy7_l;
MR_Indy7 mr_indy7_r;
MR_DualArm mr_dualarm;
MR_Indy7_DualArm dualarm;
MR_Trajectory traj;
MR_Trajectory task_traj;
DualArm* robot;
b3RobotSimulatorClientAPI_NoDirect sim;

// Xenomai RT tasks
RT_TASK RTIndy7_task;
RT_TASK safety_task;
RT_TASK print_task;
int traj_flag = 0;

//BULLET SIM
extern const int CONTROL_RATE;
const int CONTROL_RATE = 1000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;
int end_flag = 0;
double t=0;
double Tf = 20.0;
// RTIndy7_task
RTIME step;
int leftId;
mr::SE3 Trl;
class JointStatePublisherNode : public rclcpp::Node
{
public:
  JointStatePublisherNode()
      : Node("joint_state_publisher")
  {

    
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	desired_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("desired_joint_states", 10);
    joint_state_timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&JointStatePublisherNode::publish_joint_state, this));
  }

private:
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_publisher_;
  void publish_joint_state()
  {
    //ROS TOPIC
	//left_info.act.q<<0,0,0,0,0,0;
	//right_info.act.q<<0,0,0,0,0,0;
    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
	auto desired_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
	desired_joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = {"l_joint_0", "l_joint_1", "l_joint_2", "l_joint_3", "l_joint_4", "l_joint_5", "r_joint_0", "r_joint_1", "r_joint_2", "r_joint_3", "r_joint_4", "r_joint_5"};
	desired_joint_state_msg->name = {"l_joint_0", "l_joint_1", "l_joint_2", "l_joint_3", "l_joint_4", "l_joint_5", "r_joint_0", "r_joint_1", "r_joint_2", "r_joint_3", "r_joint_4", "r_joint_5"};
    joint_state_msg->position = {left_info.act.q[0], left_info.act.q[1], left_info.act.q[2], 
                                 left_info.act.q[3], left_info.act.q[4], left_info.act.q[5],
                                right_info.act.q[0], right_info.act.q[1], right_info.act.q[2], 
                                right_info.act.q[3], right_info.act.q[4], right_info.act.q[5]};
	joint_state_msg->velocity = {left_info.act.q_dot[0], left_info.act.q_dot[1], left_info.act.q_dot[2], 
                                 left_info.act.q_dot[3], left_info.act.q_dot[4], left_info.act.q_dot[5],
                                right_info.act.q_dot[0], right_info.act.q_dot[1], right_info.act.q_dot[2], 
                                right_info.act.q_dot[3], right_info.act.q_dot[4], right_info.act.q_dot[5]};																							
	desired_joint_state_msg->position = {left_info.des.q[0], left_info.des.q[1], left_info.act.q[2], 
                                 left_info.des.q[3], left_info.des.q[4], left_info.des.q[5],
                                right_info.des.q[0], right_info.des.q[1], right_info.des.q[2], 
                                right_info.des.q[3], right_info.des.q[4], right_info.des.q[5]};							
	desired_joint_state_msg->velocity = {left_info.des.q_dot[0], left_info.des.q_dot[1], left_info.des.q_dot[2], 
                                 left_info.des.q_dot[3], left_info.des.q_dot[4], left_info.des.q_dot[5],
                                right_info.des.q_dot[0], right_info.des.q_dot[1], right_info.des.q_dot[2], 
                                right_info.des.q_dot[3], right_info.des.q_dot[4], right_info.des.q_dot[5]};															
    joint_state_publisher_->publish(*joint_state_msg);
	desired_joint_state_publisher_->publish(*desired_joint_state_msg);
  }


};
void drawTaskTrajectory(class b3RobotSimulatorClientAPI_NoDirect* sim,const vector<SE3> Xd_list,double dt, double Tf){
		for(int i = 0;i<floor(Tf/dt);){
		SE3 Xd =Xd_list.at(i);
		Vector4d p0,px,py,pz;
		vector<Vector4d> p_list;
		p0<<0,0,0,1;
		px<<0.1,0,0,1;
		py<<0,0.1,0,1;
		pz<<0,0,0.1,1;
		p0 = Xd*p0;
		px = Xd*px;
		py = Xd*py;
		pz = Xd*pz;
		p_list.push_back(px);
		p_list.push_back(py);
		p_list.push_back(pz);

		double dfromXYZ[3];
		double dtoXYZ[3];
		for(int j = 0;j<p_list.size();j++){
			Vector4d p = p_list.at(j);
			dfromXYZ[0] = p0(0);
			dfromXYZ[1] = p0(1);
			dfromXYZ[2] = p0(2);

			dtoXYZ[0] = p(0);
			dtoXYZ[1] = p(1);
			dtoXYZ[2] = p(2);
			b3RobotSimulatorAddUserDebugLineArgs args;
			args.m_colorRGB[0] = 0;
			args.m_colorRGB[1] = 0;
			args.m_colorRGB[2] = 0;
			args.m_colorRGB[j] = 1;
			args.m_lineWidth = 2;
			sim->addUserDebugLine(dfromXYZ, dtoXYZ, args);
			usleep(100000/5.0);
		}
		i+=100;
		}
		SE3 Xd = Xd_list.at(task_traj.Xd_list.size()-1);
		Vector4d p0,px,py,pz;
		vector<Vector4d> p_list;
		p0<<0,0,0,1;
		px<<0.1,0,0,1;
		py<<0,0.1,0,1;
		pz<<0,0,0.1,1;
		p0 = Xd*p0;
		px = Xd*px;
		py = Xd*py;
		pz = Xd*pz;
		p_list.push_back(px);
		p_list.push_back(py);
		p_list.push_back(pz);

		double dfromXYZ[3];
		double dtoXYZ[3];
		for(int j = 0;j<p_list.size();j++){
			Vector4d p = p_list.at(j);
			dfromXYZ[0] = p0(0);
			dfromXYZ[1] = p0(1);
			dfromXYZ[2] = p0(2);

			dtoXYZ[0] = p(0);
			dtoXYZ[1] = p(1);
			dtoXYZ[2] = p(2);
			b3RobotSimulatorAddUserDebugLineArgs args;
			args.m_colorRGB[0] = 0;
			args.m_colorRGB[1] = 0;
			args.m_colorRGB[2] = 0;
			args.m_colorRGB[j] = 1;
			args.m_lineWidth = 2;
			sim->addUserDebugLine(dfromXYZ, dtoXYZ, args);
			usleep(100000/5.0);
		}
	
}
void RTIndy7_run(void *arg)
{
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	RTIME now,previous;
	double dt = 0.001;
	now= 0 ;
	previous= 0 ;
	step = 0;
	relmr::JVec eint = relmr::JVec::Zero();
	relmr::JVec q_init;
	q_init<<-0.3531 , -1.1645  ,-1.2237 , -0.788  , -0.723 , 0.0696,0.3531 , 1.1645  ,1.2237 , 0.788  , 0.723 , -0.0696;
	q_init = -q_init;
	sim.setTimeOut(0.0009);
	robot->reset_q(&sim,q_init);
	sim.stepSimulation();
	
	relmr::JVec max_torque;
	max_torque<<1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000; 
	relmr::JVec q_next=relmr::JVec::Random();
	relmr::JVec q_des = relmr::JVec::Zero();
	relmr::JVec qdot_des = relmr::JVec::Zero();
	relmr::JVec qddot_des = relmr::JVec::Zero();			
	Tf = 5;
	dualarm.FKinBody(q_init,relmr::JVec::Zero());
	mr::SE3 X_move = mr::SE3::Identity();
	X_move(2,3) = X_move(2,3)-0.2;
	mr::SE3 newTbl =dualarm.Tbl*X_move;
	mr::SE3 newTbr =dualarm.Tbr*X_move;
	mr::SE3 X_l_des = TransInv(dualarm.Tbl0)* newTbl ;
	mr::SE3 X_r_des = TransInv(dualarm.Tbr0) * newTbr;
	mr::Vector6d V_l_des = mr::Vector6d::Zero();
	mr::Vector6d V_r_des = mr::Vector6d::Zero();
	mr::Vector6d Vdot_l_des = mr::Vector6d::Zero();
	mr::Vector6d Vdot_r_des = mr::Vector6d::Zero();
	mr::Vector6d F_l_des = mr::Vector6d::Zero();
	mr::Vector6d F_r_des = mr::Vector6d::Zero();
	mr::Vector6d Ftip_l = mr::Vector6d::Zero();
	mr::Vector6d Ftip_r = mr::Vector6d::Zero();

	while (t<Tf)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle
		previous = rt_timer_read();
		relmr::JVec q = robot->get_q(&sim);
		relmr::JVec qdot = robot->get_qdot(&sim);
		relmr::JVec q_rel = dualarm.get_q_rel(q);

		relmr::MassMat Mmat = dualarm.MassMatrix(q);	
		relmr::JVec C = dualarm.VelQuadraticForces(q,qdot);
		relmr::JVec G = dualarm.GravityForces(q);		
		mr::MassMat Mmat_r=Mmat.topLeftCorner<6, 6>();
		mr::MassMat Mmat_l=Mmat.bottomRightCorner<6, 6>();
		mr::JVec C_r = C.segment<6>(0);
		mr::JVec C_l = C.segment<6>(6);
		mr::JVec G_r = G.segment<6>(0);
		mr::JVec G_l = G.segment<6>(6);


		dualarm.FKinBody(q,qdot);
		mr::JVec q_r =  q.segment<6>(0);
		mr::JVec q_l =  q.segment<6>(6);
		mr::JVec qdot_r =  qdot.segment<6>(0);
		mr::JVec qdot_l =  qdot.segment<6>(6);
		Ftip_l<<0,0,0,0,0,0;
		Ftip_r<<0,0,0,0,0,0;
		if (t>1 && t<=1.002){
			Ftip_l<<1,1,1,1,1,1; 
			Ftip_r<<1,1,1,1,1,1;
		}
		
		//mr::JVec tau_L = dualarm.L->ImpedanceControl(q_l,qdot_l,Ftip_l,dualarm.T0l,dualarm.Jb_l,dualarm.Jbdot_l,X_l_des,V_l_des,Vdot_l_des, F_l_des,Mmat_r,C_r,G_r);
		//mr::JVec tau_R = dualarm.R->ImpedanceControl(q_r,qdot_r,Ftip_r,dualarm.T0r,dualarm.Jb_r,dualarm.Jbdot_r,X_r_des,V_r_des,Vdot_r_des, F_r_des,Mmat_l,C_l,G_l);




		relmr::JVec tau_list = G;
		//tau_list.segment<6>(0) = tau_R;
		//tau_list.segment<6>(6) = tau_L;
		robot->set_torque(&sim,tau_list,max_torque);
		sim.stepSimulation();
		// Logging
		right_info.act.q = q.segment<6>(0);
		right_info.act.q_dot = qdot.segment<6>(0);
		right_info.des.q = q_des.segment<6>(0);
		right_info.des.q_dot = qdot_des.segment<6>(0);
		right_info.des.q_ddot = qddot_des.segment<6>(0);
		
		left_info.act.q=q.segment<6>(6);
		left_info.act.q_dot = qdot.segment<6>(6);
		left_info.des.q = q_des.segment<6>(6);
		left_info.des.q_dot = qdot_des.segment<6>(6);
		left_info.des.q_ddot = qddot_des.segment<6>(6);
		left_info.act.F(0) = Ftip_l(0);
		left_info.act.F(1) = Ftip_l(1);
		left_info.act.F(2) = Ftip_l(2);
		left_info.act.F(3) = Ftip_l(3);
		left_info.act.F(4) = Ftip_l(4);
		left_info.act.F(5) = Ftip_l(5);
		now = rt_timer_read();
		step = now-previous;
		t+=dt;
	}
	traj.saveJointTrajectory("");

	run =0;
	exit(1);
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	while (run)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		//rt_printf("Ftip L : %.3f \t%.3f \t%.3f \t%.3f \t%.3f \t%.3f \n",left_info.act.F(0),left_info.act.F(1),left_info.act.F(2),left_info.act.F(3),left_info.act.F(4),left_info.act.F(5));
	}
}


/****************************************************************************/
void signal_handler(int signum)
{
	rt_task_delete(&RTIndy7_task);
	rt_task_delete(&safety_task);
	rt_task_delete(&print_task);
	// FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_STOP_DEVICE;
	// FTConfigParamCB[0]=FT_STOP_DEVICE;
	// nrmk_master.writeBuffer(0x70003, FTConfigParam);
	// nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
	// nrmk_master.processRxDomain();

	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	
	run = 0;
	exit(1);
    
}


/****************************************************************************/
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Perform auto-init of rt_print buffers if the task doesn't do so
	std::cout<<"start<"<<std::endl;
    rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns = 1000000; // nanosecond -> 1kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	mr_indy7=MR_Indy7();
    mr_indy7.MRSetup();
	mr_indy7_l=MR_Indy7();
    mr_indy7_l.MRSetup();
	mr_indy7_l.g<<0,8.487,-4.9;
	mr_indy7_r=MR_Indy7();
    mr_indy7_r.MRSetup();
	mr_indy7_r.g<<0,8.487,-4.9;
	mr_dualarm = MR_DualArm();
	mr_dualarm.MRSetup();
	dualarm=MR_Indy7_DualArm();
	dualarm.MRSetup();	
	traj = MR_Trajectory();
	task_traj = MR_Trajectory();
	//---------BULLET SETUP START------------------
	b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	if (!b3CanSubmitCommand(client))
	{
		printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
	}	
    	
	b3RobotSimulatorClientAPI_InternalData data;
	data.m_physicsClientHandle = client;
	data.m_guiHelper = 0;
	sim.setInternalData(&data);
	sim.resetSimulation();
	sim.setGravity( btVector3(0 , 0 ,-9.8));
	int bodyId = sim.loadURDF("model/body.urdf");  
	leftId = sim.loadURDF("model/indy7.urdf");  
	int rightId = sim.loadURDF("model/indy7.urdf");  
	btVector3 left_pos(0,0.1563,0.3772);
	btQuaternion left_orn(-0.5,0,0,0.866);
	btVector3 right_pos(0,-0.1563,0.3772);
	btQuaternion right_orn(0.0,0.5,-0.866,0);    
	sim.resetBasePositionAndOrientation(leftId,left_pos, left_orn);
	sim.resetBasePositionAndOrientation(rightId,right_pos, right_orn);	
    robot= new DualArm(&sim,rightId,leftId);


	//---------BULLET SETUP END-----------------
	
	
	// RTIndy7_task: create and start
	
	printf("Now running rt task ...\n");

	// RTIndy7 control
	rt_task_create(&RTIndy7_task, "RTIndy7_task", 0, 2, 0);
	rt_task_start(&RTIndy7_task, &RTIndy7_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 1, 0);
	rt_task_start(&print_task, &print_run, NULL);
	

	// Must pause here
  	auto node = std::make_shared<JointStatePublisherNode>();
	rclcpp::WallRate loop_rate(1000);
	while (rclcpp::ok()&&run)
	{
	    rclcpp::spin_some(node);
	    loop_rate.sleep();
		
	}
	rclcpp::shutdown();  

	// Finalize
	signal_handler(0);
	
    return 0;
}


