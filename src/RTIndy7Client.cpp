/*
 * RTIndy7Client.cpp
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */


#ifndef __XENO__
#define __XENO__
#endif
#include "RTIndy7Client.h"
#include "MR_Indy7.h"
#include "MR_DualArm.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"
//
JointInfo right_info;
JointInfo left_info;
JointInfo info;


MR_Indy7 mr_indy7;
MR_Indy7 mr_indy7_l;
MR_Indy7 mr_indy7_r;
MR_DualArm mr_dualarm;

// Xenomai RT tasks
RT_TASK RTIndy7_task;
RT_TASK safety_task;
RT_TASK print_task;
int traj_flag = 0;
class JointStatePublisherNode : public rclcpp::Node
{
public:
  JointStatePublisherNode()
      : Node("joint_state_publisher")
  {

    
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_state_timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&JointStatePublisherNode::publish_joint_state, this));
  }

private:
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  void publish_joint_state()
  {
    //ROS TOPIC

    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = {"l_joint_0", "l_joint_1", "l_joint_2", "l_joint_3", "l_joint_4", "l_joint_5", "r_joint_0", "r_joint_1", "r_joint_2", "r_joint_3", "r_joint_4", "r_joint_5"};
    joint_state_msg->position = {left_info.act.q[0], left_info.act.q[1], left_info.act.q[2], 
                                 left_info.act.q[3], left_info.act.q[4], left_info.act.q[5],
                                right_info.act.q[0], right_info.act.q[1], right_info.act.q[2], 
                                right_info.act.q[3], right_info.act.q[4], right_info.act.q[5]};
    joint_state_publisher_->publish(*joint_state_msg);
  }


};


//For Trajectory management
//Task
JVec eint_r;
JVec e_r ;
JVec eint_l ;
JVec e_l ;
//////////////////////////////////////////////////////////////////
#ifdef __CASADI__
	int indy7_G()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_G.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_G.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "generalized_gravity");
	    if (dlerror()) {
	        printf("Failed to retrieve \"generalized_gravity\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors dlrj
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {0.0, 0.7, 0.0, 0.0, 0.0, 0.0};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[6];
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();

	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < sz_res; ++i) {
	    //     printf("%g ", output_values[i]);
	    // }
	    // printf("\n");
	    rt_printf("[cs]computation time for \"G\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    ////mr_indy7.Gvec(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"G\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return 0;
	}
	int indy7_M()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_M.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_M.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "M");
	    if (dlerror()) {
	        printf("Failed to retrieve \"M\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();
	    
	    
	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < sz_res; ++i) {
	    //     for (casadi_int j = 0; j < sz_res; ++j) {
	    //         printf("%g ", output_values[i * sz_res + j]);
	    //     }
	    //     printf("\n");
	    // }
	    rt_printf("[cs]computation time for \"M\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    ////mr_indy7.Mmat(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"M\": %lius\n", (end-start)/1000);


	    // Free the handle
	    dlclose(handle);

	    return 0;
	}

	int indy7_C()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_C.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_C.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "coriolis");
	    if (dlerror()) {
	        printf("Failed to retrieve \"C\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_pos[] = {0.0, 0.7, 0.0, 0.0, 0.0, 0.0};
	    double input_vel[] = {0.0, 0.7, 0.0, 0.0, 0.0, 0.0};

	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_pos[i];
	    }
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i+6] = &input_vel[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();

	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < sz_res; ++i) {
	    //     for (casadi_int j = 0; j < sz_res; ++j) {
	    //         printf("%g ", output_values[i * sz_res + j]);
	    //     }
	    //     printf("\n");
	    // }
		rt_printf("[cs]computation time for \"C\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    //mr_indy7.Cvec(info.act.q, info.act.q_dot);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"C\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return 0;
	}

	int indy7_FK()
	{
	    RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_fk.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_fk.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "fk_T");
	    if (dlerror()) {
	        printf("Failed to retrieve \"fk_T\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();
	    
	    
	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < sz_res; ++i) {
	    //     for (casadi_int j = 0; j < sz_res; ++j) {
	    //         printf("%g ", output_values[i * sz_res + j]);
	    //     }
	    //     printf("\n");
	    // }
	    rt_printf("[cs]computation time for \"FK\": %lius\n", (end-start)/1000);
	    start = rt_timer_read();
	    //mr_indy7.T_s(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"FK\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return 0;
	}

	int indy7_J_b()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_J_b.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_J_b.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "J_b");
	    if (dlerror()) {
	        printf("Failed to retrieve \"J_b\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();
	    
	    
	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < sz_res; ++i) {
	    //     for (casadi_int j = 0; j < sz_res; ++j) {
	    //         printf("%g ", output_values[i * sz_res + j]);
	    //     }
	    //     printf("\n");
	    // }
	    rt_printf("[cs]computation time for \"J_b\": %lius\n", (end-start)/1000);

	    start = rt_timer_read();
	    //mr_indy7.J_b(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"J_b\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return 0;
	}
	int indy7_J_s()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_J_s.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_J_s.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "J_s");
	    if (dlerror()) {
	        printf("Failed to retrieve \"J_s\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();
	    
	    
	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < sz_res; ++i) {
	    //     for (casadi_int j = 0; j < sz_res; ++j) {
	    //         printf("%g ", output_values[i * sz_res + j]);
	    //     }
	    //     printf("\n");
	    // }
	    rt_printf("[cs]computation time for \"J_s\": %lius\n", (end-start)/1000);

	    start = rt_timer_read();
	    //mr_indy7.J_s(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"J_s\": %lius\n", (end-start)/1000);


	    // Free the handle
	    dlclose(handle);

	    return 0;
	}
#endif
//////////////////////////////////////////////////////////////////

void signal_handler(int signum);

void saveLogData(){}

int initAxes()
{
	for (int i = 0; i < NUM_AXIS; i++)
	{	
		Axis[i].setGearRatio(gearRatio[i]);
		Axis[i].setGearEfficiency(EFFICIENCY);
		Axis[i].setPulsePerRevolution(ENC_CORE);
		Axis[i].setTauADC(TauADC[i]);
		Axis[i].setTauK(TauK[i]);
		Axis[i].setZeroPos(zeroPos[i]);

		Axis[i].setDirQ(dirQ[i]);
		Axis[i].setDirTau(dirTau[i]);

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);
		
		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);
	}
	
	return 1;
}
/****************************************************************************/
void setTraj(JVec& q0_r, JVec& qT_r,JVec& q0_l, JVec& qT_l, double Tf, double gt, int method){
	if(traj_flag ==0){
		q0_r = right_info.act.q;
		q0_l = left_info.act.q;
		traj_flag =1;
	}
	if(traj_flag == 1){
		if (gt <=Tf){
			JointTrajectory(q0_r, qT_r, Tf, gt , method , right_info.des.q, right_info.des.q_dot, right_info.des.q_ddot) ;
			JointTrajectory(q0_l, qT_l, Tf, gt , method , left_info.des.q, left_info.des.q_dot, left_info.des.q_ddot) ;
		}else if(gt > Tf){
			//q0_r = right_info.act.q;
			//q0_l = left_info.act.q;
			right_info.des.q = qT_r;
			left_info.des.q = qT_l;
			right_info.des.q_dot = JVec::Zero();
			left_info.des.q_dot = JVec::Zero();
			right_info.des.q_ddot = JVec::Zero();
			left_info.des.q_ddot = JVec::Zero();
		}
		
	}

}
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
	    	info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 2:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 3:
	    	info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 4:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;
	    	traj_time = 3.0;
	    	motion=1;
	    	break;
	    // default:
	    // 	motion=1;
	    // 	break;
	    }
	}

	for(int i=0;i<NUM_AXIS;i++)
	{
		if(!Axis[i].trajInitialized())
		{
			Axis[i].setTrajInitialQuintic();
			Axis[i].setTarPosInRad(info.q_target(i));
			Axis[i].setTarVelInRad(0);
			Axis[i].setTrajTargetQuintic(traj_time);
		}

		Axis[i].TrajQuintic();

		info.des.q(i)=Axis[i].getDesPosInRad();
		info.des.q_dot(i)=Axis[i].getDesVelInRad();
		info.des.q_ddot(i)=Axis[i].getDesAccInRad();
	}
}


int compute()
{
		
	return 0;
}
void readEcatData(){
	// Drive
	nrmk_master.readBuffer(0x60410, StatusWord);
	nrmk_master.readBuffer(0x60640, ActualPos);
	nrmk_master.readBuffer(0x606c0, ActualVel);
	nrmk_master.readBuffer(0x60770, ActualTor);
	nrmk_master.readBuffer(0x60610, ModeOfOperationDisplay);
	// IO Module
	// [ToDo] 0x61001~0x610025 addition iteratively
	nrmk_master.readBuffer(0x61001, StatusCode);
	nrmk_master.readBuffer(0x61002, DI5V);
	nrmk_master.readBuffer(0x61003, DI1);
	nrmk_master.readBuffer(0x61004, DI2);
	nrmk_master.readBuffer(0x61005, AI1);
	nrmk_master.readBuffer(0x61006, AI2);
	nrmk_master.readBuffer(0x61007, FTRawFxCB);
	nrmk_master.readBuffer(0x61008, FTRawFyCB);
	nrmk_master.readBuffer(0x61009, FTRawFzCB);
	nrmk_master.readBuffer(0x610010, FTRawTxCB);
	nrmk_master.readBuffer(0x610011, FTRawTyCB);
	nrmk_master.readBuffer(0x610012, FTRawTzCB);
	nrmk_master.readBuffer(0x610013, FTOverloadStatusCB);
	nrmk_master.readBuffer(0x610014, FTErrorFlagCB);
	nrmk_master.readBuffer(0x610015, RS485RxCnt);
	nrmk_master.readBuffer(0x610016, RS485RxD0);
	nrmk_master.readBuffer(0x610017, RS485RxD1);
	nrmk_master.readBuffer(0x610018, RS485RxD2);
	nrmk_master.readBuffer(0x610019, RS485RxD3);
	nrmk_master.readBuffer(0x610020, RS485RxD4);
	nrmk_master.readBuffer(0x610021, RS485RxD5);
	nrmk_master.readBuffer(0x610022, RS485RxD6);
	nrmk_master.readBuffer(0x610023, RS485RxD7);
	nrmk_master.readBuffer(0x610024, RS485RxD8);
	nrmk_master.readBuffer(0x610025, RS485RxD9);
	// Tool
	// [ToDo] 0x61001~0x610025 addition iteratively
	nrmk_master.readBuffer(0x60001, IStatus);
	nrmk_master.readBuffer(0x60002, IButton);
	nrmk_master.readBuffer(0x60003, FTRawFx);
	nrmk_master.readBuffer(0x60004, FTRawFy);
	nrmk_master.readBuffer(0x60005, FTRawFz);
	nrmk_master.readBuffer(0x60006, FTRawTx);
	nrmk_master.readBuffer(0x60007, FTRawTy);
	nrmk_master.readBuffer(0x60008, FTRawTz);
	nrmk_master.readBuffer(0x60009, FTOverloadStatus);
	nrmk_master.readBuffer(0x600010, FTErrorFlag);	
	
	int i_ = 0;
	for(int i=RIGHT_AXIS_START_NUM; i<=RIGHT_AXIS_END_NUM;i++)
	{
		Axis[i_].setCurrentPosInCnt(ActualPos[i]);
		Axis[i_].setCurrentVelInCnt(ActualVel[i]);
		Axis[i_].setCurrentTorInCnt(ActualTor[i]);
		
		Axis[i_].setCurrentTime(gt);

		right_info.act.q(i_) = Axis[i_].getCurrPosInRad();
		right_info.act.q_dot(i_) = Axis[i_].getCurrVelInRad();
		right_info.act.tau(i_) = Axis[i_].getCurrTorInNm();

		i_++;
	}
	for(int i=LEFT_AXIS_START_NUM; i<=LEFT_AXIS_END_NUM;i++)
	{
		Axis[i_].setCurrentPosInCnt(ActualPos[i]);
		Axis[i_].setCurrentVelInCnt(ActualVel[i]);
		Axis[i_].setCurrentTorInCnt(ActualTor[i]);
		
		Axis[i_].setCurrentTime(gt);

		left_info.act.q(i_-6) = Axis[i_].getCurrPosInRad();
		left_info.act.q_dot(i_-6) = Axis[i_].getCurrVelInRad();
		left_info.act.tau(i_-6) = Axis[i_].getCurrTorInNm();
		i_++;
	}
	// Update RFT data
	info.act.F(0) = FTRawFx[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(1) = FTRawFy[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(2) = FTRawFz[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(3) = FTRawTx[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(4) = FTRawTy[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(5) = FTRawTz[NUM_IO_MODULE+NUM_AXIS] / torque_divider;

	info.act.F_CB(0) = FTRawFxCB[0] / force_divider;
	info.act.F_CB(1) = FTRawFyCB[0] / force_divider;
	info.act.F_CB(2) = FTRawFzCB[0] / force_divider;
	info.act.F_CB(3) = FTRawTxCB[0] / torque_divider;
	info.act.F_CB(4) = FTRawTyCB[0] / torque_divider;
	info.act.F_CB(5) = FTRawTzCB[0] / torque_divider;

	// info.act.F<<(double)FTRawFx[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawFy[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawFz[NUM_IO_MODULE+NUM_AXIS]
	//           <<(double)FTRawTx[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawTy[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawTz[NUM_IO_MODULE+NUM_AXIS];
}

void writeEcatData(){
	// for(int i=0;i<NUM_AXIS;i++){
	// 	Axis[i].setDesTorInNm(info.des.tau(i));
	// 	TargetTor[i+NUM_IO_MODULE]=Axis[i].getDesTorInCnt();
	// }

	int i_=0;
	for(int i=RIGHT_AXIS_START_NUM; i<=RIGHT_AXIS_END_NUM;i++)
	{
		Axis[i_].setDesTorInNm(right_info.des.tau(i_));
		TargetTor[i]=Axis[i_].getDesTorInCnt();
		i_++;
	}
	for(int i=LEFT_AXIS_START_NUM; i<=LEFT_AXIS_END_NUM;i++)
	{
		Axis[i_].setDesTorInNm(left_info.des.tau(i_-6));
		TargetTor[i]=Axis[i_].getDesTorInCnt();
		i_++;
	}
	// TO DO: write data to actuators in EtherCAT system interface
	nrmk_master.writeBuffer(0x60710, TargetTor);
	// nrmk_master.writeBuffer(0x60600, ModeOfOperation);
}

// RTIndy7_task
void RTIndy7_run(void *arg)
{
	RTIME beginCycle, endCycle;
	RTIME beginRead, beginReadbuf, beginWrite, beginWritebuf, beginCompute;

	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	// nrmk_master.syncEcatMaster();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	info.des.q = JVec::Zero();
	info.des.q_dot = JVec::Zero();
	info.des.q_ddot = JVec::Zero();
	info.des.F = Vector6f::Zero();
	info.des.F_CB = Vector6f::Zero();

	right_info.des.q = JVec::Zero();
	right_info.des.q_dot = JVec::Zero();
	right_info.des.q_ddot = JVec::Zero();
	left_info.des.q = JVec::Zero();
	left_info.des.q_dot = JVec::Zero();
	left_info.des.q_ddot = JVec::Zero();
	 eint_r = JVec::Zero();
	 e_r = JVec::Zero();
	 eint_l = JVec::Zero();
	 e_l = JVec::Zero();
	int ft_init_cnt = 0;


	JVec q0_r =right_info.act.q;
	JVec q0_l =left_info.act.q;

	JVec qT_r =JVec::Zero();
	qT_r<<0.14558084, 0.813371 ,1.3075236,0.614656,1.03065,-0.0693011;
	JVec qT_l =JVec::Zero();
	qT_l<<-0.14558084, -0.813371 ,-1.3075236,-0.614656,-1.03065,0.0693011;
	while (run)
	{
		beginCycle = rt_timer_read();
		periodEcat = 0;
		periodBuffer = 0;

		beginRead = rt_timer_read();
		nrmk_master.processTxDomain();
		periodEcat += (unsigned long) rt_timer_read() - beginRead;
		
		// Read data in EtherCAT Buffer
		beginReadbuf = rt_timer_read();
		readEcatData();	
		periodBuffer += (unsigned long) rt_timer_read() - beginReadbuf;
		for(int i=0;i<NUM_AXIS;i++)
			{
				if(Axis[i].isLimitReached())
				{
					for(int i=0;i<NUM_AXIS;i++)
						nrmk_master.setServoOff(i+NUM_IO_MODULE);
					rt_printf("Servo Off!!\n");
					break;
				}
			}
		beginCompute = rt_timer_read();
		if(system_ready){
			// Trajectory Generation
			//trajectory_generation();
			setTraj(q0_r, qT_r, q0_l, qT_l,5, gt, 5);
			//[ToDo] Add MPC Function 
			compute();	

			// Calculate Joint controller
			//info.des.tau = //mr_indy7.Gravity( info.act.q ); // calcTorque
			// info.des.tau = //mr_indy7.ComputedTorqueControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot); // calcTorque
			e_r = right_info.des.q-right_info.act.q;
			eint_r = eint_r+e_r*0.001;
			e_l = left_info.des.q-left_info.act.q;
			eint_l = eint_l+e_l*0.001;
			
			JVec tauVec_r =mr_indy7_r.Gravity( right_info.act.q );
			JVec tauVec_l =mr_indy7_l.Gravity( left_info.act.q );
			
			//right_info.des.tau = tauVec_r;
			//left_info.des.tau = tauVec_l;
			right_info.des.tau = mr_indy7_r.HinfControl( right_info.act.q , right_info.act.q_dot, right_info.des.q, right_info.des.q_dot,right_info.des.q_ddot,eint_r);
			left_info.des.tau = mr_indy7_l.HinfControl( left_info.act.q , left_info.act.q_dot, left_info.des.q, left_info.des.q_dot,left_info.des.q_ddot,eint_l);
			//right_info.des.tau =tauVec_r;
			//left_info.des.tau =tauVec_r;			
			// //mr_indy7.saturationMaxTorque(info.des.tau,MAX_TORQUES);
		
		}
		else
		{
			
			//info.des.tau = //mr_indy7.Gravity( info.act.q ); // calcTorque
		}
		periodCompute  = (unsigned long) rt_timer_read() - beginCompute;
		
		// Write data in EtherCAT Buffer
		beginWritebuf = rt_timer_read();
		writeEcatData();
		periodBuffer += (unsigned long) rt_timer_read() - beginWritebuf;

		beginWrite = rt_timer_read();
		nrmk_master.processRxDomain();
		periodEcat += (unsigned long) rt_timer_read() - beginWrite;

		endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;
		
		if (nrmk_master.isSystemReady())
		{	
			if(ft_init_cnt==0)
			{
				// Set bias
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_SET_BIAS;
				FTConfigParamCB[0]=FT_SET_BIAS;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==1)
			{
				// Set Filter 100Hz
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_SET_FILTER_50;
				FTConfigParamCB[0]=FT_SET_FILTER_50;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==2)
			{
				// Start
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_START_DEVICE;
				FTConfigParamCB[0]=FT_START_DEVICE;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else
				system_ready=1;	//all drives have been done

			gt+= period;
			
			if (periodEcat > worstEcat)	worstEcat = periodEcat;
			if (periodBuffer > worstBuffer)	worstBuffer = periodBuffer;
			if (periodCompute > worstCompute) worstCompute = periodCompute;
			if (periodCycle > cycle_ns) overruns++;
		}
		rt_task_wait_period(NULL); 	//wait for next cycle
	}
}

// Safety task
void safety_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NUM_AXIS]={0,};

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		
		if (system_ready)
		{
			for(int i=0;i<NUM_AXIS;i++)
			{
				if(Axis[i].isLimitReached())
				{
					for(int i=0;i<NUM_AXIS;i++)
						nrmk_master.setServoOff(i+NUM_IO_MODULE);
					rt_printf("Servo Off!!\n");
					break;
				}
			}
		}
	}
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NUM_AXIS]={0,};
	
	rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		if (++count==10)
		{
			++stick;
			count=0;
		}
		
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;
			if (!nrmk_master.getMasterStatus(NumSlaves, masterState))
				rt_printf("Master: Offline\n");
			if (!nrmk_master.getRxDomainStatus())
				rt_printf("RxDomain: Offline\n");
			if (!nrmk_master.getTxDomainStatus())
				rt_printf("TxDomain: Offline\n");
			for(int i=1;i<=NUM_AXIS;i++){
				if (!nrmk_master.getAxisEcatStatus(i,slaveState[i-1]))
				rt_printf("idx: %u, slaveState: %u",i,slaveState[i-1]);	
			}
			
			rt_printf("Time=%0.3lfs, cycle_dt=%lius,  overrun=%d\n", gt, periodCycle/1000, overruns);
			rt_printf("compute_dt= %lius, worst_dt= %lius, buffer_dt=%lius, ethercat_dt= %lius\n", periodCompute/1000, worstCompute/1000, periodBuffer/1000, periodEcat/1000);

			// for(int j=0; j<NUM_AXIS; ++j){
			// 	rt_printf("ID: %d", j);
			// // 	//rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
			// // 	//rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			// //     //rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
			// // 	//rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
			// 	rt_printf("\t ActPos: %lf, ActVel: %lf \n",info.act.q(j), info.act.q_dot(j));
			// 	//rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
			// // 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.q_dot[j]-info.act.q_ddot[j]);
			// 	// rt_printf("\t TarTor: %f, ",				TargetTorq[j]);
			// 	rt_printf("\t TarTor: %f, ActTor: %lf,\n", info.des.tau(j), info.act.tau(j));
			// }
			for(int j=0; j<6; ++j){
				rt_printf("R-ID: %d", j);
			// 	//rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
			// 	//rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			//     //rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
			// 	//rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
				//rt_printf("\t ActPos: %lf, ActVel: %lf \n",right_info.act.q(j), right_info.act.q_dot(j));
				rt_printf("\t error: %lf \n",e_r(j));
				
				//rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
			// 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.q_dot[j]-info.act.q_ddot[j]);
				// rt_printf("\t TarTor: %f, ",				TargetTorq[j]);
				//rt_printf("\t TarTor: %f, ActTor: %lf , \n", right_info.des.tau(j), right_info.act.tau(j));
				//rt_printf("\t mr_indy7_r.g %f %f %f ",mr_indy7_r.g(0),mr_indy7_r.g(1),mr_indy7_r.g(2));
			}
			for(int j=0; j<6; ++j){
				rt_printf("L-ID: %d", j);
			// 	//rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
			// 	//rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			//     //rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
			// 	//rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
				//rt_printf("\t ActPos: %lf, ActVel: %lf \n",left_info.act.q(j), left_info.act.q_dot(j));
				//rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
			// 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.q_dot[j]-info.act.q_ddot[j]);
				// rt_printf("\t TarTor: %f, ",				TargetTorq[j]);
				rt_printf("\t error: %lf \n",e_l(j));
				//rt_printf("\t TarTor: %f, ActTor: %lf,  \n", left_info.des.tau(j), left_info.act.tau(j));
				//rt_printf("\t mr_indy7_l.g %f %f %f ",mr_indy7_l.g(0),mr_indy7_l.g(1),mr_indy7_l.g(2));
			}			
			//rt_printf("ReadFT: %f, %f, %f, %f, %f, %f\n", info.act.F(0),info.act.F(1),info.act.F(2),info.act.F(3),info.act.F(4),info.act.F(5));
			//rt_printf("ReadFT_CB: %f, %f, %f, %f, %f, %f\n", info.act.F_CB(0),info.act.F_CB(1),info.act.F_CB(2),info.act.F_CB(3),info.act.F_CB(4),info.act.F_CB(5));
			//rt_printf("overload: %u, error: %u\n", FTOverloadStatus[NUM_IO_MODULE+NUM_AXIS], FTErrorFlag[NUM_IO_MODULE+NUM_AXIS]);

#ifdef __CASADI__
			indy7_M();
		    indy7_C();
		    indy7_G();
		    indy7_J_b();
		    indy7_J_s();
            indy7_FK();
#endif

			rt_printf("\n");
		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}
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
    
    nrmk_master.deinit();
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
	//MAX_TORQUES<<MAX_TORQUE_1,MAX_TORQUE_2,MAX_TORQUE_3,MAX_TORQUE_4,MAX_TORQUE_5,MAX_TORQUE_6;
	// For CST (cyclic synchronous torque) control
	if (nrmk_master.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
	{
		printf("System Initialization Failed\n");
	    return 0;
	}
	std::cout<<"start<2"<<std::endl;	
	for (int i = 0; i < NUM_AXIS; ++i)
		ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;
	// For trajectory interpolation
	initAxes();
	for(int i=0;i<NUM_SLAVES;i++)
		nrmk_master.setServoOn(i);
	
	// RTIndy7_task: create and start
	printf("Now running rt task ...\n");

	// RTIndy7 control
	rt_task_create(&RTIndy7_task, "RTIndy7_task", 0, 99, 0);
	rt_task_start(&RTIndy7_task, &RTIndy7_run, NULL);

	// RTIndy7 safety
	rt_task_create(&safety_task, "safety_task", 0, 90, 0);
	rt_task_start(&safety_task, &safety_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 70, 0);
	rt_task_start(&print_task, &print_run, NULL);
	

	// Must pause here
	//pause();

  auto node = std::make_shared<JointStatePublisherNode>();
	  rclcpp::WallRate loop_rate(1000);
	  while (rclcpp::ok())
	  {
	    rclcpp::spin_some(node);
	    loop_rate.sleep();
	  }
	  rclcpp::shutdown();  
	/*
	while (1)
	{
		usleep(1e5);
	}
	*/
	// Finalize
	signal_handler(0);

    return 0;
}


