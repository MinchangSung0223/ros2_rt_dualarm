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
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"
#include "MR_Indy7_DualArm.h"

#include "modern_robotics_relative.h"

//#include "FTread.h"

//
JointInfo right_info;
JointInfo left_info;
JointInfo info;

canHandle hnd_R,hnd_L;
canStatus stat_R,stat_L;

MR_Indy7 mr_indy7_l;
MR_Indy7 mr_indy7_r;
MR_Indy7_DualArm dualarm;

// Xenomai RT tasks
RT_TASK RTIndy7_task;
RT_TASK readFT_task;
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
	rtcp_wrench_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>("rtcp_wrench_topic", 10);
	ltcp_wrench_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>("ltcp_wrench_topic", 10);
    joint_state_timer_ = create_wall_timer(std::chrono::microseconds(10000), std::bind(&JointStatePublisherNode::publish_joint_state, this));
    rtcp_wrench_timer_ = create_wall_timer(std::chrono::microseconds(10000), std::bind(&JointStatePublisherNode::publish_rtcp_wrench, this));
	ltcp_wrench_timer_ = create_wall_timer(std::chrono::microseconds(10000), std::bind(&JointStatePublisherNode::publish_ltcp_wrench, this));
  }

private:
  rclcpp::TimerBase::SharedPtr joint_state_timer_,rtcp_wrench_timer_,ltcp_wrench_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr rtcp_wrench_publisher_, ltcp_wrench_publisher_;
  void publish_rtcp_wrench(){
	auto rtcp_wrench_msg = std::make_shared<geometry_msgs::msg::WrenchStamped>();
	rtcp_wrench_msg->header.frame_id = "r_tcp"; // Set the frame ID
	rtcp_wrench_msg->header.stamp = this->now();
    rtcp_wrench_msg->wrench.force.x = std::isfinite(right_info.act.F(0)) ? right_info.act.F(0) : 0.0;
    rtcp_wrench_msg->wrench.force.y = std::isfinite(right_info.act.F(1)) ? right_info.act.F(1) : 0.0;
    rtcp_wrench_msg->wrench.force.z = std::isfinite(right_info.act.F(2)) ? right_info.act.F(2) : 0.0;
    rtcp_wrench_msg->wrench.torque.x = std::isfinite(right_info.act.F(3)) ? right_info.act.F(3): 0.0;
    rtcp_wrench_msg->wrench.torque.y = std::isfinite(right_info.act.F(4)) ? right_info.act.F(4): 0.0;
    rtcp_wrench_msg->wrench.torque.z = std::isfinite(right_info.act.F(5)) ? right_info.act.F(5): 0.0;
	rtcp_wrench_publisher_->publish(*rtcp_wrench_msg);	
  }
  void publish_ltcp_wrench(){
	auto ltcp_wrench_msg = std::make_shared<geometry_msgs::msg::WrenchStamped>();
	ltcp_wrench_msg->header.frame_id = "l_tcp"; // Set the frame ID
	ltcp_wrench_msg->header.stamp = this->now();
    ltcp_wrench_msg->wrench.force.x = std::isfinite(left_info.act.F(0)) ? left_info.act.F(0) : 0.0;
    ltcp_wrench_msg->wrench.force.y = std::isfinite(left_info.act.F(1)) ? left_info.act.F(1) : 0.0;
    ltcp_wrench_msg->wrench.force.z = std::isfinite(left_info.act.F(2)) ? left_info.act.F(2) : 0.0;
    ltcp_wrench_msg->wrench.torque.x = std::isfinite(left_info.act.F(3)) ? left_info.act.F(3): 0.0;
    ltcp_wrench_msg->wrench.torque.y = std::isfinite(left_info.act.F(4)) ? left_info.act.F(4) : 0.0;
    ltcp_wrench_msg->wrench.torque.z = std::isfinite(left_info.act.F(5)) ? left_info.act.F(5): 0.0;
	ltcp_wrench_publisher_->publish(*ltcp_wrench_msg);
  }  
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
	// info.act.F(0) = FTRawFx[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	// info.act.F(1) = FTRawFy[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	// info.act.F(2) = FTRawFz[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	// info.act.F(3) = FTRawTx[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	// info.act.F(4) = FTRawTy[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	// info.act.F(5) = FTRawTz[NUM_IO_MODULE+NUM_AXIS] / torque_divider;

	// info.act.F_CB(0) = FTRawFxCB[0] / force_divider;
	// info.act.F_CB(1) = FTRawFyCB[0] / force_divider;
	// info.act.F_CB(2) = FTRawFzCB[0] / force_divider;
	// info.act.F_CB(3) = FTRawTxCB[0] / torque_divider;
	// info.act.F_CB(4) = FTRawTyCB[0] / torque_divider;
	// info.act.F_CB(5) = FTRawTzCB[0] / torque_divider;

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
static void check(char *id, canStatus stat)
{
    if (stat != canOK) {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
    }
}

static void printUsageAndExit(char *prgName)
{
    printf("Usage: '%s <channel>'\n", prgName);
    exit(1);
}
static const char *busStatToStr(const unsigned long flag)
{
    switch (flag) {
    case canSTAT_ERROR_PASSIVE:
        return "canSTAT_ERROR_PASSIVE";

    case canSTAT_BUS_OFF:
        return "canSTAT_BUS_OFF";

    case canSTAT_ERROR_WARNING:
        return "canSTAT_ERROR_WARNING";

    case canSTAT_ERROR_ACTIVE:
        return "canSTAT_ERROR_ACTIVE";

    default:
        return "";
    }
}

void notifyCallback(canNotifyData *data)
{
    switch (data->eventType) {
    case canEVENT_STATUS:
        //rt_printf("CAN Status Event: %s\n", busStatToStr(data->info.status.busStatus));
        break;

    case canEVENT_ERROR:
       // rt_printf("CAN Error Event\n");
        break;

    case canEVENT_TX:
        //rt_printf("CAN Tx Event\n");
        break;

    case canEVENT_RX:
        //rt_printf("CAN Rx Event\n");
        break;
    }
    return;
}
// readFT task
void readFT_run(void *arg){

 	canHandle hnd_R,hnd_L;
    canStatus stat_R,stat_L;
    int channel_R=1;
	int channel_L=0;
    struct sigaction sigact;

    canInitializeLibrary();

    /* Open channel, set parameters and go on bus */
    hnd_R = canOpenChannel(channel_R,
                         canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
    if (hnd_R < 0) {
        printf("canOpenChannel %d", channel_R);
        //check("", hnd);
    }

    hnd_L = canOpenChannel(channel_L,
                         canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
    if (hnd_L < 0) {
        printf("canOpenChannel %d", channel_L);
        //check("", hnd);
    }


    stat_R = canSetBusParams(hnd_R, canBITRATE_1M, 0, 0, 0, 0, 0);
    check("canSetBusParams", stat_R);
    if (stat_R != canOK) {
        //return -1;
    }
    stat_L = canSetBusParams(hnd_L, canBITRATE_1M, 0, 0, 0, 0, 0);
    check("canSetBusParams", stat_L);
    if (stat_L != canOK) {
        //return -1;
    }
    stat_R = canSetNotify(hnd_R, notifyCallback,
                        canNOTIFY_RX | canNOTIFY_TX | canNOTIFY_ERROR | canNOTIFY_STATUS |
                            canNOTIFY_ENVVAR,
                        (char *)0);
    check("canSetNotify", stat_R);
    stat_L = canSetNotify(hnd_L, notifyCallback,
                        canNOTIFY_RX | canNOTIFY_TX | canNOTIFY_ERROR | canNOTIFY_STATUS |
                            canNOTIFY_ENVVAR,
                        (char *)0);
    check("canSetNotify", stat_L);

    stat_R = canBusOn(hnd_R);
    check("canBusOn", stat_R);
    if (stat_R != canOK) {
        //goto ErrorExit;
    }
    stat_L = canBusOn(hnd_L);
    check("canBusOn", stat_L);
    if (stat_L != canOK) {
        //goto ErrorExit;
    }	
    long id_ = 0x102;
    unsigned char sendMsg_R[8] = {0x01, 0x03, 0x01,0x00,0x00,0x00,0x00,0x00};
    stat_R = canWrite(hnd_R, id_, sendMsg_R, 8, canMSG_STD);
    check("canWrite", stat_R);
    if (stat_R != canOK) {
       // goto ErrorExit;
    }
	   
    stat_R = canWrite(hnd_R, id_, sendMsg_R, 8, canMSG_STD);
    check("canWrite", stat_R);
    if (stat_R != canOK) {
       // goto ErrorExit;
    }
    unsigned char sendMsg_L[8] = {0x01, 0x03, 0x01,0x00,0x00,0x00,0x00,0x00};
    stat_L = canWrite(hnd_L, id_, sendMsg_L, 8, canMSG_STD);
    check("canWrite", stat_L);
    if (stat_L != canOK) {
       // goto ErrorExit;
    }
	stat_L = canWrite(hnd_L, id_, sendMsg_L, 8, canMSG_STD);
    check("canWrite", stat_L);
    if (stat_L != canOK) {
       // goto ErrorExit;
    }

    stat_R = canWriteSync(hnd_R, 1000);
    check("canWriteSync", stat_R);
    if (stat_R != canOK) {
     //   goto ErrorExit;
    }
	stat_L = canWriteSync(hnd_R, 1000);
    check("canWriteSync", stat_L);
    if (stat_L != canOK) {
     //   goto ErrorExit;
    }
	double    init_FT_R[6],init_FT_L[6];

	for (int k= 0;k<1000;k++){
		long id;
		unsigned char msg[8];
		unsigned int dlc;
		unsigned int flag;
		unsigned long time;
		stat_R = canReadWait(hnd_R, &id, &msg, &dlc, &flag, &time, -1);
		if(id == 0x01){
			init_FT_R[0] = (msg[0]*256.0+msg[1])/100.0-300.0;
			init_FT_R[1] =  (msg[2]*256.0+msg[3])/100.0-300.0;
			init_FT_R[2] = (msg[4]*256.0+msg[5])/100.0-300.0;   	        
		}
		stat_R = canReadWait(hnd_R, &id, &msg, &dlc, &flag, &time, -1);
		if(id == 0x01+0x01){
			init_FT_R[3]  = (msg[0]*256+msg[1])/500.0-50.0;
			init_FT_R[4]  = (msg[2]*256+msg[3])/500.0-50.0;
			init_FT_R[5]  = (msg[4]*256+msg[5])/500.0-50.0;
		}
		stat_L = canReadWait(hnd_L, &id, &msg, &dlc, &flag, &time, -1);
		if(id == 0x01){
			init_FT_L[0] = (msg[0]*256.0+msg[1])/100.0-300.0;
			init_FT_L[1] =  (msg[2]*256.0+msg[3])/100.0-300.0;
			init_FT_L[2] = (msg[4]*256.0+msg[5])/100.0-300.0;   	        
		}
		stat_L = canReadWait(hnd_L, &id, &msg, &dlc, &flag, &time, -1);
		if(id == 0x01+0x01){
			init_FT_L[3]  = (msg[0]*256+msg[1])/500.0-50.0;
			init_FT_L[4]  = (msg[2]*256+msg[3])/500.0-50.0;
			init_FT_L[5]  = (msg[4]*256+msg[5])/500.0-50.0;
		}
	}
	double    prev_FT_R[6],prev_FT_L[6];
	double    filtered_FT_R[6],filtered_FT_L[6];
	for(int jj = 0;jj<6;jj++){
			filtered_FT_R[jj]=init_FT_R[jj];
			filtered_FT_L[jj]=init_FT_L[jj];
	}
		
	
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	while(run){
		rt_task_wait_period(NULL);
		long id;
		unsigned char msg[8];
		unsigned int dlc;
		unsigned int flag;
		unsigned long time;
		double    FT_R[6],FT_L[6];
		stat_R = canReadWait(hnd_R, &id, &msg, &dlc, &flag, &time, -1);
		if(id == 0x01){
			FT_R[0] = (msg[0]*256.0+msg[1])/100.0-300.0-init_FT_R[0];
			FT_R[1] =  (msg[2]*256.0+msg[3])/100.0-300.0-init_FT_R[1];
			FT_R[2] = (msg[4]*256.0+msg[5])/100.0-300.0-init_FT_R[2];   	        
		}
		stat_R = canReadWait(hnd_R, &id, &msg, &dlc, &flag, &time,-1);
		if(id == 0x01+0x01){
			FT_R[3]  = (msg[0]*256+msg[1])/500.0-50.0-init_FT_R[3];
			FT_R[4]  = (msg[2]*256+msg[3])/500.0-50.0-init_FT_R[4];
			FT_R[5]  = (msg[4]*256+msg[5])/500.0-50.0-init_FT_R[5];
		}
		stat_L = canReadWait(hnd_L, &id, &msg, &dlc, &flag, &time, -1);
		if(id == 0x01){
			FT_L[0] = (msg[0]*256.0+msg[1])/100.0-300.0-init_FT_L[0];
			FT_L[1] =  (msg[2]*256.0+msg[3])/100.0-300.0-init_FT_L[1];
			FT_L[2] = (msg[4]*256.0+msg[5])/100.0-300.0-init_FT_L[2];   	        
		}
		stat_L = canReadWait(hnd_L, &id, &msg, &dlc, &flag, &time, -1);
		if(id == 0x01+0x01){
			FT_L[3]  = (msg[0]*256+msg[1])/500.0-50.0-init_FT_L[3];
			FT_L[4]  = (msg[2]*256+msg[3])/500.0-50.0-init_FT_L[4];
			FT_L[5]  = (msg[4]*256+msg[5])/500.0-50.0-init_FT_L[5];
		}		
		double alpha = 0.99;
		for(int i=0;i<6;i++){
			filtered_FT_R[i] = (alpha)*filtered_FT_R[i]+(1-alpha)*FT_R[i];
			filtered_FT_L[i] = (alpha)*filtered_FT_L[i]+(1-alpha)*FT_L[i];
			if (abs(filtered_FT_R[i])>100.0 ){
				if(filtered_FT_R[i]<0){
					right_info.act.F(i)=-100.0;	
				}
				else{
					right_info.act.F(i)=100.0;	
				}
			}
			else{
				if(std::isfinite(filtered_FT_R[i]))
					right_info.act.F(i) = filtered_FT_R[i];
				else
					right_info.act.F(i) = 0.0;
			}

			if (abs(filtered_FT_L[i])>100.0 ){
				if(filtered_FT_L[i]<0){
					left_info.act.F(i)=-100.0;	
				}
				else{
					left_info.act.F(i)=100.0;	
				}
			}
			else{
				if(std::isfinite(filtered_FT_L[i]))
					left_info.act.F(i) = filtered_FT_L[i];
				else
					left_info.act.F(i) = 0.0;
			}
		}
			
		

	}
	
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
	relmr::JVec q_init=relmr::JVec::Zero();
	q_init.segment<6>(0) = qT_r;
	q_init.segment<6>(6) = qT_l;
	mr::SE3 X_r_des = mr_indy7_r.T_b(qT_r);
	mr::SE3 X_l_des = mr_indy7_l.T_b(qT_l);
	mr::Vector6d V_r_des = mr::Vector6d::Zero();
	mr::Vector6d dV_r_des = mr::Vector6d::Zero();
	mr::Vector6d F_r_des = mr::Vector6d::Zero();	
	mr::Vector6d V_l_des = mr::Vector6d::Zero();
	mr::Vector6d dV_l_des = mr::Vector6d::Zero();
	mr::Vector6d F_l_des = mr::Vector6d::Zero();	
	relmr::JVec q = relmr::JVec::Zero();
	relmr::JVec qdot = relmr::JVec::Zero();
	relmr::JVec qddot = relmr::JVec::Zero();

	double dt = 0.001;
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
			q.segment<6>(0) = right_info.act.q;
			q.segment<6>(6) = left_info.act.q;
			qdot.segment<6>(0) = right_info.act.q_dot;
			qdot.segment<6>(6) = left_info.act.q_dot;
			dualarm.FKinBody(q,qdot);
			relmr::MassMat Mmat = dualarm.MassMatrix(q);	
			relmr::JVec C = dualarm.VelQuadraticForces(q,qdot);
			relmr::JVec G = dualarm.GravityForces(q);		
			mr::MassMat Mmat_r=Mmat.topLeftCorner<6, 6>();
			mr::MassMat Mmat_l=Mmat.bottomRightCorner<6, 6>();
			mr::JVec C_r = C.segment<6>(0);
			mr::JVec C_l = C.segment<6>(6);
			mr::JVec G_r = G.segment<6>(0);
			mr::JVec G_l = G.segment<6>(6);
			// Calculate Joint controller
			e_r = right_info.des.q-right_info.act.q;
			eint_r = eint_r+e_r*dt;
			e_l = left_info.des.q-left_info.act.q;
			eint_l = eint_l+e_l*dt;
			
			mr::Vector6d Ftip_r = mr::Vector6d::Zero();
			mr::Vector6d Ftip_l = mr::Vector6d::Zero();

			Ftip_r(5)= right_info.act.F(2);
			Ftip_r(4)= right_info.act.F(1);
			Ftip_r(3)= right_info.act.F(0);

			Ftip_r(0)= right_info.act.F(3);
			Ftip_r(1)= right_info.act.F(4);
			Ftip_r(2)= right_info.act.F(5);

			Ftip_l(5)= left_info.act.F(2);
			Ftip_l(4)= left_info.act.F(1);
			Ftip_l(3)= left_info.act.F(0);

			Ftip_l(0)= left_info.act.F(3);
			Ftip_l(1)= left_info.act.F(4);
			Ftip_l(2)= left_info.act.F(5);

			SE3 T_ =SE3::Identity();
			T_ << 0 ,-1 ,0 ,0 ,
			      1,0,0,0,
				  0,0,1,0,
				  0,0,0,1;
			Ftip_r = Adjoint(T_).transpose()*Ftip_r;	  
			Ftip_l = Adjoint(T_).transpose()*Ftip_l;
			
			//F_r_des(5)=20*sin(2*3.141592*gt/10.0)-30;
			//F_l_des(5)=-20*sin(2*3.141592*gt/10.0)-30;
			//F_r_des(5)=-30;
			//F_l_des(5)=-30;
			//mr::JVec tau_r = dualarm.R->ImpedanceControl(right_info.act.q,right_info.act.q_dot,Ftip_r,dualarm.T0r,dualarm.Jb_r,dualarm.Jbdot_r,X_r_des,V_r_des,dV_r_des, F_r_des,Mmat_r,C_r,G_r);
			//mr::JVec tau_l = dualarm.L->ImpedanceControl(left_info.act.q,left_info.act.q_dot,Ftip_l,dualarm.T0l,dualarm.Jb_l,dualarm.Jbdot_l,X_l_des,V_l_des,dV_l_des, F_l_des,Mmat_l,C_l,G_l);
			
			//right_info.des.tau = G_r;
			//left_info.des.tau = G_l;
			mr::JVec tau_r = dualarm.R->HinfControl( right_info.act.q , right_info.act.q_dot, right_info.des.q, right_info.des.q_dot,right_info.des.q_ddot,eint_r);
			mr::JVec tau_l =dualarm.L->HinfControl( left_info.act.q , left_info.act.q_dot, left_info.des.q, left_info.des.q_dot,left_info.des.q_ddot,eint_l);
			right_info.des.tau = tau_r;
			left_info.des.tau = tau_l;
			//right_info.des.tau = mr_indy7_r.HinfControl( right_info.act.q , right_info.act.q_dot, right_info.des.q, right_info.des.q_dot,right_info.des.q_ddot,eint_r)+Jb_r.transpose()*Ftip_r;
			//left_info.des.tau = mr_indy7_l.HinfControl( left_info.act.q , left_info.act.q_dot, left_info.des.q, left_info.des.q_dot,left_info.des.q_ddot,eint_l)+Jb_l.transpose()*Ftip_l;
			//right_info.des.tau = mr_indy7_r.HinfControl( right_info.act.q , right_info.act.q_dot, right_info.des.q, right_info.des.q_dot,right_info.des.q_ddot,eint_r);
			//left_info.des.tau = mr_indy7_l.HinfControl( left_info.act.q , left_info.act.q_dot, left_info.des.q, left_info.des.q_dot,left_info.des.q_ddot,eint_l);
			//right_info.des.tau =tauVec_r;
			//left_info.des.tau =tauVec_l;			
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
			rt_printf("Right FT : %.3f \t %.3f \t %.3f \t %.3f \t %.3f \t %.3f\n",right_info.act.F(0),right_info.act.F(1),right_info.act.F(2),right_info.act.F(3),right_info.act.F(4),right_info.act.F(5));
			rt_printf("Left FT : %.3f \t %.3f \t %.3f \t %.3f \t %.3f \t %.3f\n",left_info.act.F(0),left_info.act.F(1),left_info.act.F(2),left_info.act.F(3),left_info.act.F(4),left_info.act.F(5));

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
	rt_task_delete(&readFT_task);

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

	mr_indy7_l=MR_Indy7();
    mr_indy7_l.MRSetup();
	mr_indy7_l.g<<0,8.487,-4.9;
	mr_indy7_r=MR_Indy7();
    mr_indy7_r.MRSetup();
	mr_indy7_r.g<<0,8.487,-4.9;

	dualarm=MR_Indy7_DualArm();
	dualarm.MRSetup();	
	dualarm.L->eef_mass = 0.7;
    dualarm.R->eef_mass = 0.7;

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

	// RTIndy7 readFT 
	rt_task_create(&readFT_task, "readFT_task", 0, 80, 0);
	rt_task_start(&readFT_task, &readFT_run, NULL);

	// RTIndy7 safety
	rt_task_create(&safety_task, "safety_task", 0, 90, 0);
	rt_task_start(&safety_task, &safety_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 70, 0);
	rt_task_start(&print_task, &print_run, NULL);
	

	// Must pause here
	//pause();

  auto node = std::make_shared<JointStatePublisherNode>();
	  rclcpp::WallRate loop_rate(100);
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


