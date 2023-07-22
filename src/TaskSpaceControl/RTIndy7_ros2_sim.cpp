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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
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
unsigned long  step;

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



void setJoint(JVec q_r,JVec q_l){
	for(int i =0;i<6;i++){
		left_info.act.q[i]= q_l(i);
		right_info.act.q[i]= q_r(i);
	}
}
// RTIndy7_task
void RTIndy7_run(void *arg)
{
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	RTIME now, previous=0;

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	double dt = 0.1;
	double t =0;
	JVec q_r = JVec::Zero();
	JVec qdot_r = JVec::Zero();
	JVec qddot_r = JVec::Zero();
	JVec tau_r = JVec::Zero();
	Vector6d Ftip_r = Vector6d::Zero();
	
	JVec q_l = JVec::Zero();
	JVec qdot_l = JVec::Zero();
	JVec qddot_l = JVec::Zero();
	JVec tau_l = JVec::Zero();
	Vector6d Ftip_l = Vector6d::Zero();

	while (run)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle
		previous = rt_timer_read();

		qddot_r=mr::ForwardDynamics( q_r, qdot_r, tau_r, mr_indy7_r.g, Ftip_r, mr_indy7_r.Mlist,mr_indy7_r.Glist, mr_indy7_r.Slist);
		qddot_l=mr::ForwardDynamics( q_l, qdot_l, tau_l, mr_indy7_l.g, Ftip_l, mr_indy7_l.Mlist,mr_indy7_l.Glist, mr_indy7_l.Slist);
		mr::EulerStep(q_r,qdot_r,qddot_r,dt);
		mr::EulerStep(q_l,qdot_l,qddot_l,dt);
		setJoint(q_r,q_l);
		
		now = rt_timer_read();
		step=(unsigned long)(now - previous) / 1000000;
		t+=dt;
	}
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
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		rt_printf("cycle_dt=%lius\n",step/1000);
		
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
	rclcpp::shutdown();
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

	
	// RTIndy7_task: create and start
	printf("Now running rt task ...\n");

	// RTIndy7 control
	rt_task_create(&RTIndy7_task, "RTIndy7_task", 0, 99, 0);
	rt_task_start(&RTIndy7_task, &RTIndy7_run, NULL);

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
	// Finalize
	signal_handler(0);

    return 0;
}


