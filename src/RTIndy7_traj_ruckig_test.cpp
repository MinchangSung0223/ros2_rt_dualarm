
#include "RTIndy7Client.h"
#include "MR_Indy7.h"
#include "MR_DualArm.h"
// #include "ScurveGenerator.h"
#include <iostream>
#include <ruckig/ruckig.hpp>
using namespace ruckig;

int main(void){
    rt_printf("Hello!");
    //vector<Trajectory> traj_list;
    RTIME now,previous;
    Ruckig<12> otg {0.001};  // control cycle
    InputParameter<12> input;
    OutputParameter<12> output;

    // Set input parameters
    input.current_position = {0,0,0,0,0,0,0,0,0,0,0,0};
    input.current_velocity = {0,0,0,0,0,0,0,0,0,0,0,0};
    input.current_acceleration = {0,0,0,0,0,0,0,0,0,0,0,0};

    input.target_position = {1,1,1,1,1,1,1,1,1,1,1,1};
    input.target_velocity = {0,0,0,0,0,0,0,0,0,0,0,0};
    input.target_acceleration = {0,0,0,0,0,0,0,0,0,0,0,0};

    input.max_velocity = {10,10,10,10,10,10,10,10,10,10,10,10};
    input.max_acceleration = {100,100,100,100,100,100,100,100,100,100,100,100};
    input.max_jerk = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};

    // Generate the trajectory within the control loop
    std::cout << "t | p1 | p2 | p3" << std::endl;
    previous = rt_timer_read();
    while (otg.update(input, output) == Result::Working) {
        auto& p = output.new_position;
        std::cout << output.time;
        for(int i =0;i<12;i++){
            cout<<p[i]<<" ";
        }
          cout<<" " << std::endl;

        output.pass_to_input(input);
    }
    now = rt_timer_read();
    rt_printf("\n %llu ns \n",now-previous);
    std::cout << "Trajectory duration: " << output.trajectory.get_duration() << " [s]." << std::endl;
    // for(int j = 0;j<12;j++){

    //     Trajectory traj; 
    //     traj.j = 2000;
    //     traj.amax =100;
    //     traj.dmax =100;
    //     traj.vmax = 10;
    //     traj.vmin =-10;
    //     traj.ao = 0;
    //     traj.vo = 0;
    //     traj.af = 0;
    //     traj.vf = 0;
    //     traj.so = 0;
    //     traj.sf = 1;
    //     traj_list.push_back(traj);
    // }
    // ScurveGenerator sg = ScurveGenerator(traj_list);
    // previous = rt_timer_read();
    // for(int j =0;j<12;j++){
    //     sg.update_(j,U2D);
    //     sg.updateTargetTime_(j,U2D,1.0);
    // }
    // now = rt_timer_read();
    // rt_printf("\n %llu ns",now-previous);

    return -1;
}