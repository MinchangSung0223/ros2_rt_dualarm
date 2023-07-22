#include "../RTIndy7Client.h"
#include "MR_Indy7.h"
#include "MR_DualArm.h"
#include "ScurveGenerator.h"

int main(void){
    rt_printf("Hello!");
    vector<Trajectory> traj_list;
    RTIME now,previous;
    
    for(int j = 0;j<12;j++){

        Trajectory traj; 
        traj.j = 1000;
        traj.amax =100;
        traj.dmax =100;
        traj.vmax = 10;
        traj.vmin =0;
        traj.ao = 1;
        traj.vo = 1;
        traj.af = 1;
        traj.vf = 1;
        traj.so = 0;
        traj.sf = 1;
        traj_list.push_back(traj);
    }
    ScurveGenerator sg = ScurveGenerator(traj_list);
    previous = rt_timer_read();
    for(int j =0;j<12;j++){
        sg.update_(j,U2D);
        Trajectory traj = traj_list.at(j);
        sg.updateTargetTime(&sg.traj_list.at(j),1);
    }
    now = rt_timer_read();
    rt_printf("\n %llu ns",now-previous);
    sg.printAllTrajectory();


    return -1;
}
