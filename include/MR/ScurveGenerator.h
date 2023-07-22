#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include <cstring>
#include <iomanip> 
#include <cmath>
#include <chrono>

using namespace std;
using namespace chrono;
#define EPS 1e-15
#define INF 1e+15
#define ERR_EPS  1e-11
#define SQRT2 sqrt(2)
#define MAXITER 100
#define TOLERANCE 1e-12
enum {
    U2D = 0,
    U2U = 1,
    D2D = 2,
    D2U = 3,
    UU1 = 4,
    UU2 = 5,
    DD = 6,

    INVALID_S = 100,
    INVALID_V = 101,
    INVALID_VO = 102,
    INVALID_VF = 103,
    INVALID_AO = 104,
    INVALID_AF = 105,
    INVALID_J = 106,
    INVALID_A1 = 107,
    INVALID_A2 = 108,

    
    MINUS_TJ1 = 201,
    MINUS_TJ2 = 202,
    MINUS_TJ3 = 203,
    MINUS_TJ4 = 204,
    MINUS_TCA1 = 205,
    MINUS_TCA2 = 206,
    MINUS_TC = 207,
    DISPLACEMENT_ERROR = 208,

    

    ERROR_MINUS_VELOCITY = 301,
    ERROR_OVER_VELOCITY = 302,

    WRONG_TARGET_TIME = 400,
    MINTIME_ERR = 401,
    MAXTIME_ERR = 402,
    UPDATE_ERR = 403,

    VMIN_ERR = 501,
    VMAX_ERR = 502

};


class Trajectory {
public:
    double ta1; 
    double ta2; 
    double tc;  
    double tt;  
    double tj1; 
    double tj2; 
    double tj3; 
    double tj4; 
    double tca1; 
    double tca2; 
    double vo;  
    double vf;  
    double vp;  
    double vmax;   
    double vmin;   

    double amax;   
    double dmax;   
    double a1;    
    double a2;   
    double ao;  
    double af; 
    double j;  
    double so; 
    double sf; 

    double sa1; 
    double sa2; 
    double sj1; 
    double sj2; 
    double sj3; 
    double sj4; 
    double sca1; 
    double sca2; 
    double sc; 
    double sof; 
    double s;  
    double ds; 
    double dds; 
    double ddds; 
    double st[6] = { 0 };
    double dst[6] = { 0 };


    double vp_lower_limits[8] = { 0 };
    double vp_upper_limits[8] = { 0 };
    double a1_upper_limits[8] = { 0 };
    double a1_lower_limits[8] = { 0 }; 
    double a2_upper_limits[8] = { 0 };
    double a2_lower_limits[8] = { 0 };

    double min_time[8] = { 0 }; 
    double max_time[8] = { 0 }; 

    double k[8][2] = { {1.0,-1.0},{1.0,1.0},{-1.0,-1.0},{-1.0,1.0},{1,-1} ,{1,-1} ,{-1,1} ,{1,-1} };
    unsigned int type = 0;
    int update_required = 0; 
    unsigned int init_err[8] = { 0 };
    unsigned int err[8] = { 0 };
    unsigned int min_time_err[8] = { 0 };
    unsigned int max_time_err[8] = { 0 };
    double v_0=0;
    double state; 

    Trajectory() {};
};
class ScurveGenerator
{
protected:
    double sync_time;
    int traj_err;
public:
    vector<Trajectory> traj_list;
    ScurveGenerator(vector<Trajectory> _traj_list) {

        for (int idx = 0; idx < _traj_list.size(); idx++) {
            this->traj_list.push_back(_traj_list.at(idx));
            //generate(&this->traj_list.at(idx));
           // checkInitialError(&this->traj_list.at(idx));
            setLimits(&this->traj_list.at(idx));
                    this->traj_list.at(idx).type = findMinTrajType(&this->traj_list.at(idx));

        }
    };
    int get_traj_err(){
        return this->traj_err;
    };
    double max(double a, double b);
    double min(double a, double b);
    vector<double> roots2(double a, double  b, double c);
    vector<double> roots3(double a, double  b, double c, double d);
    double rectifier(double val, double upper_limit, double lower_limit);



    bool checkSmall(double val, double eps);
    void generate(Trajectory* traj);
    void generate(Trajectory* traj, double t);
    void setTraj(Trajectory* traj, double so, double sf, double vo, double vf, double ao, double af, double vmax, double amax, double dmax, double  j, unsigned int type);
    Trajectory getTraj(int idx);
    vector<double> pygenerate(Trajectory traj, double t);
    void update(Trajectory* traj);
    void checkInitialError(Trajectory* traj);
    void checkError(Trajectory* traj);
    void setLimits(Trajectory* traj);
    void setMinTime(Trajectory* traj);
    void setMaxTime(Trajectory* traj);
    void getType(Trajectory* traj, double& T);
    double getTime(Trajectory* traj, double s);
    void updateTargetTime(Trajectory* traj, double T);
    bool checkAllEqual(const vector<double>& v);
    void syncTime();
    void syncTargetTime(double targetTime);
    void printTrajectory(Trajectory* traj);
    void printAllTrajectory();
    void saveAllTrajectory(double time_step);
    void setType(int num, int type);
    void update_(int num, int type);
    void updateTargetTime_(int num, int type, double T);
    void U2DminTimeTraj(Trajectory *traj,Trajectory &ret_traj);
    void U2DmaxTimeTraj(Trajectory *traj,Trajectory &ret_traj);
    void U2UminTimeTraj1(Trajectory *traj,Trajectory &ret_traj);
    void U2UminTimeTraj2(Trajectory *traj,Trajectory &ret_traj);
    void D2DminTimeTraj(Trajectory *traj,Trajectory &ret_traj);
    void D2UminTimeTraj(Trajectory *traj,Trajectory &ret_traj);
    void D2UmaxTimeTraj(Trajectory *traj,Trajectory &ret_traj);
    void U2UmaxTimeTraj1(Trajectory *traj,Trajectory &ret_traj);
    void U2UmaxTimeTraj2(Trajectory *traj,Trajectory &ret_traj);

    void D2DmaxTimeTraj(Trajectory *traj,Trajectory &ret_traj);
    vector<double> getSlist(Trajectory traj,double time_step);
    vector<double> getdSlist(Trajectory traj,double time_step);
    vector<double> getddSlist(Trajectory traj,double time_step);
    vector<double> getdddSlist(Trajectory traj,double time_step);
    int  findMinTrajType(Trajectory *traj);
};