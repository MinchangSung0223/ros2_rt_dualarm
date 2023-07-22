#include <canlib.h>
#include <canstat.h>
#include <iostream>
#include <string>
#include <csignal>
#include <cerrno>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <Eigen/Dense>
#include <iomanip>

#include <canlib.h>
#include <canstat.h>
#include <iostream>
#include <string>
#include <csignal>
#include <cerrno>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#define READ_WAIT_INFINITE (unsigned long)(-1)
static unsigned int msgCounter = 0;

using namespace Eigen;
using namespace std;
class FTread
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;   
    typedef Eigen::Matrix<double, 3, 1> Vector3d;       
    canHandle hnd;
    canStatus stat;
    int channel;
    int canId_R;
    int canId_L;    
    struct sigaction sigact;    

    public:
        Vector6d FT_R;
        Vector6d init_FT_R;
        Vector6d filtered_FT_R;
        Vector6d prev_filtered_FT_R;
        Vector6d prev_FT_R;
        Vector6d FT_L;
        Vector6d init_FT_L;
        Vector6d filtered_FT_L;
        Vector6d prev_filtered_FT_L;
        Vector6d prev_FT_L;        
        int init_flag;
        double wc;  //cut-off-frequency for LPF
        double dt;  //Sampling Time
        double Fs;  //Sampling Freq
        FTread();
        FTread(int channel, int canId_R, int canId_L);
        void readData();
        void setBias();
        void setCutOffFreq(double wc);
        void clearBias();
        void setId();        
        void initialize();
        void print(Vector6d vec);
        ~FTread();
};