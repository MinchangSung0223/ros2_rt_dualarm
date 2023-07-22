#include "ScurveGenerator.h"
#include <sstream>
#include <string>
double ScurveGenerator::max(double a, double b) {
    if (a > b)
        return a;
    else
        return b;
}
double ScurveGenerator::min(double a, double b) {
    if (a > b)
        return b;
    else
        return a;
}
vector<double> ScurveGenerator::roots2(double a, double  b, double c) {
    // 2차방적식의 근을 구하는 함수
    double ret1;
    double ret2;
    vector<double> roots;
    if ((b * b - 4.0 * a * c) > 0) {
        ret1 = (-b + sqrt((b * b - 4.0 * a * c))) / (2.0 * a);
        ret2 = (-b - sqrt((b * b - 4.0 * a * c))) / (2.0 * a);
        roots.push_back(ret1);
        roots.push_back(ret2);
        return roots;
    }
    else if ((b * b - 4.0 * a * c) == 0) {
        ret1 = -b / (2.0 * a);
        roots.push_back(ret1);
        return roots;
    }
    else
        return roots;
    return roots;
}
vector<double> ScurveGenerator::roots3(double a, double  b, double c, double d) {
    // 3차방적식의 근을 구하는 함수
    vector<double> ret;
    if (a == 0) {
        ////cout << "The coefficient of the cube of x is 0. Please use the utility for a SECOND degree quadratic. No further action taken" << endl;
        return ret;
    }
    if (d == 0) {
        ////cout << "One root is 0. Now divide through by x and use the utility for a SECOND degree quadratic to solve the resulting equation for the other two roots. No further action taken." << endl;
        return ret;
    }
    b /= a;
    c /= a;
    d /= a;
    double q = (3.0 * c - (b * b)) / 9.0;
    double r = -(27.0 * d) + b * (9.0 * c - 2.0 * b * b);
    r /= 54.0;
    double disc = q * q * q + r * r;
    double s, t;
    double x1_real, x1_imag;
    double x2_real, x2_imag;
    double x3_real, x3_imag;
    x1_imag = 0;
    double term1 = (b / 3.0);
    if (disc > 0) {
        //하나의 근이 실근, 두개는 복소수
        s = r + sqrt(disc);
        s = ((s < 0) ? -pow(-s, (1.0 / 3.0)) : pow(s, (1.0 / 3.0)));
        t = r - sqrt(disc);
        t = ((t < 0) ? -pow(-t, (1.0 / 3.0)) : pow(t, (1.0 / 3.0)));
        x1_real = -term1 + s + t;
        term1 += (s + t) / 2.0;
        x3_real = -term1;
        x2_real = -term1;
        term1 = sqrt(3.0) * (-t + s) / 2;
        x2_imag = term1;
        x3_imag = -term1;
        ////cout << "CASE1" << endl;
        ////cout << "x1 : " << x1_real <<"\t" << endl;
        ////cout << "x2 : " << x2_real << "\t" << x2_imag << "i" << endl;
        ////cout << "x3 : " << x3_real << "\t" << x3_imag << "i" << endl;
        ret.push_back(x1_real);
        return ret;
    }
    else if (disc == 0) {
        x3_imag = 0;
        x2_imag = 0;
        double r13 = ((r < 0) ? -pow(-r, (1.0 / 3.0)) : pow(r, (1.0 / 3.0)));
        x1_real = -term1 + 2.0 * r13;
        x3_real = -(r13 + term1);
        x2_real = -(r13 + term1);
        ////cout << "CASE2" << endl;
        ////cout << "x1 : " << x1_real  << endl;
        ////cout << "x2 : " << x2_real <<"\t" << endl;
        ////cout << "x3 : " << x3_real <<"\t"  << endl;
        ret.push_back(x1_real);
        ret.push_back(x2_real);
        ret.push_back(x3_real);

        return ret;

    }
    q = -q;
    double dum1 = q * q * q;
    dum1 = acos(r / sqrt(dum1));
    double r13 = 2.0 * sqrt(q);
    x1_real = -term1 + r13 * cos(dum1 / 3.0);
    x2_real = -term1 + r13 * cos((dum1 + 2.0 * M_PI) / 3.0);
    x3_real = -term1 + r13 * cos((dum1 + 4.0 * M_PI) / 3.0);
    x2_imag = 0;
    x3_imag = 0;
    ////cout << "CASE3" << endl;
    ////cout << "x1 : " << x1_real << x1_imag << "i" << endl;
    ////cout << "x2 : " << x2_real << x2_imag << "i" << endl;
    ////cout << "x3 : " << x3_real << x3_imag << "i" << endl;

    ret.push_back(x1_real);
    ret.push_back(x2_real);
    ret.push_back(x3_real);
    return ret;
}
double ScurveGenerator::rectifier(double val, double upper_limit, double lower_limit) {
    if (val > upper_limit) {
        return upper_limit;
    }
    else if (val <= upper_limit && val >= lower_limit) {
        return val;
    }
    else {
        return lower_limit;
    }
}
bool ScurveGenerator::checkSmall(double val, double eps) {
    bool ret = false;
    if (abs(val) < eps) {
        ret = true;
    }
    return ret;
}

void ScurveGenerator::generate(Trajectory* traj) {


    unsigned int type = traj->type;
    double k1 = traj->k[type][0];
    double k2 = traj->k[type][1];
    double so = traj->so;
    double sf = traj->sf;
    double sof = sf - so;
    double vp = traj->vp;
    double vo = traj->vo;
    double vf = traj->vf;
    double ao = traj->ao;
    double af = traj->af;
    double a1 = traj->a1;
    double a2 = traj->a2;
    double vmax = traj->vmax;
    double amax = traj->amax;
    double dmax = traj->dmax;
    double j = traj->j;

    double tj1;
    double tca1;
    double tj2;
    double ta1;
    
    double tj3;
    double tca2;
    double tj4;
    double ta2;

    double vj1 ;
    double vca1 ; 
    double vj2 ;

    double vj3 ;
    double vca2 ;
    double vj4 ;


   
    double sj1;
    double sca1;
    double sj2;
    double sa1;

    double sj3;
    double sca2;
    double sj4;
    double sa2;

    double sc;
    double tc;
    double tt;

    double t0;
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    
    double dst0 ;
    double dst1 ;
    double dst2 ;
    double dst3 ;
    double dst4 ;
    double dst5 ;
    double dst6 ;
    double dst7 ;        

    double st0 ;
    double st1 ;
    double st2 ;
    double st3 ;
    double st4 ;
    double st5 ;
    double st6 ;
    double st7 ;     


    
    if(type==UU1 || type == UU2 ){
        if( type==UU1  ){
            k1 = 1;
        }else{
            k1 = -1;
        }
        tj1 = (k1*a1-ao)/j;
        tca1 = (k1+1)/2.0*(2*a2*a2 - af*af - 2*a1*a1 + ao*ao + 2*j*(vf-vo))/(2*a1*j);
        tca1 = abs(tca1) < EPS ? 0 : tca1;        
        tj2 = (-k1*a2+k1*a1)/j;
        ta1 = tj1+tca1+tj2;
        
        tj3 = 0;
        tca2 = -(k1-1)/2.0*(-(- 2*a1*a1 + 2*a2*a2 - af*af + ao*ao + 2*j*(vf-vo))/(2*a2*j));
        tca2 = abs(tca2) < EPS ? 0 : tca2;          
        tj4 = (-k1*a2+af)/j;
        ta2 = tj3+tca2+tj4;
        tc = 0;
        tt = tj1+tca1+tj2+tc+tj3+tca2+tj4;

        vj1 = 1.0/2.0*j*tj1*tj1+ao*tj1;
        vca1 = k1*a1*tca1;    
        vj2 = -1.0/2.0*j*tj2*tj2+k1*a1*tj2;
        vj3 = 0;
        vca2 = k1*a2*tca2;       
        vj4 = 1.0/2.0*j*tj4*tj4+k1*a2*tj4;

        sj1 = 1.0/6.0*j*tj1*tj1*tj1+1.0/2.0*ao*tj1*tj1+vo*tj1;
        sca1 = k1*1.0/2.0*a1*tca1*tca1+(vo+vj1)*tca1;        
        sj2 = -1.0/6.0*j*tj2*tj2*tj2+k1*1.0/2.0*a1*tj2*tj2+(vo+vj1+vca1)*tj2;
        sa1 = sj1+sca1+sj2;
        sc = 0;
        sj3= -1.0/6.0*j*tj3*tj3*tj3+(vo+vj1+vca1+vj2)*tj3;
        sca2 = k1*1.0/2.0*a2*tca2*tca2+(vo+vj1+vca1+vj2+vj3)*tca2;
        sj4 = 1.0/6.0*j*tj4*tj4*tj4+k1*1.0/2.0*a2*tj4*tj4+(vo+vj1+vca1+vj2+vj3+vca2)*tj4;

        t0 = 0;
        t1 = tj1;
        t2 = ta1 - tj2;
        t3 = ta1;
        t4 = tt - ta2;
        t5 = tt - ta2 + tj3;
        t6 = tt - tj4;
        t7 = tt;

    }
    else{

        const double aoao = ao*ao;
        const double afaf = af*af;

        tj1 = abs(a1 - k1 * ao) < EPS ? 0 : (a1 - k1 * ao) / j;

        tj2 = abs(a1) < EPS ? 0 : a1 / j;
        ta1 = abs(a1) < EPS ? INF : (2 * a1 * a1 - 2 * k1 * a1 * ao + aoao + 2 * k1 * j * vp - 2 * k1 * j * vo) / (2.0 * a1 * j);
        tca1 = ta1 - tj1 - tj2;
        tca1 = abs(tca1) < EPS ? 0 : tca1;
        tj3 = abs(a2 / j) < EPS ? 0 : a2 / j;
        tj4 = abs((a2 - k2 * af) / j) < EPS ? 0 : (a2 - k2 * af) / j;
        ta2 = abs(a2) < EPS ? INF : (k2 * (vf - vp)) / a2 + (2 * a2 * a2 - 2 * k2 * a2 * af + afaf) / (2 * a2 * j);
        tca2 = ta2 - tj4 - tj3;
        tca2 = abs(tca2) < EPS ? 0 : tca2;
       const double tj1tj1 = tj1*tj1;
        const double tj2tj2 = tj2*tj2;
        const double tj3tj3 = tj3*tj3;
        const double tj4tj4 = tj4*tj4;

        vj1 = 1.0 / 2.0 * k1 * j * tj1tj1 + ao * tj1;
        vca1 = k1 * a1 * tca1 ;
        vj2 = -1.0 / 2.0 * k1 * j * tj2tj2 + k1 * a1 * tj2;
        vj3 = 1.0 / 2.0 * k2 * j * tj3tj3;
        vca2 = k2 * a2 * tca2;
        vj4 = -1.0 / 2.0 * k2 * j * tj4* tj4 + k2*a2*tj4;

        sj1 = 1.0 / 6.0 * k1 * j * tj1tj1 * tj1 + 1.0 / 2.0 * ao * tj1tj1 + vo * tj1;
        sca1 = 1.0 / 2.0 * k1 * a1 * (tca1) * (tca1)+(1.0 / 2.0 * k1 * j * tj1tj1 + ao * tj1 + vo) * (tca1);
        sj2 = -1.0 / 6.0 * k1 * j * tj2tj2 * tj2 + 1.0 / 2.0 * k1 * a1 * tj2tj2 + (vp - k1 * a1 * a1 / 2.0 / j) * tj2;
        sa1 = sj1 + sca1 + sj2;
        sj3 = 1.0 / 6.0 * k2 * j * tj3tj3 * tj3 + vp * tj3;
        sca2 = (vp + 1.0 / 2.0 * k2 * j * tj3tj3) * (tca2)+1.0 / 2.0 * k2 * a2 * (tca2) * (tca2);
        sj4 = (vp + 1.0 / 2.0 * k2 * j * tj3tj3 + k2 * a2 * (tca2)) * tj4 - 1.0 / 6.0 * k2 * j * tj4tj4 * tj4 + 1.0 / 2.0 * k2 * a2 * tj4tj4;
        sa2 = sj3 + sca2 + sj4;
        sc = abs(sof - (sa1 + sa2)) < EPS*10.0 ? 0 : (sof - (sa1 + sa2));  
        tc = 1.0 / vp * sc;
        tt = ta1 + tc + ta2;

        t0 = 0;
        t1 = tj1;
        t2 = ta1 - tj2;
        t3 = ta1;
        t4 = tt - ta2;
        t5 = tt - ta2 + tj3;
        t6 = tt - tj4;
        t7 = tt;
  
    }
    
    dst0 = vo;
    dst1 = vo+vj1;
    dst2 = vo+vj1+vca1;
    dst3 = vo+vj1+vca1+vj2;
    dst4 = vo+vj1+vca1+vj2;
    dst5 = vo+vj1+vca1+vj2+vj3;
    dst6 = vo+vj1+vca1+vj2+vj3+vca2;
    dst7 = vo+vj1+vca1+vj2+vj3+vca2+vj4;

    st0 = so;
    st1 = so+sj1;
    st2 = so+sj1+sca1;
    st3 = so+sj1+sca1+sj2;
    st4 = so+sj1+sca1+sj2+sc;
    st5 = so+sj1+sca1+sj2+sc+sj3;
    st6 = so+sj1+sca1+sj2+sc+sj3+sca2;
    st7 = so+sj1+sca1+sj2+sc+sj3+sca2+sj4;



    // 구간별 위치,속도계산

    traj->st[0] = st1;
    traj->st[1] = st2;
    traj->st[2] = st3;
    traj->st[3] = st4;
    traj->st[4] = st5;
    traj->st[5] = st6;
    traj->dst[0] = dst1;
    traj->dst[1] = dst2;
    traj->dst[2] = dst3;
    traj->dst[3] = dst4;
    traj->dst[4] = dst5;
    traj->dst[5] = dst6;

    traj->tj1 = tj1;
    traj->tj2 = tj2;
    traj->tj3 = tj3;
    traj->tj4 = tj4;
    traj->ta1 = ta1;
    traj->ta2 = ta2;
    traj->tca1 = tca1;
    traj->tca2 = tca2;
    traj->tc = tc;
    traj->tt = tt;
    traj->sj1 = sj1;
    traj->sj2 = sj2;
    traj->sj3 = sj3;
    traj->sj4 = sj4;
    traj->sca1 = sca1;
    traj->sca2 = sca2;
    traj->sa1 = sa1;
    traj->sa2 = sa2;
    traj->sc = sc;
    traj->sof = sof;
}
void ScurveGenerator::generate(Trajectory* traj, double t) {
    unsigned int type = traj->type;

    int k1 = traj->k[type][0];
    int k2 = traj->k[type][1];
    //PARAMETER SET
    double vo = traj->vo;
    double vf = traj->vf;
    double vp = traj->vp;
    double vmax = traj->vmax;
    double ao = traj->ao;
    double af = traj->af;
    double a1 = traj->a1;
    double a2 = traj->a2;
    double amax = traj->amax;
    double dmax = traj->dmax;
    double so = traj->so;
    double sf = traj->sf;
    double sof = sf - so;
    double j = traj->j;

    double t0 = 0;
    double t1 = traj->tj1;
    double t2 = traj->ta1 - traj->tj2;
    double t3 = traj->ta1;
    double t4 = traj->tt - traj->ta2;
    double t5 = traj->tt - traj->ta2 + traj->tj3;
    double t6 = traj->tt - traj->tj4;
    double t7 = traj->tt;
    double st[6];
    double dst[6];

    for (int i = 0; i < 6; i++) {
        st[i] = traj->st[i];
        dst[i] = traj->dst[i];
    }

    double s = 0;
    double ds = 0;
    double dds = 0;
    double ddds = 0;
    double state = 0; // 그림 그리기 위한 용도
    if(type == UU1  || type == UU2){
        if (type == UU1) k1 =1;
        else k1=-1;

        if (t >= t0 && t < t1) {
            state = 0;
            s = 1.0 / 6.0 * j * t * t * t + 1.0 / 2.0 * ao * t * t + vo * t + so;
            ds = 1.0 / 2.0 * j * t * t + ao * t + vo;
            dds =  j * t + ao;
            ddds = j;
        }
        else if (t >= t1 && t < t2) {
            state = 1;
            s = 1.0 / 2.0 * k1 * a1 * (t - t1) * (t - t1) + dst[0] * (t - t1) + st[0];
            ds = k1 * a1 * (t - t1) + dst[0];
            dds = k1 * a1;
            ddds = 0;
        }
        else if (t >= t2 && t < t3) {
            state = 2;
            s = -1.0 / 6.0  * j * (t - t2) * (t - t2) * (t - t2) + 1.0 / 2.0 * k1 * a1 * (t - t2) * (t - t2) + dst[1] * (t - t2) + st[1];
            ds = -1.0 / 2.0  * j * (t - t2) * (t - t2) + k1 * a1 * (t - t2) + dst[1];
            dds = -j * (t - t2) + k1 * a1;
            ddds = -j;
        }
        else if (t >= t3 && t < t4) {
            state = 3;
            s = vp * (t - t3) + st[2];
            ds = vp;
            dds = 0;
            ddds = 0;
        }
        else if (t >= t4 && t < t5) {
            state = 4;
            s = -1.0 / 6.0 *  j * (t - t4) * (t - t4) * (t - t4) + dst[3] * (t - t4) + st[3];
            ds = -1.0 / 2.0 *  j * (t - t4) * (t - t4) + dst[3];
            dds = -j * (t - t4);
            ddds = -j;
        }

        else if (t >= t5 && t < t6) {
            state = 5;
            s = 1.0 / 2.0 * k1* a2 * (t - t5) * (t - t5) + dst[4] * (t - t5) + st[4];
            ds =  k1* a2 * (t - t5) + dst[4];
            dds =  k1* a2;
            ddds = 0;
        }

        else if (t >= t6 ) {
            state = 6;
            s = 1.0 / 6.0  * j * (t - t6) * (t - t6) * (t - t6) + 1.0 / 2.0 * k1* a2 * (t - t6) * (t - t6) + dst[5] * (t - t6) + st[5];
            ds = 1.0 / 2.0  * j * (t - t6) * (t - t6) + k1* a2 * (t - t6) + dst[5];
            dds =  j * (t - t6) + k1*a2;
            ddds = j;
        }

    }
 


    else{
        if (t >= t0 && t < t1) {
            state = 0;
            s = 1.0 / 6.0 * k1 * j * t * t * t + 1.0 / 2.0 * ao * t * t + vo * t + so;
            ds = 1.0 / 2.0 * k1 * j * t * t + ao * t + vo;
            dds = k1 * j * t + ao;
            ddds = k1 * j;
        }
        else if (t >= t1 && t < t2) {
            state = 1;
            s = 1.0 / 2.0 * k1 * a1 * (t - t1) * (t - t1) + dst[0] * (t - t1) + st[0];
            ds = k1 * a1 * (t - t1) + dst[0];
            dds = k1 * a1;
            ddds = 0;
        }
        else if (t >= t2 && t < t3) {
            state = 2;
            s = -1.0 / 6.0 * k1 * j * (t - t2) * (t - t2) * (t - t2) + 1.0 / 2.0 * k1 * a1 * (t - t2) * (t - t2) + dst[1] * (t - t2) + st[1];
            ds = -1.0 / 2.0 * k1 * j * (t - t2) * (t - t2) + k1 * a1 * (t - t2) + dst[1];
            dds = -k1 * j * (t - t2) + k1 * a1;
            ddds = -k1 * j;
        }
        else if (t >= t3 && t < t4) {
            state = 3;
            s = vp * (t - t3) + st[2];
            ds = vp;
            dds = 0;
            ddds = 0;
        }
        else if (t >= t4 && t < t5) {
            state = 4;
            s = 1.0 / 6.0 * k2 * j * (t - t4) * (t - t4) * (t - t4) + dst[3] * (t - t4) + st[3];
            ds = 1.0 / 2.0 * k2 * j * (t - t4) * (t - t4) + dst[3];
            dds = k2 * j * (t - t4);
            ddds = k2 * j;
        }

        else if (t >= t5 && t < t6) {
            state = 5;
            s = 1.0 / 2.0 * k2 * a2 * (t - t5) * (t - t5) + dst[4] * (t - t5) + st[4];
            ds = k2 * a2 * (t - t5) + dst[4];
            dds = k2 * a2;
            ddds = 0;
        }

        else if (t >= t6) {
            state = 6;
            s = -1.0 / 6.0 * k2 * j * (t - t6) * (t - t6) * (t - t6) + 1.0 / 2.0 * k2 * a2 * (t - t6) * (t - t6) + dst[5] * (t - t6) + st[5];
            ds = -1.0 / 2.0 * k2 * j * (t - t6) * (t - t6) + k2 * a2 * (t - t6) + dst[5];
            dds = -k2 * j * (t - t6) + k2 * a2;
            ddds = -k2 * j;
        }
    }

    traj->s = s;
    traj->ds = ds;
    traj->dds = dds;
    traj->ddds = ddds;
    traj->state = state;
}
Trajectory ScurveGenerator::getTraj(int idx) {
    return this->traj_list.at(idx);
}
vector<double> ScurveGenerator::pygenerate(Trajectory traj, double t) {
    unsigned int type = traj.type;

    int k1 = traj.k[type][0];
    int k2 = traj.k[type][1];
    //PARAMETER SET
    double vo = traj.vo;
    double vf = traj.vf;
    double vp = traj.vp;
    double vmax = traj.vmax;
    double ao = traj.ao;
    double af = traj.af;
    double a1 = traj.a1;
    double a2 = traj.a2;
    double amax = traj.amax;
    double dmax = traj.dmax;
    double so = traj.so;
    double sf = traj.sf;
    double sof = sf - so;
    double j = traj.j;

    double t0 = 0;
    double t1 = traj.tj1;
    double t2 = traj.ta1 - traj.tj2;
    double t3 = traj.ta1;
    double t4 = traj.tt - traj.ta2;
    double t5 = traj.tt - traj.ta2 + traj.tj3;
    double t6 = traj.tt - traj.tj4;
    double t7 = traj.tt;
    double st[6];
    double dst[6];

    for (int i = 0; i < 6; i++) {
        st[i] = traj.st[i];
        dst[i] = traj.dst[i];
    }

    double s = 0;
    double ds = 0;
    double dds = 0;
    double ddds = 0;
    double state = 0; // 그림 그리기 위한 용도
    if(type == UU1 || type == UU2){
        if (type == UU1) k1 =1;
        else k1=-1;

        if (t >= t0 && t < t1) {
            state = 0;
            s = 1.0 / 6.0 * j * t * t * t + 1.0 / 2.0 * ao * t * t + vo * t + so;
            ds = 1.0 / 2.0 * j * t * t + ao * t + vo;
            dds =  j * t + ao;
            ddds = j;
        }
        else if (t >= t1 && t < t2) {
            state = 1;
            s = 1.0 / 2.0 * k1 * a1 * (t - t1) * (t - t1) + dst[0] * (t - t1) + st[0];
            ds = k1 * a1 * (t - t1) + dst[0];
            dds = k1 * a1;
            ddds = 0;
        }
        else if (t >= t2 && t < t3) {
            state = 2;
            s = -1.0 / 6.0  * j * (t - t2) * (t - t2) * (t - t2) + 1.0 / 2.0 * k1 * a1 * (t - t2) * (t - t2) + dst[1] * (t - t2) + st[1];
            ds = -1.0 / 2.0  * j * (t - t2) * (t - t2) + k1 * a1 * (t - t2) + dst[1];
            dds = -j * (t - t2) + k1 * a1;
            ddds = -j;
        }
        else if (t >= t3 && t < t4) {
            state = 3;
            s = vp * (t - t3) + st[2];
            ds = vp;
            dds = 0;
            ddds = 0;
        }
        else if (t >= t4 && t < t5) {
            state = 4;
            s = -1.0 / 6.0 *  j * (t - t4) * (t - t4) * (t - t4) + dst[3] * (t - t4) + st[3];
            ds = -1.0 / 2.0 *  j * (t - t4) * (t - t4) + dst[3];
            dds = -j * (t - t4);
            ddds = -j;
        }

        else if (t >= t5 && t < t6) {
            state = 5;
            s = 1.0 / 2.0 * k1* a2 * (t - t5) * (t - t5) + dst[4] * (t - t5) + st[4];
            ds =  k1* a2 * (t - t5) + dst[4];
            dds =  k1* a2;
            ddds = 0;
        }

        else if (t >= t6 ) {
            state = 6;
            s = 1.0 / 6.0  * j * (t - t6) * (t - t6) * (t - t6) + 1.0 / 2.0 * k1* a2 * (t - t6) * (t - t6) + dst[5] * (t - t6) + st[5];
            ds = 1.0 / 2.0  * j * (t - t6) * (t - t6) + k1* a2 * (t - t6) + dst[5];
            dds =  j * (t - t6) + k1*a2;
            ddds = j;
        }

    }
  

    else{
        if (t >= t0 && t < t1) {
            state = 0;
            s = 1.0 / 6.0 * k1 * j * t * t * t + 1.0 / 2.0 * ao * t * t + vo * t + so;
            ds = 1.0 / 2.0 * k1 * j * t * t + ao * t + vo;
            dds = k1 * j * t + ao;
            ddds = k1 * j;
        }
        else if (t >= t1 && t < t2) {
            state = 1;
            s = 1.0 / 2.0 * k1 * a1 * (t - t1) * (t - t1) + dst[0] * (t - t1) + st[0];
            ds = k1 * a1 * (t - t1) + dst[0];
            dds = k1 * a1;
            ddds = 0;
        }
        else if (t >= t2 && t < t3) {
            state = 2;
            s = -1.0 / 6.0 * k1 * j * (t - t2) * (t - t2) * (t - t2) + 1.0 / 2.0 * k1 * a1 * (t - t2) * (t - t2) + dst[1] * (t - t2) + st[1];
            ds = -1.0 / 2.0 * k1 * j * (t - t2) * (t - t2) + k1 * a1 * (t - t2) + dst[1];
            dds = -k1 * j * (t - t2) + k1 * a1;
            ddds = -k1 * j;
        }
        else if (t >= t3 && t < t4) {
            state = 3;
            s = vp * (t - t3) + st[2];
            ds = vp;
            dds = 0;
            ddds = 0;
        }
        else if (t >= t4 && t < t5) {
            state = 4;
            s = 1.0 / 6.0 * k2 * j * (t - t4) * (t - t4) * (t - t4) + dst[3] * (t - t4) + st[3];
            ds = 1.0 / 2.0 * k2 * j * (t - t4) * (t - t4) + dst[3];
            dds = k2 * j * (t - t4);
            ddds = k2 * j;
        }

        else if (t >= t5 && t < t6) {
            state = 5;
            s = 1.0 / 2.0 * k2 * a2 * (t - t5) * (t - t5) + dst[4] * (t - t5) + st[4];
            ds = k2 * a2 * (t - t5) + dst[4];
            dds = k2 * a2;
            ddds = 0;
        }

        else if (t >= t6) {
            state = 6;
            s = -1.0 / 6.0 * k2 * j * (t - t6) * (t - t6) * (t - t6) + 1.0 / 2.0 * k2 * a2 * (t - t6) * (t - t6) + dst[5] * (t - t6) + st[5];
            ds = -1.0 / 2.0 * k2 * j * (t - t6) * (t - t6) + k2 * a2 * (t - t6) + dst[5];
            dds = -k2 * j * (t - t6) + k2 * a2;
            ddds = -k2 * j;
        }
    }

    traj.s = s;
    traj.ds = ds;
    traj.dds = dds;
    traj.ddds = ddds;
    traj.state = state;
    vector<double> ret;
    ret.push_back(s);
    ret.push_back(ds);
    ret.push_back(dds);
    ret.push_back(ddds);
    ret.push_back(state);
    return ret;
}
void ScurveGenerator::setTraj(Trajectory* traj, double so, double sf, double vo, double vf, double ao, double af, double vmax, double amax, double dmax, double  j, unsigned int type) {
    traj->vo = vo;
    traj->vf = vf;
    traj->ao = ao;
    traj->af = af;
    traj->vmax = vmax;
    traj->amax = amax;
    traj->dmax = dmax;
    traj->j = j;
    traj->so = so;
    traj->sf = sf;
    traj->type = type;
    traj->vp = vmax;
    traj->a1 = amax;
    traj->a2 = dmax;
}
void ScurveGenerator::U2DminTimeTraj(Trajectory *traj,Trajectory &ret_traj){
        unsigned int type =U2D;
        double k1 = 1;
        double k2 = -1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;        
        double a1_upper_limit = amax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = dmax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;     
        double vp_upper_limit = vmax;
        double a1_lower_vp = (k1 * ao >= 0) ? vo + k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_lower_vp = (k2 * af >= 0) ? vf + k2 * abs(af) * af / 2.0 / j : vf;
        double vp_lower_limit = max(a1_lower_vp, a2_lower_vp);
        double vp = vp_upper_limit;
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = (SQRT2*sqrt(af*af + 2*j*k2*vf - 2*j*k2*vp))/SQRT2;
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;

        double new_vp = vp;

        for (int iter = 0; iter < MAXITER; iter++) {
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            generate(traj);
            double dscvp = (vp * (a1 * k2 - a2 * k1)) / (a1 * a2) - (a1 + a2) / (2 * j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            new_vp = vp -(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            if(isnan(new_vp)) new_vp = vp_lower_limit;
            if(new_vp >vp_upper_limit) new_vp = vp_upper_limit;

             a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
            if(a1>a1_upper_limit)a1=a1_upper_limit;
            if(a1<a1_lower_limit)a1=a1_lower_limit;

             a2 = (SQRT2*sqrt(af*af + 2*j*k2*vf - 2*j*k2*vp))/SQRT2;
            if(a2>a2_upper_limit)a2=a2_upper_limit;
            if(a2<a2_lower_limit)a2=a2_lower_limit;
                if (abs(vp - new_vp) < TOLERANCE) {
                    break;
                }
                vp = new_vp;
            }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;

        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::U2DmaxTimeTraj(Trajectory *traj,Trajectory &ret_traj){
        unsigned int type =U2D;
        double k1 = 1;
        double k2 = -1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;        
        double a1_upper_limit = amax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = dmax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;     
        double vp_upper_limit = vmax;
        double a1_lower_vp = (k1 * ao >= 0) ? vo + k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_lower_vp = (k2 * af >= 0) ? vf + k2 * abs(af) * af / 2.0 / j : vf;
        double vp_lower_limit = max(a1_lower_vp, a2_lower_vp);
        double vp = vp_lower_limit;
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;

        double new_vp = vp;

        for (int iter = 0; iter < MAXITER; iter++) {
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            generate(traj);
            double dscvp = (vp * (a1 * k2 - a2 * k1)) / (a1 * a2) - (a1 + a2) / (2 * j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            new_vp = vp +(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            if(isnan(new_vp)){
                new_vp = vp_lower_limit;
                 a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
                if(a1>a1_upper_limit)a1=a1_upper_limit;
                if(a1<a1_lower_limit)a1=a1_lower_limit;

                 a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
                if(a2>a2_upper_limit)a2=a2_upper_limit;
                if(a2<a2_lower_limit)a2=a2_lower_limit;
                break;
            };
            if(new_vp >vp_upper_limit) new_vp = vp_upper_limit;

             a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
            if(a1>a1_upper_limit)a1=a1_upper_limit;
            if(a1<a1_lower_limit)a1=a1_lower_limit;

             a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
            if(a2>a2_upper_limit)a2=a2_upper_limit;
            if(a2<a2_lower_limit)a2=a2_lower_limit;

                if (abs(vp - new_vp) < TOLERANCE) {
                    break;
                }
                vp = new_vp;
            }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;

        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::D2UminTimeTraj(Trajectory *traj,Trajectory &ret_traj){
        unsigned int type =D2U;
        traj->type = type;
        double k1 = -1;
        double k2 = 1;

        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;       
        double D2Ua1upperlimit = traj->a1_upper_limits[type];
        double D2Ua1lowerlimit = traj->a1_lower_limits[type];
        cout<<"D2Ua1lowerlimit : "<<D2Ua1lowerlimit <<endl;
        double D2Ua2upperlimit = traj->a2_upper_limits[type];
        double D2Ua2lowerlimit = traj->a2_lower_limits[type];     
        double vp =traj->vp_upper_limits[type];
        cout<<vp<<endl;

        double a1 = sqrt(ao * ao + 2 * k1 * j * (vp - vo)) / SQRT2;
        if( a1 > dmax) a1 = dmax;
           cout<<a1<<endl;
        if(a1>D2Ua1upperlimit)a1=D2Ua1upperlimit;
       if(a1<D2Ua1lowerlimit)a1=D2Ua1lowerlimit;

        double a2 = sqrt(af * af - 2 * k2 * j * (vp - vf)) / SQRT2;
        if( a2 > amax) a2 = amax;
             cout<<a2<<endl;
        if(a2>D2Ua2upperlimit)a2=D2Ua2upperlimit;
        if(a2<D2Ua2lowerlimit)a2=D2Ua2lowerlimit;

        double new_vp = vp;
        for (int  i=0;i<MAXITER;i++){
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            generate(traj);
            double dscvp = (k2*vp)/a2 - (a1*a1 + a2*a1 + 2*j*k1*vp)/(2*a1*j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            new_vp = vp -(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            if(isnan(new_vp)){ cout<<"isnan"<<endl;break;}
            if(new_vp > traj->vp_upper_limits[type])new_vp = traj->vp_upper_limits[type];
            if(new_vp < traj->vmin) new_vp = traj->vmin;
            cout<<i << "--" <<"vp : "<<vp<<endl;
            cout<<i << "--" <<"new_vp : "<<new_vp<<endl;
            cout<<i << "--" <<"sc : "<<sc<<endl;

            a1 = sqrt((ao * ao + 2 * k1 * j * (new_vp - vo)) )/ SQRT2;
            //a1 = -sqrt(2*(ao*ao - 2*j*k1*vo + 2*j*k1*new_vp))/2.0;
            //a1 = sqrt(2*(ao*ao - 2*j*k1*vo + 2*j*k1*new_vp))/2.0;
           // if( a1 > dmax) a1 = dmax;
            if(a1>D2Ua1upperlimit)a1=D2Ua1upperlimit;
            if(a1<D2Ua1lowerlimit)a1=D2Ua1lowerlimit;

            a2 = sqrt((af * af - 2 * k2 * j * (new_vp - vf)) )/ SQRT2;
            if(a2>D2Ua2upperlimit)a2=D2Ua2upperlimit;
            if(a2<D2Ua2lowerlimit)a2=D2Ua2lowerlimit;
            //if( a2 > amax) a2 = amax;
            if (abs(vp - new_vp) < TOLERANCE) {
                break;
            }
            vp = new_vp;
        }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;
        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::D2UmaxTimeTraj(Trajectory *traj,Trajectory &ret_traj){
        unsigned int type =D2U;
        traj->type = type;
        double k1 = -1;
        double k2 = 1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;       
        double a1_upper_limit = k1 >= 0 ? amax : dmax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = k2 >= 0 ? amax : dmax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;
        double vp =traj->vp_lower_limits[type];
        cout<<vp<<endl;

         double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

         double  a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;

        double new_vp = vp;
        for (int  i=0;i<MAXITER;i++){
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            generate(traj);
            double dscvp = (k2*vp)/a2 - (a1*a1 + a2*a1 + 2*j*k1*vp)/(2*a1*j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            new_vp = vp +(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            if(isnan(new_vp)){ cout<<"isnan"<<endl;break;}
            if(new_vp > traj->vp_upper_limits[type])new_vp = traj->vp_upper_limits[type];
            if(new_vp < traj->vmin) {
                new_vp = traj->vmin;

                  a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
                if(a1>a1_upper_limit)a1=a1_upper_limit;
                if(a1<a1_lower_limit)a1=a1_lower_limit;

                   a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
                if(a2>a2_upper_limit)a2=a2_upper_limit;
                if(a2<a2_lower_limit)a2=a2_lower_limit;
                break;
            }


              a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
            if(a1>a1_upper_limit)a1=a1_upper_limit;
            if(a1<a1_lower_limit)a1=a1_lower_limit;

               a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
            if(a2>a2_upper_limit)a2=a2_upper_limit;
            if(a2<a2_lower_limit)a2=a2_lower_limit;

            //if( a2 > amax) a2 = amax;
            if (abs(vp - new_vp) < TOLERANCE) {
                break;
            }
            vp = new_vp;
        }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;
        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::U2UminTimeTraj1(Trajectory *traj,Trajectory &ret_traj){
        if(traj->vf < traj->vo) {
            cout<<"U2U ERROR"<<endl;
            memcpy(&ret_traj,traj,sizeof(ret_traj));
            return;
        }    
        unsigned int type =U2U;
        double k1 = 1;
        double k2 = 1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;        
        double a1_upper_limit = amax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = amax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;     
        double a1_upper_vp = (k1 * ao >= 0) ? vo - k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_upper_vp = (k2 * af >= 0) ? vf - k2 * abs(af) * af / 2.0 / j : vf;        
        double vp_upper_limit = a2_upper_vp;
        double a1_lower_vp = (k1 * ao >= 0) ? vo + k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_lower_vp = (k2 * af >= 0) ? vf + k2 * abs(af) * af / 2.0 / j : vf;
        double vp_lower_limit =a1_lower_vp;
        double vp = vp_upper_limit;
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;

        double new_vp = vp;
        cout<<new_vp<<endl;
        for (int iter = 0; iter < MAXITER; iter++) {
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            traj->type = U2U;
            generate(traj);
            double dscvp = (vp * (a1 * k2 - a2 * k1)) / (a1 * a2) - (a1 + a2) / (2 * j);
            double dttvp = (ao*ao - 2*j*vo + 2*j*vp)/(2*a1*j) + (a1*a2 - 2*a2*af - 2*a2*ao + 2*j*vf - 2*j*vp + a2*a2 + af*af)/(2*a2*j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            double tt = traj->tt;
           // new_vp = vp+ (tt)/dttvp;
           new_vp = vp +(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            cout<<new_vp<<endl;
            if(isnan(new_vp)){
                new_vp = vp_lower_limit;
                 a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
                if(a1>a1_upper_limit)a1=a1_upper_limit;
                if(a1<a1_lower_limit)a1=a1_lower_limit;

                 a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
                if(a2>a2_upper_limit)a2=a2_upper_limit;
                if(a2<a2_lower_limit)a2=a2_lower_limit;
                break;
            };
            if(new_vp >vp_upper_limit) new_vp = vp_upper_limit;
            if(new_vp <vp_lower_limit) new_vp = vp_lower_limit;
             a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
            if(a1>a1_upper_limit)a1=a1_upper_limit;
            if(a1<a1_lower_limit)a1=a1_lower_limit;

             a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
            if(a2>a2_upper_limit)a2=a2_upper_limit;
            if(a2<a2_lower_limit)a2=a2_lower_limit;

                if (abs(vp - new_vp) < TOLERANCE && sc>-EPS) {
                    break;
                }
                vp = new_vp;
            }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;
        traj->type = U2U;

        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::U2UminTimeTraj2(Trajectory *traj,Trajectory &ret_traj){
        if(traj->vf < traj->vo) {
            cout<<"U2U ERROR"<<endl;
            memcpy(&ret_traj,traj,sizeof(ret_traj));
            return;
        }    
        unsigned int type =U2U;
        double k1 = 1;
        double k2 = 1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;        
        double a1_upper_limit = amax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = amax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;     
        double a1_upper_vp = (k1 * ao >= 0) ? vo - k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_upper_vp = (k2 * af >= 0) ? vf - k2 * abs(af) * af / 2.0 / j : vf;        
        double vp_upper_limit = a2_upper_vp;
        double a1_lower_vp = (k1 * ao >= 0) ? vo + k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_lower_vp = (k2 * af >= 0) ? vf + k2 * abs(af) * af / 2.0 / j : vf;
        double vp_lower_limit =a1_lower_vp;
        double vp = vp_lower_limit;
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;

        double new_vp = vp;
        cout<<new_vp<<endl;
        for (int iter = 0; iter < MAXITER; iter++) {
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            traj->type = U2U;
            generate(traj);
            double dscvp = (vp * (a1 * k2 - a2 * k1)) / (a1 * a2) - (a1 + a2) / (2 * j);
            double dttvp = (ao*ao - 2*j*vo + 2*j*vp)/(2*a1*j) + (a1*a2 - 2*a2*af - 2*a2*ao + 2*j*vf - 2*j*vp + a2*a2 + af*af)/(2*a2*j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            double tt = traj->tt;
           // new_vp = vp+ (tt)/dttvp;
           new_vp = vp -(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            cout<<new_vp<<endl;
            if(isnan(new_vp)){
                new_vp = vp_lower_limit;
                 a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
                if(a1>a1_upper_limit)a1=a1_upper_limit;
                if(a1<a1_lower_limit)a1=a1_lower_limit;

                 a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
                if(a2>a2_upper_limit)a2=a2_upper_limit;
                if(a2<a2_lower_limit)a2=a2_lower_limit;
                break;
            };
            if(new_vp >vp_upper_limit) new_vp = vp_upper_limit;
            if(new_vp <vp_lower_limit) new_vp = vp_lower_limit;
             a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
            if(a1>a1_upper_limit)a1=a1_upper_limit;
            if(a1<a1_lower_limit)a1=a1_lower_limit;

             a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
            if(a2>a2_upper_limit)a2=a2_upper_limit;
            if(a2<a2_lower_limit)a2=a2_lower_limit;

                if (abs(vp - new_vp) < TOLERANCE && sc>-EPS) {
                    break;
                }
                vp = new_vp;
            }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;
        traj->type = U2U;

        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::U2UmaxTimeTraj1(Trajectory *traj,Trajectory &ret_traj){
        if(traj->vf < traj->vo) {
            cout<<"U2U ERROR"<<endl;
            memcpy(&ret_traj,traj,sizeof(ret_traj));
            return;
        }    
        unsigned int type =U2U;
        double k1 = 1;
        double k2 = 1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;        
        double a1_upper_limit = amax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = amax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;     
        double a1_upper_vp = (k1 * ao >= 0) ? vo - k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_upper_vp = (k2 * af >= 0) ? vf - k2 * abs(af) * af / 2.0 / j : vf;        
        double vp_upper_limit = a2_upper_vp;
        double a1_lower_vp = (k1 * ao >= 0) ? vo + k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_lower_vp = (k2 * af >= 0) ? vf + k2 * abs(af) * af / 2.0 / j : vf;
        double vp_lower_limit =a1_lower_vp;
        double vp = vp_upper_limit;
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;

        double new_vp = vp;
        cout<<new_vp<<endl;
        for (int iter = 0; iter < MAXITER; iter++) {
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            traj->type = U2U;
            generate(traj);
            double dscvp = (vp * (a1 * k2 - a2 * k1)) / (a1 * a2) - (a1 + a2) / (2 * j);
            double dttvp = (ao*ao - 2*j*vo + 2*j*vp)/(2*a1*j) + (a1*a2 - 2*a2*af - 2*a2*ao + 2*j*vf - 2*j*vp + a2*a2 + af*af)/(2*a2*j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            double tt = traj->tt;
            //new_vp = vp+(tt)/dttvp;
            new_vp = vp -(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            cout<<new_vp<<endl;
            if(isnan(new_vp)){
                new_vp = vp_lower_limit;
                 a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
                if(a1>a1_upper_limit)a1=a1_upper_limit;
                if(a1<a1_lower_limit)a1=a1_lower_limit;

                 a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
                if(a2>a2_upper_limit)a2=a2_upper_limit;
                if(a2<a2_lower_limit)a2=a2_lower_limit;
                break;
            };
            if(new_vp >vp_upper_limit) new_vp = vp_upper_limit;
            if(new_vp <vp_lower_limit) new_vp = vp_lower_limit;
             a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
            if(a1>a1_upper_limit)a1=a1_upper_limit;
            if(a1<a1_lower_limit)a1=a1_lower_limit;

             a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
            if(a2>a2_upper_limit)a2=a2_upper_limit;
            if(a2<a2_lower_limit)a2=a2_lower_limit;

                if (abs(vp - new_vp) < TOLERANCE) {
                    break;
                }
                vp = new_vp;
            }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;
        traj->type = U2U;
        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::U2UmaxTimeTraj2(Trajectory *traj,Trajectory &ret_traj){
        if(traj->vf < traj->vo) {
            cout<<"U2U ERROR"<<endl;
            memcpy(&ret_traj,traj,sizeof(ret_traj));
            return;
        }    
        unsigned int type =U2U;
        double k1 = 1;
        double k2 = 1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;        
        double a1_upper_limit = amax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = amax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;     
        double a1_upper_vp = (k1 * ao >= 0) ? vo - k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_upper_vp = (k2 * af >= 0) ? vf - k2 * abs(af) * af / 2.0 / j : vf;        
        double vp_upper_limit = a2_upper_vp;
        double a1_lower_vp = (k1 * ao >= 0) ? vo + k1 * abs(ao) * ao / 2.0 / j : vo;
        double a2_lower_vp = (k2 * af >= 0) ? vf + k2 * abs(af) * af / 2.0 / j : vf;
        double vp_lower_limit =a1_lower_vp;
        double vp = vp_lower_limit;
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;

        double new_vp = vp;
        cout<<new_vp<<endl;
        for (int iter = 0; iter < MAXITER; iter++) {
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            traj->type = U2U;
            generate(traj);
            double dscvp = (vp * (a1 * k2 - a2 * k1)) / (a1 * a2) - (a1 + a2) / (2 * j);
            double dttvp = (ao*ao - 2*j*vo + 2*j*vp)/(2*a1*j) + (a1*a2 - 2*a2*af - 2*a2*ao + 2*j*vf - 2*j*vp + a2*a2 + af*af)/(2*a2*j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            double tt = traj->tt;
            //new_vp = vp+(tt)/dttvp;
            new_vp = vp +(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            cout<<new_vp<<endl;
            if(isnan(new_vp)){
                new_vp = vp_lower_limit;
                 a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
                if(a1>a1_upper_limit)a1=a1_upper_limit;
                if(a1<a1_lower_limit)a1=a1_lower_limit;

                 a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
                if(a2>a2_upper_limit)a2=a2_upper_limit;
                if(a2<a2_lower_limit)a2=a2_lower_limit;
                break;
            };
            if(new_vp >vp_upper_limit) new_vp = vp_upper_limit;
            if(new_vp <vp_lower_limit) new_vp = vp_lower_limit;
             a1 = sqrt(abs(ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
            if(a1>a1_upper_limit)a1=a1_upper_limit;
            if(a1<a1_lower_limit)a1=a1_lower_limit;

             a2 = sqrt(af*af - 2*j*k2*(new_vp-vf))/sqrt(2);
            if(a2>a2_upper_limit)a2=a2_upper_limit;
            if(a2<a2_lower_limit)a2=a2_lower_limit;

                if (abs(vp - new_vp) < TOLERANCE) {
                    break;
                }
                vp = new_vp;
            }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;
        traj->type = U2U;
        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}
void ScurveGenerator::D2DminTimeTraj(Trajectory *traj,Trajectory &ret_traj){
        if(traj->vo < traj->vf) {
            cout<<"D2D ERROR"<<endl;
            memcpy(&ret_traj,traj,sizeof(ret_traj));
            return;
        }
        int type = D2D;
        traj->type = D2D;
        double k1 = -1;
        double k2 = -1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;     
        double a1_upper_limit = k1 >= 0 ? amax : dmax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = k2 >= 0 ? amax : dmax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;
        double vp = traj->vp_upper_limits[type];
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = vp;
        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}

void ScurveGenerator::D2DmaxTimeTraj(Trajectory *traj,Trajectory &ret_traj){
        if(traj->vo < traj->vf) {
            cout<<"D2D ERROR"<<endl;
            memcpy(&ret_traj,traj,sizeof(ret_traj));
            return;
        }
        int type = D2D;
        traj->type = D2D;
        double k1 = -1;
        double k2 = -1;
        double so = traj->so;
        double sf = traj->sf;
        double sof = sf - so;
        double vo = traj->vo;
        double vf = traj->vf;
        double ao = traj->ao;
        double af = traj->af;
        double vmax = traj->vmax;
        double amax = traj->amax;
        double dmax = traj->dmax;
        double vmin = traj->vmin;
        double j = traj->j;
        const double aoao = ao*ao;
        const double afaf = af*af;
        const double k1j = k1*j;
        const double k2j = k2*j;     
        double a1_upper_limit = k1 >= 0 ? amax : dmax;
        double a1_lower_limit = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        double a2_upper_limit = k2 >= 0 ? amax : dmax;
        double a2_lower_limit = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;
        double vp = traj->vp_upper_limits[type];
        double a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>a1_upper_limit)a1=a1_upper_limit;
        if(a1<a1_lower_limit)a1=a1_lower_limit;

        double a2 = sqrt(af*af - 2*j*k2*(vp-vf))/sqrt(2);
        if(a2>a2_upper_limit)a2=a2_upper_limit;
        if(a2<a2_lower_limit)a2=a2_lower_limit;
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = vp;
        generate(traj);
        checkError(traj);
        memcpy(&ret_traj,traj,sizeof(ret_traj));
}


void ScurveGenerator::update(Trajectory* traj) {
    unsigned int type = traj->type;
     double k1 = traj->k[type][0];
    double k2 = traj->k[type][1];
     double so = traj->so;
     double sf = traj->sf;
     double sof = sf - so;
     double vo = traj->vo;
     double vf = traj->vf;
     double ao = traj->ao;
     double af = traj->af;
     double vmax = traj->vmax;
     double amax = traj->amax;
     double dmax = traj->dmax;
     double vmin = traj->vmin;
     double j = traj->j;


    ////cout<<"D2U vp upper limit : "<<traj->vp_upper_limits[D2U]<<endl;
    const double aoao = ao*ao;
    const double afaf = af*af;
    const double k1j = k1*j;
    const double k2j = k2*j;

    if (type == U2D ) {
        double U2Da1upperlimit = traj->a1_upper_limits[type];
        double U2Da1lowerlimit = traj->a1_lower_limits[type];
        double U2Da2upperlimit = traj->a2_upper_limits[type];
        double U2Da2lowerlimit = traj->a2_lower_limits[type];        
        double vp = traj->vp_upper_limits[type];
        double a1;
        a1 = sqrt(abs(ao*ao + 2 * k1*j * (vp - vo))) / sqrt(2);
        if(a1>U2Da1upperlimit)a1=U2Da1upperlimit;
       // if(a1<U2Da1lowerlimit)a1 = U2Da1lowerlimit;

        double a2;
        a2 = (SQRT2*sqrt(af*af + 2*j*k2*vf - 2*j*k2*vp))/SQRT2;
        if(a2>U2Da2upperlimit)a2=U2Da2upperlimit;
        double new_vp = vp;

        for (int iter = 0; iter < MAXITER; iter++) {
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            generate(traj);
            double dscvp = (vp * (a1 * k2 - a2 * k1)) / (a1 * a2) - (a1 + a2) / (2 * j);
            double ddscvp =  k2/a2 - k1/a1;
            double sc  = traj->sc;
            new_vp = vp -(sc)/ (dscvp)/(1-sc*ddscvp/2.0/dscvp/dscvp );
            //new_vp= sqrt(3*(a1 + a2)*(3*a1*af*af*af*af + 3*a2*ao*ao*ao*ao + 3*a1*a1*a2*a2*a2 + 3*a1*a1*a1*a2*a2 + 6*a1*a2*a2*af*af + 6*a1*a1*a2*ao*ao + 12*a1*j*j*vf*vf + 12*a2*j*j*vo*vo + 8*a1*a2*af*af*af - 8*a1*a2*ao*ao*ao + 24*a1*a2*j*j*sf - 24*a1*a2*j*j*so - 12*a1*a2*a2*j*vf - 12*a1*a1*a2*j*vo - 12*a1*af*af*j*vf - 12*a2*ao*ao*j*vo - 24*a1*a2*af*j*vf + 24*a1*a2*ao*j*vo))/(6*j*(a1 + a2)) - (a1*a2)/(2*j);
            if(isnan(new_vp)) new_vp = traj->vp_lower_limits[type];

            //if(traj->tca1 <-EPS  || traj->tca2 <-EPS ) break;

            
            //new_vp = rectifier(new_vp, traj->vp_upper_limits[type], traj->vp_lower_limits[type]);
            if(new_vp > traj->vp_upper_limits[type]) new_vp =  traj->vp_upper_limits[type];
            //if(new_vp < traj->vp_lower_limits[type]) new_vp = traj->vp_lower_limits[type];
            a1 = sqrt((ao*ao + 2 * k1*j * (new_vp - vo))) / sqrt(2);
            if(a1>U2Da1upperlimit)a1=U2Da1upperlimit;
            a2 =sqrt((af*af - 2 * k2*j * (new_vp - vf))) / sqrt(2);
            if(a2>U2Da2upperlimit)a2=U2Da2upperlimit;

            //if ((2*amax*amax-aoao)/2.0/k1j + vo < new_vp)a1 = amax;
            //else a1 = sqrt(abs(aoao + 2 * k1j * (vp - vo))) / SQRT2;
            //if (-(2*dmax*dmax-afaf)/2.0/k2j + vf < new_vp) a2 = dmax;
            //else  a2 = sqrt(abs(afaf - 2 * k2j * (vp - vf))) / SQRT2;
            

            if (abs(vp - new_vp) < TOLERANCE) {
                break;
            }
            vp = new_vp;
        }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = new_vp;

        generate(traj);
        checkError(traj);

      return;
    }    
    else if(type==D2U){
        double vp = -vmax;
        k1 =-1.0;
        k2 = 1.0;        
        double a1 = sqrt(ao * ao + 2 * k1 * j * (vp - vo)) / SQRT2;
        if( a1 > dmax) a1 = dmax;
        double a2 = sqrt(af * af - 2 * k2 * j * (vp - vf)) / SQRT2;
        if( a2 > amax) a2 = amax;


        double new_vp = vp;
        for (int  i=0;i<MAXITER;i++){
            traj->vp = vp;
            traj->a1 = a1;
            traj->a2 = a2;
            generate(traj);
            double sc = traj->sc;
            double dscdvp = (3*a1*a2*a2 - a1*a1*a2 - 2*a1*af*af - 4*a1*j*vf + 6*a1*j*vp + 2*a2*j*vp)/(2*a1*a2*j);
            new_vp = vp-sc/dscdvp;
            if(isnan(vp)) break;
            if(vp < traj->vmin) vp = traj->vmin;

            a1 = sqrt(ao * ao + 2 * k1 * j * (new_vp - vo)) / SQRT2;
            if( a1 > dmax) a1 = dmax;

            a2 = sqrt(af * af - 2 * k2 * j * (new_vp - vf)) / SQRT2;
            if( a2 > amax) a2 = amax;
            if(abs(vp-new_vp)<TOLERANCE ){
                traj->a1 = a1;
                traj->a2 = a2;
                traj->vp = vp;
                traj->type = DD;
                generate(traj);
                return;
            }
            vp = new_vp;


        }
    }
    else if(type==UU1 || type == UU2){
        double a1=traj->a1;
        double a2=traj->a2;
        if(type==UU1  ){
            k1 = 1.0;
            a2 = af;
            a1 = sqrt(2*a2*a2 - af*af + ao*ao + 2*j*(vf-vo))/SQRT2;    
            if(a1> amax)a1 = amax;
        }else{
            k1 = -1.0;
            a1 = ao;
            a2=sqrt(2*a1*a1 + af*af - ao*ao - 2*j*vf + 2*j*vo)/SQRT2;            
            if(a2> dmax)a2 = dmax;
        }
        for (int  i=0;i<100;i++){
            double tj1 = (k1*a1-ao)/j;
            double tca1 = (k1+1)/2.0*(2*a2*a2 - af*af - 2*a1*a1 + ao*ao + 2*j*(vf-vo))/(2*a1*j);
            double tj2 = (-k1*a2+k1*a1)/j;
            double ta1 = tj1+tca1+tj2;
            
            double tj3 = 0;
            double tca2 = -(k1-1)/2.0*(-(- 2*a1*a1 + 2*a2*a2 - af*af + ao*ao + 2*j*(vf-vo))/(2*a2*j));
            double tj4 = (-k1*a2+af)/j;
            double ta2 = tj3+tca2+tj4;
            double tc = 0;
            double tt = tj1+tca1+tj2+tc+tj3+tca2+tj4;

            double vj1 = 1.0/2.0*j*tj1*tj1+ao*tj1;
            double vca1 = k1*a1*tca1;    
            double vj2 = -1.0/2.0*j*tj2*tj2+k1*a1*tj2;
            double vj3 = 0;
            double vca2 = k1*a2*tca2;       
            double vj4 = 1.0/2.0*j*tj4*tj4+k1*a2*tj4;
            double sj1 = 1.0/6.0*j*tj1*tj1*tj1+1.0/2.0*ao*tj1*tj1+vo*tj1;
            double sca1 = k1*1.0/2.0*a1*tca1*tca1+(vo+vj1)*tca1;        
            double sj2 = -1.0/6.0*j*tj2*tj2*tj2+k1*1.0/2.0*a1*tj2*tj2+(vo+vj1+vca1)*tj2;
            double sa1 = sj1+sca1+sj2;
            double sc = 0;
            double sj3= 0;
            double sca2 = k1*1.0/2.0*a2*tca2*tca2+(vo+vj1+vca1+vj2)*tca2;
            double sj4 = 1.0/6.0*j*tj4*tj4*tj4+k1*1.0/2.0*a2*tj4*tj4+(vo+vj1+vca1+vj2+vj3+vca2)*tj4;  
            double s = sj1+sca1+sj2+sc+sj3+sca2+sj4+so-sf;
            if(type==UU1){
                double dsoda2 = -((a2 - a1)*(- 2*a2*a2 + a1*a2 + af*af - 2*j*vf))/(a1*j*j);   
                a2= a2-s/dsoda2;
                if(isnan(a2)) break;

                ////cout<<"UU a2 : "<<a2<<endl;                
                a1 = sqrt(2*a2*a2 - af*af + ao*ao + 2*j*(vf-vo))/SQRT2;

                if (abs(a1)> amax) a1 = amax;
               // if (a2<0) a2 =EPS;
                
            }else if(type==UU2){
                double dsoda1 = -((a1 - a2)*(- 2*a1*a1 + a2*a1 + ao*ao - 2*j*vo))/(a2*j*j);
                a1= a1-s/dsoda1;
                if(isnan(a1)) break;
                ////cout<<"UU a1 : "<<a1<<endl;
                a2=sqrt(2*a1*a1 + af*af - ao*ao - 2*j*vf + 2*j*vo)/SQRT2;            
                if (abs(a2)> dmax) a2 = dmax;       
                //if ((a1)< 0) a1 = EPS;
            }
            if (abs(s) < 1e-14){
                traj->a1 = a1;
                traj->a2 = a2;
                traj->type = type;
                generate(traj);
                return;

            }
        }   




    }
    else if(type==DD){
        double vp = -vmax;
        k1 =-1.0;
        k2 = 1.0;        
        double a1 = sqrt(ao * ao + 2 * k1 * j * (vp - vo)) / SQRT2;
        if( a1 > dmax) a1 = dmax;

        double a2 = sqrt(af * af - 2 * k2 * j * (vp - vf)) / SQRT2;
        if( a2 > amax) a1 = amax;

       // //cout<<"DD new_vp = " << vp<<endl;
       // //cout<<"DD a1 = " << a1<<endl;
       // //cout<<"DD a2 = " << a2<<endl;
        double new_vp = vp;
        for (int  i=0;i<100;i++){
            double tj1 = (a1-k1*ao)/j;
            double ta1 = (2*a1*a1+ao*ao+k1*(2*j*vp-2*j*vo-2*a1*ao))/(2*a1*j);
            double tj2 = a1/j;
            double tca1 = ta1-tj1-tj2;
            double tj3 = a2/j;
            double tj4 = (a2-k2*af)/j;
            double ta2 = (2*a2*a2+af*af+k2*(-2*j*vp+2*j*vf-2*a2*af))/(2*a2*j);
            double tca2 = ta2-tj3-tj4;

            double vtj1 = ao*tj1 + k1*1.0/2.0*j*tj1*tj1;
            double vtca1 = k1*a1*(ta1-tj1-tj2);
            double vtj2 = k1*a1*tj2-k1*1.0/2.0*j*tj2*tj2;

            double vtj3 = k2*1.0/2.0*j*tj3*tj3;
            double vtca2 = k2*a2*(ta2-tj3-tj4);
            double vtj4 = k2*a2*tj4-k2*1.0/2.0*j*tj4*tj4;

            double stj1 = k1*1.0/6.0*j*tj1*tj1*tj1+1.0/2.0*ao*tj1*tj1+vo*tj1;
            double stca1 = k1*1.0/2.0*a1*tca1*tca1+(vo+vtj1)*tca1;
            double stj2 = -k1*1.0/6.0*j*tj2*tj2*tj2+k1*1.0/2.0*a1*tj2*tj2+(vp-vtj2)*tj2;
            double stj3 = k2*1.0/6.0*j*tj3*tj3*tj3+(vp)*tj3;
            double stca2 = k2*1.0/2.0*a2*tca2*tca2+(vp+vtj3)*tca2;
            double stj4 = -k2*1.0/6.0*j*tj4*tj4*tj4+k2*1.0/2.0*a2*tj4*tj4+(vf-vtj4)*tj4;

            double ds = vtj1+vtca1+vtj2+vtj3+vtca2+vtj4+vo-vf;
            double s = stj1+stca1+stj2+stj3+stca2+stj4+so-sf;
            double sc = (sf-so) - (stj1+stca1+stj2+stj3+stca2+stj4);
            double tc = sc/vp;
            double dscdvp = (3*a1*a2*a2 - a1*a1*a2 - 2*a1*af*af - 4*a1*j*vf + 6*a1*j*vp + 2*a2*j*vp)/(2*a1*a2*j);
            new_vp = vp-sc/dscdvp;
            if(isnan(new_vp)) break;
            if(new_vp < traj->vmin) new_vp = traj->vmin;
          //cout<<i<<"--DD new_vp= " << vp<<endl;
          //  //cout<<i<<"--DD a1 = " << a1<<endl;
          //  //cout<<i<<"--DD a2 = " << a2<<endl;
          //  //cout<<i<<"--DD dscdvp = " << dscdvp<<endl;
            a1 = sqrt(ao * ao + 2 * k1 * j * (new_vp - vo)) / SQRT2;
            if( a1 > dmax) a1 = dmax;

            a2 = sqrt(af * af - 2 * k2 * j * (new_vp - vf)) / SQRT2;
            if( a2 > amax) a2 = amax;

            if(abs(sc)<1e-15 || abs(vp-new_vp)<TOLERANCE ){
                traj->a1 = a1;
                traj->a2 = a2;
                traj->vp = vp;
                traj->type = DD;
                generate(traj);
                return;
            }
            vp = new_vp;


        }
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = vp;
        traj->type = DD;
        generate(traj);
        return;        

    }


    else{
        double vp = traj->vp_upper_limits[type];
        double a1 = sqrt(ao * ao + 2 * k1 * j * (vp - vo)) / SQRT2;
        a1 = rectifier(a1, traj->a1_upper_limits[type], traj->a1_lower_limits[type]);
        double a2 = sqrt(af * af - 2 * k2 * j * (vp - vf)) / SQRT2;
        a2 = rectifier(a2, traj->a2_upper_limits[type], traj->a2_lower_limits[type]);
        traj->a1 = a1;
        traj->a2 = a2;
        traj->vp = vp;
        generate(traj);
        checkError(traj);
        return;
    }
}

void ScurveGenerator::setType(int num, int type){
    this->traj_list.at(num).type = type;
}
int  ScurveGenerator::findMinTrajType(Trajectory *traj){


   // //cout<<"findMinTrajType"<<endl;
    int typeList[6] = {0};
    if(traj->vf>traj->vo){
      typeList[0]=U2D;
      typeList[1]=UU1;
      typeList[2]=UU2;
      typeList[3]=U2U;
      typeList[4]=D2U;
      typeList[5]=DD;
      traj->err[D2D] = 999;

    }else{
      typeList[0]=U2D;
      typeList[1]=UU1;
      typeList[2]=UU2;
      typeList[3]=D2D;
      typeList[4]=D2U;
      typeList[5]=DD;
      traj->err[U2U] = 999;
    }

    for( int i= 0;i<=5;i++){
    auto start = chrono::steady_clock::now();
        traj->type = typeList[i];
        this->update(traj);

        this->generate(traj);   

        checkError(traj);
         auto end = chrono::steady_clock::now();

        if(traj->err[traj->type] > 0 )continue;    
        //cout<<"findMinTrajType : type : "<<traj->type<<endl;   
        
        return typeList[i];
    }

    return DD;

    

}
void ScurveGenerator::checkInitialError(Trajectory* traj) {
    unsigned int type = traj->type;
    double so = traj->so;
    double sf = traj->sf;
    double sof = sf - so;
    double vo = traj->vo;
    double vf = traj->vf;
    double ao = traj->ao;
    double af = traj->af;
    double vmax = traj->vmax;
    double vmin = traj->vmin;
    double amax = traj->amax;
    double dmax = traj->dmax;
    double j = traj->j;
    for (unsigned int type = U2D; type <= D2U; type++) {
        unsigned int errcode = 0;
        if (so >= sf || so < 0 || sf < 0) {
            errcode = INVALID_S;
        }
        if (vo > vmax || vo < 0) {
            errcode = INVALID_VO;
        }
        if (vf > vmax || vf < 0) {
            errcode = INVALID_VF;
        }
        if (j <= 0) {
            errcode = INVALID_J;
        }
        switch (type) {
        case U2D:
            if (abs(ao) > amax) {
                errcode = INVALID_AO;
            }
            if (abs(af) > dmax) {
                errcode = INVALID_AF;
            }
            if ((ao < 0 && vo - ao * ao / 2.0 / j < vmin) || (af > 0 && vf - af * af / 2.0 / j <vmin)) {
               // errcode=0;
                errcode = ERROR_MINUS_VELOCITY;
            }
            if (ao > 0 && vo + ao * ao / 2.0 / j > vmax) {
                errcode = ERROR_OVER_VELOCITY;
            }     

            break;
        case U2U:
            if (vo > vf) {
                errcode = INVALID_V;
            }
            if (abs(ao) > amax) {
                errcode = INVALID_AO;
            }
            if (abs(af) > amax) {
                errcode = INVALID_AF;
            }
            if (ao < 0 && vo -ao * ao / 2.0 / j <vmin ) {
                errcode = ERROR_MINUS_VELOCITY;
                //errcode=0;
            }
            if (af < 0 && vf + af * af / 2.0 / j > vmax) {
                errcode = ERROR_OVER_VELOCITY;
            }
            break;
        case D2D:
            if (vo < vf) {
                errcode = INVALID_V;
            }
            if (abs(ao) > dmax) {
                errcode = INVALID_AO;
            }
            if (abs(af) > dmax) {
                errcode = INVALID_AF;
            }
            if (ao > 0 && vo + ao * ao / 2.0 / j > vmax) {
                errcode = ERROR_OVER_VELOCITY;
            }
            if (af > 0 && vf -af * af / 2.0 / j<vmin ) {
                //errcode=0;
                errcode = ERROR_MINUS_VELOCITY;
            }
            break;
        case D2U:
            if (abs(ao) > dmax) {
                errcode = INVALID_AO;
            }
            if (abs(af) > amax) {
                errcode = INVALID_AF;
            }
            if ((ao > 0 && vo + ao * ao / 2.0 / j > vmax)
                || (af < 0 && vf + af * af / 2.0 / j > vmax)) {
                errcode = ERROR_OVER_VELOCITY;
            }
            if ((ao < 0 && vo -ao * ao / 2.0 / j <vmin )
                || (af > 0 && vf -af * af / 2.0 / j<vmin )) {
                //errcode = 0;
                errcode = ERROR_MINUS_VELOCITY;
            }
            break;
        }

        traj->init_err[type] = errcode;
    }
}
void ScurveGenerator::checkError(Trajectory* traj) {
    int errcode = 0;
    unsigned int type = traj->type;
    if (traj->tj1 < -ERR_EPS) errcode = MINUS_TJ1;    
    else if (traj->tj2 < -ERR_EPS) errcode = MINUS_TJ2;  
    else if (traj->tj3 < -ERR_EPS) errcode = MINUS_TJ3;  
    else if (traj->tj4 < -ERR_EPS) errcode = MINUS_TJ4;  
    else if (traj->tca1 < -ERR_EPS) errcode = MINUS_TCA1;  
    else if (traj->tca2 < -ERR_EPS) errcode = MINUS_TCA2;  
    else if (traj->tc < -ERR_EPS) errcode = MINUS_TC;  
    else if (traj->sj1 + traj->sj2 + traj->sj3 + traj->sj4 + traj->sca1 + traj->sca2 + traj->sc - traj->sof > EPS) errcode = DISPLACEMENT_ERROR; 
    else if (traj->vp < traj->vmin) errcode =VMIN_ERR;
    else if (traj->vp > traj->vmax) errcode =VMAX_ERR;


    double t0 = 0;
    double t1 = traj->tj1;
    double t2 = traj->ta1 - traj->tj2;
    double t3 = traj->ta1;
    double t4 = traj->tt - traj->ta2;
    double t5 = traj->tt - traj->ta2 + traj->tj3;
    double t6 = traj->tt - traj->tj4;
    double t7 = traj->tt;

    double k1 = traj->k[traj->type][0];
    double k2 = traj->k[traj->type][1];
    double vtj1_min = traj->vo -k1* traj->ao*traj->ao/2.0/traj->j;
    double vtca1_min = min(k1*traj->a1*(traj->tca1)+traj->dst[0],k1*traj->a1*(0)+traj->dst[0]);
    double vtj2_min = -1.0 / 2.0  *k1* traj->j * ((traj->j*t2 + k1*traj->a1)/traj->j - t2) * ((traj->j*t2 + k1*traj->a1)/traj->j -t2) + k1 * traj->a1 * ((traj->j*t2 + k1*traj->a1)/traj->j - t2) + traj->dst[1];
    double vtj3_min = traj->dst[3];
    double vtca2_min =min(k2 *  traj->a2 * (t5 - t5) + traj->dst[4],k2 *  traj->a2 * (t6 - t5) + traj->dst[4]);
    double vtj4_min = -1.0 / 2.0 * k2 * traj->j * (traj->a2/traj->j ) * (traj->a2/traj->j) + k2 * traj->a2 * (traj->a2/traj->j) + traj->dst[5];
    if(traj->type == UU1 || traj->type == UU2){
        vtj1_min = 1.0 / 2.0 * traj->j *(-traj->ao/traj->j)* (-traj->ao/traj->j) + traj->ao * (-traj->ao/traj->j) + traj->vo;
        vtca1_min =  traj->dst[0];
        vtj2_min = -1.0 / 2.0  * traj->j * ((k1*traj->a1+traj->j*t2)/traj->j - t2) * ((k1*traj->a1+traj->j*t2)/traj->j- t2) + k1 * traj->a1 * ((k1*traj->a1+traj->j*t2)/traj->j - t2) + traj->dst[1];
        traj->vp = traj->dst[3];
        vtj3_min = traj->dst[3];
        vtca2_min = traj->dst[4];
        vtj4_min = 1.0 / 2.0  * traj->j * ((traj->j*t6-k1*traj->a2)/traj->j - t6) * ((traj->j*t6-k1*traj->a2)/traj->j - t6) + k1* traj->a2 * ((traj->j*t6-k1*traj->a2)/traj->j - t6) + traj->dst[5];
    }
    //cout<<"============================================"<<endl;
    //cout << "type : "<<traj->type<<" vtj1_min : " << vtj1_min<<endl;
    //cout << "type : "<<traj->type<<" vtca1_min : " << vtca1_min<<endl;
    //cout << "type : "<<traj->type<<" vtj2_min : " << vtj2_min<<endl;
    //cout << "type : "<<traj->type<<" vp : " << traj->vp<<endl;
    //cout << "type : "<<traj->type<<" vtj3_min : " << vtj3_min<<endl;
    //cout << "type : "<<traj->type<<" vtca2_min : " << vtca2_min<<endl;
    //cout << "type : "<<traj->type<<" vtj4_min : " << vtj4_min<<endl;
    
    
    
    traj->err[type] = errcode;

    return;
}
void ScurveGenerator::setLimits(Trajectory* traj) {
    double so = traj->so;
    double sf = traj->sf;
    double sof = sf - so;
    double vo = traj->vo;
    double vf = traj->vf;
    double ao = traj->ao;
    double af = traj->af;
    double vmax = traj->vmax;
    double vmin = traj->vmin;
    double amax = traj->amax;
    double dmax = traj->dmax;
    double j = traj->j;
    for (unsigned int type = U2D; type <= D2U; type++) {
        double  a1_lower_vp ;
        double  a2_lower_vp ;
        double  a1_upper_vp ;
        double  a2_upper_vp ;
        // 유형별 k1과, k2를 불러옵니다. 예) U2D의 경우  k1 = 1, k2 = -1
        double k1 = traj->k[type][0];
        double k2 = traj->k[type][1];
        // k1이 양수인 경우 amax 아닌경우 dmax
        traj->a1_upper_limits[type] = k1 >= 0 ? amax : dmax;
        // k2가 양수인 경우 amax 아닌경우 dmax
        traj->a2_upper_limits[type] = k2 >= 0 ? amax : dmax;
        // k1과 ao의 부호가 같은 경우 |ao| 아닌경우 |ao|/sqrt(2)
        traj->a1_lower_limits[type] = (k1 * ao >= 0) ? abs(ao) : abs(ao) / SQRT2;
        traj->a1_lower_limits[type] = abs(traj->a1_lower_limits[type]) < EPS ? EPS : traj->a1_lower_limits[type];
        //traj->a1_upper_limits[D2U] = dmax;
        //traj->a1_lower_limits[D2U] = EPS;
        // k2와 af의 부호가 같은 경우 |af| 아닌경우 |af|/sqrt(2)
        traj->a2_lower_limits[type] = (k2 * af >= 0) ? abs(af) : abs(af) / SQRT2;
        traj->a2_lower_limits[type] = abs(traj->a2_lower_limits[type]) < EPS ? EPS : traj->a2_lower_limits[type];
        //traj->a2_upper_limits[D2U] = amax;
        //traj->a2_lower_limits[D2U] = EPS;
        // vp_upper_limits과 vp_lower_limits를 구하기 위해 각 가속도별 상한 하한 vp를 계산
        // 부호가 같은경우와 부호가 다른경우로 계산
        a1_lower_vp = (k1 * ao >= 0) ? vo + k1 * abs(ao) * ao / 2.0 / j : vo;
        a1_upper_vp = (k1 * ao >= 0) ? vo - k1 * abs(ao) * ao / 2.0 / j : vo;
        a2_lower_vp = (k2 * af >= 0) ? vf + k2 * abs(af) * af / 2.0 / j : vf;
        a2_upper_vp = (k2 * af >= 0) ? vf - k2 * abs(af) * af / 2.0 / j : vf;
        // 유형별  vp_upper_limits과 vp_lower_limits 를 계산
        if (type == U2D) {
            traj->vp_lower_limits[type] = max(a1_lower_vp, a2_lower_vp);
            //traj->vp_lower_limits[type] = 0.0;
            traj->vp_upper_limits[type] = vmax;

        }
        else if (type == U2U) {
            traj->vp_lower_limits[type] = a1_lower_vp;
            traj->vp_upper_limits[type] = a2_upper_vp;
        }
        else if (type == D2D) {
            traj->vp_lower_limits[type] = a2_lower_vp;
            traj->vp_upper_limits[type] = a1_upper_vp;
        }
        else if (type == D2U) {
            traj->vp_lower_limits[type] = vmin;
            traj->vp_upper_limits[type] = min(a1_upper_vp, a2_upper_vp);
        }
    }
}
void ScurveGenerator::update_(int num, int type){
    this->traj_list.at(num).type = type;
     update(&this->traj_list.at(num));
}
void ScurveGenerator::setMinTime(Trajectory* traj) {
    Trajectory temp_traj;
    for (unsigned int type = U2D; type <= D2U; type++) {
        memcpy(&temp_traj, traj, sizeof(temp_traj));
        temp_traj.type = type;
        generate(&temp_traj);
        update(&temp_traj);
        checkError(&temp_traj);
        traj->min_time[type] = rectifier(temp_traj.tt, INF, -INF);
        traj->err[type] = temp_traj.err[type];
    }
}
// void ScurveGenerator::U2DMaxTime(Trajectory *traj){

// }
// void ScurveGenerator::U2UMaxTime(Trajectory *traj){
// }
// void ScurveGenerator::D2DMaxTime(Trajectory *traj){
// }
// void ScurveGenerator::D2UMaxTime(Trajectory *traj){
// }

// void ScurveGenerator::U2DMinTime(Trajectory *traj){

// }
// void ScurveGenerator::U2UMinTime(Trajectory *traj){
// }
// void ScurveGenerator::D2DMinTime(Trajectory *traj){
// }
// void ScurveGenerator::D2UMinTime(Trajectory *traj){
// }


void ScurveGenerator::setMaxTime(Trajectory* traj) {
    Trajectory temp_traj;
    memcpy(&temp_traj, traj, sizeof(temp_traj));
    for (unsigned int type = U2D; type <= D2U; type++) {
        temp_traj.type = type;
        temp_traj.vp = temp_traj.vp_lower_limits[type];
        double k1 = temp_traj.k[type][0];
        double k2 = temp_traj.k[type][1];
        temp_traj.a1 = rectifier(sqrt(temp_traj.ao * temp_traj.ao + 2 * k1 * temp_traj.j * (temp_traj.vp - temp_traj.vo)) / SQRT2, traj->a1_upper_limits[type], traj->a1_lower_limits[type]);
        temp_traj.a2 = rectifier(sqrt(temp_traj.af * temp_traj.af - 2 * k2 * temp_traj.j * (temp_traj.vp - temp_traj.vf)) / SQRT2, traj->a2_upper_limits[type], traj->a2_lower_limits[type]);
        generate(&temp_traj);
        checkError(&temp_traj);
        traj->max_time[type] = rectifier(temp_traj.tt, INF, -INF);
        traj->err[type] = temp_traj.err[type];
        
    }
}
void ScurveGenerator::getType(Trajectory* traj, double& T) {
    /*
    int err_val =1;
    double min_time_val = 99999.0;
    int min_time_type = 0;
    for(int type = U2D;type<=D2U;type++){
       err_val*= traj->min_time_err[type]*traj->max_time_err[type];
       if(traj->min_time[type]>0){
            if(traj->min_time[type] < min_time_val){
                min_time_val = traj->min_time[type];
                min_time_type = type;
            }
       }
       if(traj->max_time[type]>0){
            if(traj->max_time[type] < min_time_val){
                min_time_val = traj->max_time[type];
                min_time_type = type;
            }
       }       
       //cout<<"min_time_val : "<<min_time_val<<endl;
    }
    if(err_val >0){
        T = min_time_val;
        traj->type = min_time_type;
        //cout<<"dddddddd T : "<<T << "    type : "<<traj->type<<endl;
        return;
    }
    */
    if(traj->vmax < traj->vo + traj->ao*traj->ao/traj->j/2.0 && traj->vo >traj->vf && traj->ao >=0){
        traj->type = D2D;
        T = traj->min_time[D2D];
        return;
    }
    if(traj->vmax < traj->vf + traj->af*traj->af/traj->j/2.0 && traj->vo <traj->vf && traj->af <=0){
        traj->type = U2U;   
        T = traj->min_time[U2U];
        return;
    }

    if(traj->min_time_err[U2D] == 0 && traj->init_err[U2D]==0){
        if (traj->min_time[U2D] >= T){
            T = traj->min_time[U2D];
            traj->type = U2D;
            return;
        }
    }
    if(traj->min_time_err[U2D] == 0 && traj->max_time_err[U2D] == 0 && traj->init_err[U2D]==0 ){
        if (traj->min_time[U2D] <= T && T <= traj->max_time[U2D] ){
            traj->type = U2D;
            return;
        }
    }
    if(traj->max_time_err[U2D] == 0 && traj->min_time_err[U2U] == 0 && traj->init_err[U2D]==0 && traj->init_err[U2U]==0 ){
        if (traj->max_time[U2D] < T && T < traj->min_time[U2U] ){
            T = traj->min_time[U2U];
            traj->type = U2U;
            return;
        }
    }    
    if(traj->min_time_err[U2U] == 0 && traj->max_time_err[U2U] == 0 && traj->init_err[U2U]==0){
        if (traj->min_time[U2U] <= T && T <= traj->max_time[U2U] ){
            traj->type = U2U;
            return;
        }
    }
    if(traj->max_time_err[U2D] == 0 && traj->min_time_err[D2D] == 0 && traj->init_err[U2D]==0 && traj->init_err[D2D]==0 ){
        if (traj->max_time[U2D] < T && T < traj->min_time[D2D] ){
            T = traj->min_time[D2D];
            traj->type = D2D;
            return;
        }
    }      
    if(traj->min_time_err[D2D] == 0 && traj->max_time_err[D2D] == 0 && traj->init_err[U2U]==0 ){
        if (traj->min_time[D2D] <= T && T <= traj->max_time[D2D] ){
            traj->type = D2D;
            return;
        }
    }
    if(traj->max_time_err[U2U] == 0 && traj->min_time_err[D2U] == 0 && traj->init_err[U2U]==0 && traj->init_err[D2U]==0 ){
        if (traj->max_time[U2U] < T && T < traj->min_time[D2U] ){
            T = traj->min_time[D2U];
            traj->type = D2U;
            return;
        }
    }   
    if(traj->max_time_err[D2D] == 0 && traj->min_time_err[D2U] == 0 && traj->init_err[U2U]==0 && traj->init_err[D2U]==0 ){
        if (traj->max_time[D2D] < T && T < traj->min_time[D2U] ){
            T = traj->min_time[D2U];
            traj->type = D2U;
            return;
        }
    }   
     if(traj->min_time_err[D2U] == 0 && traj->max_time_err[D2U] == 0 && traj->init_err[D2U]==0 ){
        if (traj->min_time[D2U] <= T && T <= traj->max_time[D2U] ){
            traj->type = D2U;
            return;
        }
    }

    if(traj->min_time_err[U2D]==0 && traj->init_err[U2D]==0 ){
        T = traj->min_time[U2D];
        traj->type = U2D;
        return;
    }
    if(traj->max_time_err[U2D]==0 && traj->init_err[U2D]==0 ){
        T = traj->max_time[U2D];
        traj->type = U2D;
        return;
    }
    if(traj->min_time_err[U2U]==0 && traj->init_err[U2U]==0 ){
        T = traj->min_time[U2U];
        traj->type = U2U;
        return;
    }    
    if(traj->max_time_err[U2U]==0 && traj->init_err[U2U]==0 ){
        T = traj->max_time[U2U];
         traj->type = U2U;
        return;
    }
    if(traj->min_time_err[D2D]==0 && traj->init_err[D2D]==0 ){
        T = traj->min_time[D2D];
        traj->type = D2D;
        return;
    }        
    if(traj->max_time_err[D2D]==0 && traj->init_err[D2D]==0 ){
        T = traj->max_time[D2D];
        traj->type = D2D;
        return;
    }
    if(traj->min_time_err[D2U]==0 && traj->init_err[D2U]==0 ){
        T = traj->min_time[D2U];
        traj->type = D2U;
        return;
    }
    if(traj->max_time_err[D2U]==0 && traj->init_err[D2U]==0 ){
        T = traj->max_time[D2U];
        traj->type = D2U;
        return;
    }

    
}
double ScurveGenerator::getTime(Trajectory* traj, double s) {
    generate(traj);
    unsigned int type = traj->type;
    int k1 = traj->k[type][0];
    int k2 = traj->k[type][1];
    //PARAMETER SET
    double vo = traj->vo;
    double vf = traj->vf;
    double vp = traj->vp;
    double vmax = traj->vmax;
    double ao = traj->ao;
    double af = traj->af;
    double a1 = traj->a1;
    double a2 = traj->a2;
    double amax = traj->amax;
    double dmax = traj->dmax;
    double so = traj->so;
    double sf = traj->sf;
    double sof = sf - so;
    double j = traj->j;


    double t0 = 0;
    double t1 = traj->tj1;
    double t2 = traj->ta1 - traj->tj2;
    double t3 = traj->ta1;
    double t4 = traj->tt - traj->ta2;
    double t5 = traj->tt - traj->ta2 + traj->tj3;
    double t6 = traj->tt - traj->tj4;
    double t7 = traj->tt;


    double t = 0;
    double st[6];
    double dst[6];
    for (int i = 0; i < 6; i++) {
        st[i] = traj->st[i];
        dst[i] = traj->dst[i];
    }
    if (s >= so && s < st[0]) {
        double c1 = k1 * 1 / 6.0 * j;
        double c2 = 1 / 2.0 * ao;
        double c3 = vo;
        double c4 = so - s;
        vector<double> roots = roots3(c1, c2, c3, c4);
        double min_t = INF;
        for (int i = 0; i < roots.size(); i++) {
            if (roots.at(i) < 0) continue;
            else {
                if (min_t > roots.at(i)) {
                    min_t = roots.at(i);
                }
            }
        }
        t = min_t + t0;
    }
    if (s >= st[0] && s < st[1]) {
        double c1 = 1.0 / 2.0 * k1 * a1;
        double c2 = dst[0];
        double c3 = st[0] - s;
        vector<double> roots = roots2(c1, c2, c3);
        double min_t = INF;
        for (int i = 0; i < roots.size(); i++) {
            if (roots.at(i) < 0) continue;
            else {
                if (min_t > roots.at(i)) {
                    min_t = roots.at(i);
                }
            }
        }
        t = min_t + t1;
    }
    if (s >= st[1] && s < st[2]) {
        double c1 = -1.0 / 6.0 * k1 * j;
        double c2 = 1.0 / 2.0 * k1 * a1;
        double c3 = dst[1];
        double c4 = st[1] - s;
        vector<double> roots = roots3(c1, c2, c3, c4);
        double min_t = INF;

        for (int i = 0; i < roots.size(); i++) {
            if (roots.at(i) < 0) continue;
            else {
                if (min_t > roots.at(i)) {
                    min_t = roots.at(i);
                }
            }
        }
        t = min_t + t2;
    }
    if (s >= st[2] && s < st[3]) {
        t = -(st[2] - s) / vp + t3;
    }
    if (s >= st[3] && s < st[4]) {
        double c1 = 1.0 / 6.0 * k2 * j;
        double c2 = 0;
        double c3 = dst[3];
        double c4 = st[3] - s;


        vector<double> roots = roots3(c1, c2, c3, c4);
        double min_t = INF;

        for (int i = 0; i < roots.size(); i++) {
            if (roots.at(i) < 0) continue;
            else {
                if (min_t > roots.at(i)) {
                    min_t = roots.at(i);
                }
            }
        }
        t = min_t + t4;
    }
    if (s >= st[4] && s < st[5]) {
        double c1 = 1.0 / 2.0 * k2 * a2;
        double c2 = dst[4];
        double c3 = st[4] - s;
        vector<double> roots = roots2(c1, c2, c3);
        double min_t = INF;
        for (int i = 0; i < roots.size(); i++) {
            if (roots.at(i) < 0) continue;
            else {
                if (min_t > roots.at(i)) {
                    min_t = roots.at(i);
                }
            }
        }
        t = min_t + t5;
    }
    if (s >= st[5] && s < st[6]) {
        double c1 = -1.0 / 6.0 * k2 * j;
        double c2 = 1.0 / 2.0 * k2 * a2;
        double c3 = dst[6];
        double c4 = st[5] - s;
        vector<double> roots = roots3(c1, c2, c3, c4);

        double min_t = INF;

        for (int i = 0; i < roots.size(); i++) {
            //std:://cout <<"roots "<<i<<" : " << roots.at(i) << std::endl;
            if (roots.at(i) < 0) continue;
            else {
                if (min_t > roots.at(i)) {
                    min_t = roots.at(i);
                }
            }
        }
        t = min_t + t6;
    }
    if (s >= sof) {
        t = traj->tt;
    }
    return t;
}
void ScurveGenerator::updateTargetTime_(int num, int type, double T){
    //cout<<"UPDATE TARGET TIME"<<endl;
    this->traj_list.at(num).type = type;
    this->updateTargetTime(&this->traj_list.at(num),T);

}
void ScurveGenerator::updateTargetTime(Trajectory* traj, double T) {
    // jerk based 

    for(int jj = 0;jj<100;jj++){

         if(traj->type ==U2D){
            double jmax = traj->j;
            double k1 = 1.0;
            double k2 = -1.0;
            double vo = traj->vo;
            double vf = traj->vf;
            double vp = traj->vp;
            double vmax = traj->vmax;
            double ao = traj->ao;
            double af = traj->af;
            double a1 = traj->a1;
            double a2 = traj->a2;
            double amax = traj->amax;
            double dmax = traj->dmax;
            double so = traj->so;
            double sf = traj->sf;
            double sof = sf - so;
            double j = traj->j;        
            double c = -(af*af*af/3.0 - ao*ao*ao/3.0 + (a1*ao*ao)/4.0)/vp - (a2*af*af)/(4*vp) - af*af*af*af/(8*a2*vp) - ao*ao*ao*ao/(8*a1*vp);
            double b = ((a1*vo)/2.0 - (a1*vp)/2.0 + af*vf - af*vp - ao*vo + ao*vp)/vp + ((af*af*vf)/2.0 - (af*af*vp)/2.0)/(a2*vp) + (a2*(2*vf - 2*vp))/(4*vp) + (ao*ao*(2*vo - 2*vp))/(4*a1*vp);
            double a =   (so - sf + T*vp)/vp - (vf*vf/2.0 - vf*vp + vp*vp/2.0)/(a2*vp) - (2*vo - 2*vp)* (2*vo - 2*vp)/(8*a1*vp);
            vector<double> roots = roots2(a, b, c);
            if(roots.at(0)> 0)
                traj->j = roots.at(0);
            else
                traj->j = roots.at(1);
            int new_type =  findMinTrajType(traj);
            traj->type = new_type;
            this->generate(traj);   
            cout<<traj->tt<<endl;
            if(abs(traj->tt -T)<1e-15)break;
        }
        else if (traj->type == UU1){
            double jmax = traj->j;
            double k1 = 1.0;
            double vo = traj->vo;
            double vf = traj->vf;
            double vp = traj->vp;
            double vmax = traj->vmax;
            double ao = traj->ao;
            double af = traj->af;
            double a1 = traj->a1;
            double a2 = traj->a2;
            double amax = traj->amax;
            double dmax = traj->dmax;
            double so = traj->so;
            double sf = traj->sf;
            double sof = sf - so;
            double j = traj->j;              
            traj->j = (2*a1*a1 - 4*a1*a2 + 2*a1*af - 2*a1*ao + 2*a2*a2 - af*af + ao*ao)/(2*vo - 2*vf + 2*T*a1);
            int new_type =  findMinTrajType(traj);
            traj->type = new_type;            
            this->generate(traj);               
            if(abs(traj->tt -T)<1e-15)break;

        }
        else if(traj->type ==DD){
            double jmax = traj->j;
            double k1 = -1.0;
            double k2 = 1.0;    
            double vo = traj->vo;
            double vf = traj->vf;
            //cout<<"====================="<<endl;
            traj->vp = -traj->vmax;
            traj->a1 = sqrt(traj->ao * traj->ao + 2 * k1 * traj->j * (traj->vp - traj->vo)) / SQRT2;
            if( traj->a1 > traj->dmax) traj->a1 = traj->dmax;

            traj->a2 = sqrt(traj->af * traj->af - 2 * k2 * traj->j * (traj->vp - traj->vf)) / SQRT2;
            if( traj->a2 > traj->amax) traj->a1 = traj->amax;
            traj->type = DD;
            int new_type =  findMinTrajType(traj);
            traj->type = new_type;
            this->generate(traj);    
            double vp = traj->vp;
            double a1 = traj->a1;
            double a2 = traj->a2;          
            double dttjdj = (k2*(2*vf - 2*vp))/(2*a2) - (k1*(2*vo - 2*vp))/(2*a1);
            double j = traj->j;
            double new_j = j-(T*j-traj->tt*j)/dttjdj;
            //if (new_j > jmax || isnan(new_j))new_j = jmax;
            if (new_j < 0 )new_j = 1;

                    
            traj->j = new_j;
            j = new_j;
        
        }
    }

            

}
bool ScurveGenerator::checkAllEqual(const vector<double>& v) {
    bool ret = true;
    for (int idx = 1; idx < v.size(); idx++) {
        if (abs(v.at(0) - v.at(idx)) > v.at(0) * EPS) {
            ret = false;
        }
    }
    return ret;
}
void ScurveGenerator::syncTime() {
    //FIRST SYNCRONIZTION
    double targetTime = 0;
    for (int idx = 0; idx < this->traj_list.size(); idx++) {
        Trajectory traj = traj_list.at(idx);
        if (traj.min_time[U2D] > targetTime) {
            targetTime = traj.min_time[U2D];
        }
    }
    vector<double> tt_list;
    for (int idx = 0; idx < this->traj_list.size(); idx++) {
        updateTargetTime(&this->traj_list.at(idx), targetTime);
        tt_list.push_back(this->traj_list.at(idx).tt);
    }

    bool is_done = checkAllEqual(tt_list);
    if (!is_done) return;
    //SYNCRONIZTION LOOP
    while (!is_done) {
        vector<double> tt_list;
        targetTime = *(std::max)(tt_list.begin(), tt_list.end());
        for (int idx = 0; idx < this->traj_list.size(); idx++) {
            updateTargetTime(&this->traj_list.at(idx), targetTime);
            tt_list.push_back(this->traj_list.at(idx).tt);
        }
        is_done = checkAllEqual(tt_list);
    }
    this->sync_time = targetTime;
}
void ScurveGenerator::syncTargetTime(double targetTime) {
    if (targetTime < this->sync_time) {
        targetTime = this->sync_time;
    }
    bool is_done = 0;
    while (!is_done) {
        vector<double> tt_list;

        for (int idx = 0; idx < this->traj_list.size(); idx++) {
            updateTargetTime(&this->traj_list.at(idx), targetTime);
            tt_list.push_back(this->traj_list.at(idx).tt);
            ////cout<<tt_list.at(idx)<<"\t";
        }
        ////cout<<endl;
        is_done = checkAllEqual(tt_list);
        targetTime = *(std::max)(tt_list.begin(), tt_list.end());
        ////cout<<targetTime<<endl;
        ////cout<<is_done<<endl;

    }
}
void ScurveGenerator::printTrajectory(Trajectory* traj) {
    int idx = 0;
    vector<std::string> lines;
    lines.push_back("-------------------");
    lines.push_back("|       AXIS      |");
    lines.push_back("|-----------------|");
    lines.push_back("|       vp        |");
    lines.push_back("|       a1        |");
    lines.push_back("|       a2        |");
    lines.push_back("|       vmax      |");
    lines.push_back("|       amax      |");
    lines.push_back("|       dmax      |");
    lines.push_back("|       j         |");
    lines.push_back("|       so        |");
    lines.push_back("|       vo        |");
    lines.push_back("|       ao        |");
    lines.push_back("|       sf        |");
    lines.push_back("|       vf        |");
    lines.push_back("|       af        |");
    lines.push_back("|-----------------|");
    lines.push_back("|       tj1       |");
    lines.push_back("|       tca1      |");
    lines.push_back("|       tj2       |");
    lines.push_back("|       tc        |");
    lines.push_back("|       tj3       |");
    lines.push_back("|       tca2      |");
    lines.push_back("|       tj4       |");
    lines.push_back("|-----------------|");
    lines.push_back("|  init_err[U2D]  |");
    lines.push_back("|  init_err[U2U]  |");
    lines.push_back("|  init_err[D2D]  |");
    lines.push_back("|  init_err[D2U]  |");
    lines.push_back("|  init_err[UU1]  |");
    lines.push_back("|  init_err[UU2]  |");
    lines.push_back("|  init_err[DD ]  |");
    lines.push_back("|-----------------|");
    lines.push_back("|    err[U2D]     |");
    lines.push_back("|    err[U2U]     |");
    lines.push_back("|    err[D2D]     |");
    lines.push_back("|    err[D2U]     |");
    lines.push_back("|    err[UU1]     |");
    lines.push_back("|    err[UU2]     |");
    lines.push_back("|    err[DD ]     |");
    lines.push_back("|-----------------|");
    lines.push_back("|       TYPE      |");
    lines.push_back("|-----------------|");
    lines.push_back("|       tt        |");
    lines.push_back("|-----------------|");

    int dof = this->traj_list.size();
    for (int idx = 0; idx < dof; idx++) {
        
        std::stringstream stream;
        int j = 0;

        if (idx < dof - 1)stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(8) << setprecision(1) << idx + 1;
        stream << "       |";
        lines.at(j++).append(stream.str());


        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vp;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->a1;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->a2;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vmax;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->amax;
        stream << "|";
        lines.at(j++).append(stream.str());


        stream.str("");
        stream << setw(15) << setprecision(5) << traj->dmax;
        stream << "|";
        lines.at(j++).append(stream.str());


        stream.str("");
        stream << setw(15) << setprecision(5) << traj->j;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->so;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vo;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->ao;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->sf;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vf;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->af;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj1;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tca1;
        stream << "|";
        lines.at(j++).append(stream.str());


        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj2;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tc;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj3;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tca2;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj4;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[U2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[U2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[D2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[D2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[UU1];
        stream << "|";
        lines.at(j++).append(stream.str());        

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[UU2];
        stream << "|";
        lines.at(j++).append(stream.str());        

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[DD];
        stream << "|";
        lines.at(j++).append(stream.str());        

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[U2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[U2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[D2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[D2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[UU1];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[UU2];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[DD];
        stream << "|";
        lines.at(j++).append(stream.str());                
        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (traj->type == U2D)
            stream << setw(9) << setprecision(1) << "U2D";
        else if (traj->type == U2U)
            stream << setw(9) << setprecision(1) << "U2U";
        else if (traj->type == D2D)
            stream << setw(9) << setprecision(1) << "D2D";
        else if (traj->type == D2U)
            stream << setw(9) << setprecision(1) << "D2U";
        else if (traj->type == UU1)
            stream << setw(9) << setprecision(1) << "UU1";        
        else if (traj->type == UU2)
            stream << setw(9) << setprecision(1) << "UU2";                
        else if (traj->type == DD)
            stream << setw(9) << setprecision(1) << "DD ";                
        stream << "      |";
        lines.at(j++).append(stream.str());


        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());



        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tt;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());


    }
    for (int i = 0; i < lines.size(); i++)
        cout << lines.at(i) << endl;
}
vector<double> ScurveGenerator::getSlist(Trajectory traj,double time_step) {
    vector<double> s_list;
   
    for (int i = 0; i < int(traj.tt / time_step); i++) {
        generate(&traj,i*time_step);
        s_list.push_back(traj.s);
    }
 
    return s_list;

}
vector<double> ScurveGenerator::getdSlist(Trajectory traj,double time_step) {
    vector<double> ds_list;
 
    for (int i = 0; i < int(traj.tt / time_step); i++) {
        generate(&traj,i*time_step);
        ds_list.push_back(traj.ds);
    }
    
    return ds_list;

}
vector<double> ScurveGenerator::getddSlist(Trajectory traj,double time_step) {
    vector<double> dds_list;
   
    for (int i = 0; i < int(traj.tt / time_step); i++) {
        generate(&traj,i*time_step);
        dds_list.push_back(traj.dds);
    }    
    
    return dds_list;
}
vector<double> ScurveGenerator::getdddSlist(Trajectory traj,double time_step) {
    vector<double> ddds_list;

    for (int i = 0; i < int(traj.tt / time_step); i++) {
        generate(&traj,i*time_step);
        ddds_list.push_back(traj.ddds);
    }        
      
    return ddds_list;
}
void ScurveGenerator::printAllTrajectory() {
    vector<std::string> lines;
    lines.push_back("-------------------");
    lines.push_back("|       AXIS      |");
    lines.push_back("|-----------------|");
    lines.push_back("|       vp        |");
    lines.push_back("|       a1        |");
    lines.push_back("|       a2        |");
    lines.push_back("|       vmax      |");
    lines.push_back("|       amax      |");
    lines.push_back("|       dmax      |");
    lines.push_back("|       j         |");
    lines.push_back("|       so        |");
    lines.push_back("|       vo        |");
    lines.push_back("|       ao        |");
    lines.push_back("|       sf        |");
    lines.push_back("|       vf        |");
    lines.push_back("|       af        |");
    lines.push_back("|-----------------|");
    lines.push_back("|       tj1       |");
    lines.push_back("|       tca1      |");
    lines.push_back("|       tj2       |");
    lines.push_back("|       tc        |");
    lines.push_back("|       tj3       |");
    lines.push_back("|       tca2      |");
    lines.push_back("|       tj4       |");
    lines.push_back("|-----------------|");
    lines.push_back("|  init_err[U2D]  |");
    lines.push_back("|  init_err[U2U]  |");
    lines.push_back("|  init_err[D2D]  |");
    lines.push_back("|  init_err[D2U]  |");
    lines.push_back("|  init_err[UU1]  |");
    lines.push_back("|  init_err[UU2]  |");
    lines.push_back("|  init_err[DD ]  |");
    lines.push_back("|-----------------|");
    lines.push_back("|    err[U2D]     |");
    lines.push_back("|    err[U2U]     |");
    lines.push_back("|    err[D2D]     |");
    lines.push_back("|    err[D2U]     |");
    lines.push_back("|    err[UU1]     |");
    lines.push_back("|    err[UU2]     |");
    lines.push_back("|    err[DD ]     |");
    lines.push_back("|-----------------|");
    lines.push_back("|       TYPE      |");
    lines.push_back("|-----------------|");
    lines.push_back("|       tt        |");
    lines.push_back("|-----------------|");

    int dof = this->traj_list.size();
    for (int idx = 0; idx < dof; idx++) {
        Trajectory* traj = &this->traj_list.at(idx);
        std::stringstream stream;
        int j = 0;

        if (idx < dof - 1)stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(8) << setprecision(1) << idx + 1;
        stream << "       |";
        lines.at(j++).append(stream.str());


        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vp;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->a1;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->a2;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vmax;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->amax;
        stream << "|";
        lines.at(j++).append(stream.str());


        stream.str("");
        stream << setw(15) << setprecision(5) << traj->dmax;
        stream << "|";
        lines.at(j++).append(stream.str());


        stream.str("");
        stream << setw(15) << setprecision(5) << traj->j;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->so;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vo;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->ao;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->sf;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->vf;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->af;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj1;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tca1;
        stream << "|";
        lines.at(j++).append(stream.str());


        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj2;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tc;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj3;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tca2;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tj4;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[U2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[U2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[D2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[D2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[UU1];
        stream << "|";
        lines.at(j++).append(stream.str());        

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[UU2];
        stream << "|";
        lines.at(j++).append(stream.str());        

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->init_err[DD];
        stream << "|";
        lines.at(j++).append(stream.str());        

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[U2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[U2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[D2D];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[D2U];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[UU1];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[UU2];
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        stream << setw(15) << setprecision(5) << traj->err[DD];
        stream << "|";
        lines.at(j++).append(stream.str());                
        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (traj->type == U2D)
            stream << setw(9) << setprecision(1) << "U2D";
        else if (traj->type == U2U)
            stream << setw(9) << setprecision(1) << "U2U";
        else if (traj->type == D2D)
            stream << setw(9) << setprecision(1) << "D2D";
        else if (traj->type == D2U)
            stream << setw(9) << setprecision(1) << "D2U";
        else if (traj->type == UU1)
            stream << setw(9) << setprecision(1) << "UU1";        
        else if (traj->type == UU2)
            stream << setw(9) << setprecision(1) << "UU2";                
        else if (traj->type == DD)
            stream << setw(9) << setprecision(1) << "DD ";                
        stream << "      |";
        lines.at(j++).append(stream.str());


        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());



        stream.str("");
        stream << setw(15) << setprecision(5) << traj->tt;
        stream << "|";
        lines.at(j++).append(stream.str());

        stream.str("");
        if (idx < dof - 1) stream << "----------------";
        else stream << "----------------";
        lines.at(j++).append(stream.str());


    }
    for (int i = 0; i < lines.size(); i++)
        cout << lines.at(i) << endl;
}


void ScurveGenerator::saveAllTrajectory(double time_step) {
    FILE* fp=fopen("data.csv", "w");
    char buf[200];
    fputs("time , s , ds , dds , ddds , state , vmax , amax , dmax, j , type , idx \n", fp);
    for (int idx = 0; idx < this->traj_list.size(); idx++) {
        Trajectory* traj;
        traj = &this->traj_list.at(idx);

        unsigned int type = traj->type;
        generate(traj, 0);
        double tt = traj->tt;
        vector<double> t_list;
        vector<double> s_list;
        vector<double> ds_list;
        vector<double> dds_list;
        vector<double> ddds_list;
        vector<int> state_list;

        for (int i = 0; i < int(tt / time_step) + 2; i++) {
            t_list.push_back(i * time_step);
            s_list.push_back(0);
            ds_list.push_back(0);
            dds_list.push_back(0);
            ddds_list.push_back(0);
            state_list.push_back(0);
        }

        for (int i = 0; i < int(tt / time_step) + 2; i++) {
            generate(traj, t_list.at(i));

            s_list.at(i) = (traj->s);
            ds_list.at(i) = (traj->ds);
            dds_list.at(i) = (traj->dds);
            ddds_list.at(i) = (traj->ddds);
            state_list.at(i) = (traj->state);
        }


        for (int i = 0; i < t_list.size(); i++) {
            sprintf(buf, "%f , %f , %f , %f , %f , %d, %f, %f, %f, %f, %d , %d \n ", t_list.at(i), s_list.at(i), ds_list.at(i), dds_list.at(i), ddds_list.at(i), state_list.at(i), traj->vmax, traj->amax, traj->dmax, traj->j, traj->type, idx);
            fputs(buf, fp);   // 파일에 문자열 저장
        }
    }
}