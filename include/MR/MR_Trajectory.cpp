#include "MR_Trajectory.h"
MR_Trajectory::MR_Trajectory(){
    cout<<"START MR_Trajectory"<<endl;
    this->Tf = 0;
    this->N = 0;
}
Vector6d flip(Vector6d V){
	Vector6d V_flip;
	V_flip<<V(3),V(4),V(5),V(0),V(1),V(2);
	return V_flip;
}
// void MR_Trajectory::addLieScrewTrajectory(mr::SE3 X0,mr::SE3 X1,mr::Vector6d V0,mr::Vector6d V1,mr::Vector6d Vd0,mr::Vector6d Vd1,double t0,double t1,double dt,int trajType){
// 	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
// 	lambda_0 = Vector6d::Zero();
// 	lambda_T = flip(se3ToVec(MatrixLog6(TransInv(X0)*X1)));
// 	dlambda_0 = V0;
// 	dlambda_T = dlog6(-lambda_T)*V1;
// 	ddlambda_0 = Vd0;
// 	ddlambda_T = dlog6(-lambda_T)*Vd1 +ddlog6(-lambda_T,-dlambda_T)*V1;
    
//     this->Tf = (t1-t0);
//     this->N = floor(this->Tf)/dt;
//     this->Xd_list.resize(N);
//     this->Vd_list.resize(N);
//     this->dVd_list.resize(N);
//     this->t_list.resize(N);
// 	double timegap = Tf /(N/1.0 - 1.0);
// 	for (int i = 0;i<N;i++){
// 		lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();
// 		double t= timegap*(i-1);
// 		for(int j = 0;j<6;j++){
// 			Vector3d ret = QuinticTimeScalingKinematics(lambda_0(j),lambda_T(j),dlambda_0(j),dlambda_T(j),ddlambda_0(j),ddlambda_T(j),Tf,t) ;
// 			lambda_t(j) = ret(0);
// 			dlambda_t(j) = ret(1);
// 			ddlambda_t(j) = ret(2);
// 		}
// 		Vector6d V = dexp6(-lambda_t)*dlambda_t;
// 		Vector6d dV = dexp6(-lambda_t)*ddlambda_t+ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
// 		SE3 T = X0*MatrixExp6(VecTose3(flip(lambda_t)));
// 		Xd_list.at(i) = T;
// 		Vd_list.at(i) = flip(V);
// 		dVd_list.at(i) = flip(dV);
//         t_list.at(i) = t;
//     }    

// }

void MR_Trajectory::addLieScrewTrajectory(mr::SE3 X0,mr::SE3 X1,mr::Vector6d V0,mr::Vector6d V1,mr::Vector6d Vd0,mr::Vector6d Vd1,double t0,double t1,double dt,int trajType){
	Vector6d lambda_0,lambda_1,dlambda_0,dlambda_1,ddlambda_0,ddlambda_1,lambda_t,dlambda_t,ddlambda_t;
    Vector6d vec_eps;
    vec_eps<<dt/100.0,dt/100.0,dt/100.0,dt/100.0,dt/100.0,dt/100.0;
	lambda_0 = Vector6d::Zero();
	lambda_1 = flip(se3ToVec(MatrixLog6(TransInv(X0)*X1)))+vec_eps;
	dlambda_0 = V0;
	dlambda_1 = dlog6(-lambda_1)*V1;
	ddlambda_0 = Vd0;
	ddlambda_1 = dlog6(-lambda_1)*Vd1 +ddlog6(-lambda_1,-dlambda_1)*V1;

    Vector6d s0,s1,sdot0,sdot1,sddot0,sddot1,s_t,sdot_t,sddot_t;
    for(int j = 0;j<6;j++){
        s0(j) = lambda_0(j)/(lambda_1(j)-lambda_0(j));
        s1(j) = lambda_1(j)/(lambda_1(j)-lambda_0(j));
        sdot0(j) = dlambda_0(j)/(lambda_1(j)-lambda_0(j));
        sdot1(j) = dlambda_1(j)/(lambda_1(j)-lambda_0(j));
        sddot0(j) = ddlambda_0(j)/(lambda_1(j)-lambda_0(j));
        sddot1(j) = ddlambda_1(j)/(lambda_1(j)-lambda_0(j));
    }


    this->Tf = (t1-t0);
    this->N = floor(this->Tf)/dt;
    this->Xd_list.resize(N);
    this->Vd_list.resize(N);
    this->dVd_list.resize(N);
    this->t_list.resize(N);
	double timegap = Tf /(N/1.0 - 1.0);
	for (int i = 0;i<N;i++){
		lambda_t=dlambda_t=ddlambda_t=s_t=sdot_t=sddot_t= Vector6d::Zero();
		double t= timegap*(i-1);
		for(int j = 0;j<6;j++){
			Vector3d ret = QuinticTimeScalingKinematics(s0(j),s1(j),sdot0(j),sdot1(j),sddot0(j),sddot1(j),Tf,t) ;
			s_t(j) = ret(0);
			sdot_t(j) = ret(1);
			sddot_t(j) = ret(2);
            lambda_t(j) = lambda_0(j) + s_t(j) * (lambda_1(j)-lambda_0(j));
            dlambda_t(j) = sdot_t(j) * (lambda_1(j)-lambda_0(j));
            ddlambda_t(j) = sddot_t(j) * (lambda_1(j)-lambda_0(j));            
		}
		Vector6d V = dexp6(-lambda_t)*dlambda_t;
		Vector6d dV = dexp6(-lambda_t)*ddlambda_t+ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
		SE3 T = X0*MatrixExp6(VecTose3(flip(lambda_t)));
		Xd_list.at(i) = T;
		Vd_list.at(i) = flip(V);
		dVd_list.at(i) = flip(dV);
        t_list.at(i) = t;
    }    

}
void MR_Trajectory::setJointTrajectory(const mr::JVec& q0,const mr::JVec& qT,const mr::JVec& qdot0,const mr::JVec& qdotT,const mr::JVec& qddot0,const mr::JVec& qddotT,double t0,double Tf,int N){
    double timegap = Tf / (N - 1);
    double st;
    this->q_list.resize(N);
    this->t_list.resize(N);
    for (int i = 0; i < N; ++i) {
        st = QuinticTimeScaling(Tf, timegap*i);
        this->q_list.at(i) = st * qT + (1 - st)*q0;
        this->t_list.at(i) = timegap*i+t0;
    }    
}

void MR_Trajectory::setRelativeJointTrajectory(const relmr::JVec& q0,const relmr::JVec& q1,const relmr::JVec& qdot0,const relmr::JVec& qdot1,const relmr::JVec& qddot0,const relmr::JVec& qddot1,double Tf,double dt,int trajType){
    relmr::JVec s0,s1,sdot0,sdot1,sddot0,sddot1;
    for(int j = 0;j<q0.size();j++){
        s0(j) = 0;
        s1(j) = 1;
        sdot0(j) = qdot0(j)/(q1(j)-q0(j));
        sdot1(j) = qdot1(j)/(q1(j)-q0(j));
        sddot0(j) = qddot0(j)/(q1(j)-q0(j));
        sddot1(j) = qddot1(j)/(q1(j)-q0(j));
    }
        
    int N = floor(Tf/dt);
    double timegap = Tf / (N - 1);

    this->Tf = Tf;
    this->N = N;
    this->q_rel_list.resize( this->N );
    this->qdot_rel_list.resize( this->N );
    this->qddot_rel_list.resize( this->N );
    this->t_list.resize( this->N );    
    this->timegap = timegap;
    for (int i = 0; i < N; ++i) {
        relmr::JVec s_t =relmr::JVec::Zero();
        relmr::JVec sdot_t =relmr::JVec::Zero();
        relmr::JVec  sddot_t =relmr::JVec::Zero();
        relmr::JVec q_t =relmr::JVec::Zero(); 
        relmr::JVec qdot_t =relmr::JVec::Zero(); 
        relmr::JVec qddot_t =relmr::JVec::Zero(); 
        for(int j = 0;j<q0.size();j++){
            Vector3d ret = QuinticTimeScalingKinematics(s0(j),s1(j),sdot0(j),sdot1(j),sddot0(j),sddot1(j),Tf,timegap*i) ;
            s_t(j) = ret(0);
            sdot_t(j) = ret(1);
            sddot_t(j) = ret(2);
            q_t(j) = q0(j) + s_t(j) * (q1(j)-q0(j));
            qdot_t(j) = sdot_t(j) * (q1(j)-q0(j));
            qddot_t(j) = sddot_t(j) * (q1(j)-q0(j));
        }
     
        this->q_rel_list.at(i) = q_t;
        this->qdot_rel_list.at(i) = qdot_t;
        this->qddot_rel_list.at(i) = qddot_t;
        this->t_list.at(i) = timegap*i;
    } 
}
void MR_Trajectory::addRelativeJointTrajectory(const relmr::JVec& q0,const relmr::JVec& q1,const relmr::JVec& qdot0,const relmr::JVec& qdot1,const relmr::JVec& qddot0,const relmr::JVec& qddot1,double t0,double t1,double dt,int trajType){
    if(this->N==0) {
        this->setRelativeJointTrajectory(q0,q1,qdot0,qdot1,qddot0,qddot1,t1,dt,trajType);
        return;
    }
    relmr::JVec s0,s1,sdot0,sdot1,sddot0,sddot1;
    for(int j = 0;j<q0.size();j++){
        s0(j) = 0;
        s1(j) = 1;
        sdot0(j) = qdot0(j)/(q1(j)-q0(j));
        sdot1(j) = qdot1(j)/(q1(j)-q0(j));
        sddot0(j) = qddot0(j)/(q1(j)-q0(j));
        sddot1(j) = qddot1(j)/(q1(j)-q0(j));
    }
    
    int N = floor((t1-t0)/dt);
    double timegap = (t1-t0) / (N - 1);
    int prevN =this->N; 
    this->N = this->N+N;
    this->q_rel_list.resize(this->N);
    this->qdot_rel_list.resize(this->N);
    this->qddot_rel_list.resize(this->N);
    this->t_list.resize(this->N);
    this->Tf = this->Tf+(t1-t0);
    for (int i = 0; i < N; ++i) {
        relmr::JVec s_t =relmr::JVec::Zero();
        relmr::JVec sdot_t =relmr::JVec::Zero();
        relmr::JVec  sddot_t =relmr::JVec::Zero();
        relmr::JVec q_t =relmr::JVec::Zero(); 
        relmr::JVec qdot_t =relmr::JVec::Zero(); 
        relmr::JVec qddot_t =relmr::JVec::Zero(); 
        for(int j = 0;j<q0.size();j++){
            Vector3d ret = QuinticTimeScalingKinematics(s0(j),s1(j),sdot0(j),sdot1(j),sddot0(j),sddot1(j),(t1-t0),timegap*i) ;
            s_t(j) = ret(0);
            sdot_t(j) = ret(1);
            sddot_t(j) = ret(2);
            q_t(j) = q0(j) + s_t(j) * (q1(j)-q0(j));
            qdot_t(j) = sdot_t(j) * (q1(j)-q0(j));
            qddot_t(j) = sddot_t(j) * (q1(j)-q0(j));
        }
     
        this->q_rel_list.at(prevN+i) = q_t;
        this->qdot_rel_list.at(prevN+i) = qdot_t;
        this->qddot_rel_list.at(prevN+i) = qddot_t;
        this->t_list.at(prevN+i) = timegap*i+t0;
    }   
}

void MR_Trajectory::getJointTrajectory(double now_t ,mr::JVec& q_des,mr::JVec& qdot_des,mr::JVec& qddot_des ){

}
void MR_Trajectory::getRelativeJointTrajectory(double now_t ,relmr::JVec& q_des,relmr::JVec& qdot_des,relmr::JVec& qddot_des ){
    if (now_t >this->Tf){
        q_des = this->q_rel_list.at(this->N-1);
        qdot_des= this->qdot_rel_list.at(this->N-1);
        qddot_des= this->qddot_rel_list.at(this->N-1);
    }
    else{
        q_des=this->q_rel_list.at(floor((now_t)/timegap ));
        qdot_des=this->qdot_rel_list.at(floor((now_t)/timegap ));
        qddot_des=this->qddot_rel_list.at(floor((now_t)/timegap ));
    }
}
Json::Value toJson(const vector<relmr::JVec>& q_rel_list) {
    Json::Value root(Json::arrayValue);

   for (int i =0;i<q_rel_list.size();i++){
        Json::Value row(Json::arrayValue);
        relmr::JVec q_rel = q_rel_list.at(i);
        for(int j = 0;j<q_rel_list.at(0).size() ; j++){
             row.append(q_rel(j));
        }
        root.append(row);
   }
    return root;
}
Json::Value toJson(const vector<double>& t_list) {
    Json::Value root(Json::arrayValue);
   for (int i =0;i<t_list.size();i++){
        Json::Value row(Json::arrayValue);
        row.append(t_list.at(i));
        root.append(row);
   }
    return root;
}

void MR_Trajectory::saveJointTrajectory(const std::string& filename) {
   Json::Value json_q_rel_list = toJson(q_rel_list);
   Json::Value json_qdot_rel_list = toJson(qdot_rel_list);
   Json::Value json_qddot_rel_list = toJson(qddot_rel_list);
   Json::Value json_t_list = toJson(t_list);

    Json::StreamWriterBuilder writerBuilder;
    std::string output_q = Json::writeString(writerBuilder, json_q_rel_list);
    std::string output_qdot = Json::writeString(writerBuilder, json_qdot_rel_list);
    std::string output_qddot = Json::writeString(writerBuilder, json_qddot_rel_list);
    std::string output_t = Json::writeString(writerBuilder, json_t_list);
    std::ofstream outputFileStream_q("q_rel_list.json", std::ofstream::binary);
    std::ofstream outputFileStream_qdot("qdot_rel_list.json", std::ofstream::binary);
    std::ofstream outputFileStream_qddot("qddot_rel_list.json", std::ofstream::binary);
    std::ofstream outputFileStream_t("t_list.json", std::ofstream::binary);
    outputFileStream_q << output_q;
    outputFileStream_qdot << output_qdot;
    outputFileStream_qddot << output_qddot;
    outputFileStream_t << output_t;

}
 void MR_Trajectory::addScurveRelativeJointTrajectory(const relmr::JVec& q0,const relmr::JVec& q1,const relmr::JVec& qdot0,const relmr::JVec& qdot1,const relmr::JVec& qddot0,const relmr::JVec& qddot1,const relmr::JVec& qdot_max,const relmr::JVec& qddot_max,double t0,double t1,double dt,int trajType){
        relmr::JVec s0,s1,sdot0,sdot1,sddot0,sddot1;
        vector<Trajectory> traj_list;
        for(int j = 0;j<q0.size();j++){
            s0(j) = 0;
            s1(j) = 1;
            sdot0(j) = qdot0(j)/(q1(j)-q0(j));
            sdot1(j) = qdot1(j)/(q1(j)-q0(j));
            sddot0(j) = qddot0(j)/(q1(j)-q0(j));
            sddot1(j) = qddot1(j)/(q1(j)-q0(j));
            Trajectory traj; 
            traj.j = 2000/abs(q1(j)-q0(j));
            traj.amax =qddot_max(j)/abs(q1(j)-q0(j));
            traj.dmax =qddot_max(j)/abs(q1(j)-q0(j));
            traj.vmax = qdot_max(j)/abs(q1(j)-q0(j));
            traj.vmin =-qdot_max(j)/abs(q1(j)-q0(j));
            cout<<"traj.j : "<<traj.j<<endl;
            cout<<"traj.amax : "<<traj.amax <<endl;
            traj.ao = sddot0(j);
            traj.vo = sdot0(j);
            traj.af = sddot1(j);
            traj.vf = sdot1(j);
            traj.so = s0(j);
            traj.sf = s1(j);
            traj_list.push_back(traj);
        }
        cout<<sdot0.transpose()<<endl;
        ScurveGenerator sg = ScurveGenerator(traj_list);
        for(int j =0;j<q0.size();j++){
            sg.update_(j,U2D);
            sg.updateTargetTime_(j,U2D,t1-t0);

        }
        // Trajectory traj = sg.traj_list.at(0);
        // sg.generate(&traj,0.1);
        //     cout<<traj.s<<endl;
        // // sg.syncTime();
             sg.printAllTrajectory();

            int N = floor((t1-t0)/dt);
            double timegap = (t1-t0) / (N - 1);
            int prevN =this->N; 
            this->N = this->N+N;
            this->q_rel_list.resize(this->N);
            this->qdot_rel_list.resize(this->N);
            this->qddot_rel_list.resize(this->N);
            this->t_list.resize(this->N);
            this->Tf = this->Tf+(t1-t0);
            for (int i = 0; i < N; ++i) {
                relmr::JVec s_t =relmr::JVec::Zero();
                relmr::JVec sdot_t =relmr::JVec::Zero();
                relmr::JVec  sddot_t =relmr::JVec::Zero();
                relmr::JVec q_t =relmr::JVec::Zero(); 
                relmr::JVec qdot_t =relmr::JVec::Zero(); 
                relmr::JVec qddot_t =relmr::JVec::Zero(); 
                for(int j = 0;j<q0.size();j++){
                    Trajectory traj = sg.traj_list.at(j);
                    sg.generate(&traj,timegap*i);
                    s_t(j) = traj.s;
                    sdot_t(j) = traj.ds;
                    sddot_t(j) = traj.dds;
                    q_t(j) = q0(j) + s_t(j) * (q1(j)-q0(j));
                    qdot_t(j) = sdot_t(j) * (q1(j)-q0(j));
                    qddot_t(j) = sddot_t(j) * (q1(j)-q0(j));
                }
                this->q_rel_list.at(prevN+i) = q_t;
                this->qdot_rel_list.at(prevN+i) = qdot_t;
                this->qddot_rel_list.at(prevN+i) = qddot_t;
                this->t_list.at(prevN+i) = timegap*i+t0;
            }          
 }
// Vector3d ScurveTimeScalingKinematics(double s0, double s1 ,double sdot0,double sdot1,double sddot0,double sddot1,double dt,double Tf,double t){
//     Vector3d ret_sdsdds;
// 	Trajectory traj;
// 	traj.j = 10;
// 	traj.a = 1;
// 	traj.d = 1;
// 	traj.vmax = 10;
// 	traj.ao = sddot0;
// 	traj.vo = sdot0;
// 	traj.af = sddot1;
// 	traj.vf = sdot1;
// 	traj.so = s0;
// 	traj.sf = s1;
// 	int sampleFreq = floor(1/dt);
// 	Scurve s= Scurve(traj,sampleFreq);
// 	s.generate(0);
// 	double eps = 1e-20;
//    	double result;
// 	int ret=0;
// 	double minimumTime =s.update(ret,eps);
// 	int ret2=0;		    
//     double retTime = s.updateTargetTime(ret2, Tf);
//     s.generate(t);
//     traj = s.getTrajectory();

//     ret_sdsdds(0) = traj.s;
//     ret_sdsdds(1) = traj.ds;
//     ret_sdsdds(2) = traj.dds;
    
//     return ret_sdsdds;
// }
// void MR_Trajectory::addScurveRelativeJointTrajectory(const relmr::JVec& q0,const relmr::JVec& q1,const relmr::JVec& qdot0,const relmr::JVec& qdot1,const relmr::JVec& qddot0,const relmr::JVec& qddot1,double t0,double t1,double dt,int trajType){
//     relmr::JVec s0,s1,sdot0,sdot1,sddot0,sddot1;
//     for(int j = 0;j<q0.size();j++){
//         s0(j) = 0;
//         s1(j) = 1;
//         sdot0(j) = qdot0(j)/(q1(j)-q0(j));
//         sdot1(j) = qdot1(j)/(q1(j)-q0(j));
//         sddot0(j) = qddot0(j)/(q1(j)-q0(j));
//         sddot1(j) = qddot1(j)/(q1(j)-q0(j));
//     }
//     int N = floor((t1-t0)/dt);
//     double timegap = (t1-t0) / (N - 1);
//     int prevN =this->N; 
//     this->N = this->N+N;
//     this->q_rel_list.resize(this->N);
//     this->qdot_rel_list.resize(this->N);
//     this->qddot_rel_list.resize(this->N);
//     this->t_list.resize(this->N);
//     this->Tf = this->Tf+(t1-t0);
//     for (int i = 0; i < N; ++i) {
//         relmr::JVec s_t =relmr::JVec::Zero();
//         relmr::JVec sdot_t =relmr::JVec::Zero();
//         relmr::JVec  sddot_t =relmr::JVec::Zero();
//         relmr::JVec q_t =relmr::JVec::Zero(); 
//         relmr::JVec qdot_t =relmr::JVec::Zero(); 
//         relmr::JVec qddot_t =relmr::JVec::Zero(); 
//         for(int j = 0;j<q0.size();j++){
//             Vector3d ret = ScurveTimeScalingKinematics(s0(j), s1(j) ,sdot0(j),sdot1(j),sddot0(j),sddot1(j),dt,t1-t0,timegap*i);
//             s_t(j) = ret(0);
//             sdot_t(j) = ret(1);
//             sddot_t(j) = ret(2);
//             q_t(j) = q0(j) + s_t(j) * (q1(j)-q0(j));
//             qdot_t(j) = sdot_t(j) * (q1(j)-q0(j));
//             qddot_t(j) = sddot_t(j) * (q1(j)-q0(j));
//         }
     
//         this->q_rel_list.at(prevN+i) = q_t;
//         this->qdot_rel_list.at(prevN+i) = qdot_t;
//         this->qddot_rel_list.at(prevN+i) = qddot_t;
//         this->t_list.at(prevN+i) = timegap*i+t0;
//     }       
// }

// void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N,vector<SE3>&Xd_list,vector<Vector6d>&Vd_list,vector<Vector6d>&dVd_list){
// 	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
// 	lambda_0 = Vector6d::Zero();
// 	lambda_T = flip(se3ToVec(MatrixLog6(TransInv(X0)*XT)));
// 	dlambda_0 = V0;
// 	dlambda_T = dlog6(-lambda_T)*VT;
// 	ddlambda_0 = dV0;
// 	ddlambda_T = dlog6(-lambda_T)*dVT +ddlog6(-lambda_T,-dlambda_T)*VT;
// 	double timegap = Tf /(N/1.0 - 1.0);
// 	for (int i = 0;i<N;i++){
// 		lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();
// 		double t= timegap*(i-1);
// 		for(int j = 0;j<6;j++){
// 			Vector3d ret = QuinticTimeScalingKinematics(lambda_0(j),lambda_T(j),dlambda_0(j),dlambda_T(j),ddlambda_0(j),ddlambda_T(j),Tf,t) ;
// 			lambda_t(j) = ret(0);
// 			dlambda_t(j) = ret(1);
// 			ddlambda_t(j) = ret(2);
// 		}
// 		Vector6d V = dexp6(-lambda_t)*dlambda_t;
// 		Vector6d dV = dexp6(-lambda_t)*ddlambda_t+ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
// 		SE3 T = X0*MatrixExp6(VecTose3(flip(lambda_t)));
// 		Xd_list.at(i) = T;
// 		Vd_list.at(i) = flip(V);
// 		dVd_list.at(i) = flip(dV);
// 	}
	
	
// }