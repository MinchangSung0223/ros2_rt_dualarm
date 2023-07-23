#include "MR_Indy7.h"
#include "MR_Indy7_DualArm.h"
#include <chrono>
#include <iostream>

bool ReadFromFile_(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData_(const char* filename,Json::Value &rootr){
	cout<<"START ReadMRData"<<endl;
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile_(filename, readBuffer, BufferLength)) {
		std::cout<<"Failed"<<std::endl;
		return -1;
	}
	std::string config_doc = readBuffer;

	Json::Reader reader;
	bool parsingSuccessful = reader.parse(config_doc,rootr);
	if ( !parsingSuccessful ) { 
		std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
		return -1;
		
	}
    cout<<"END ReadMRData"<<endl;

    return 1;
}
// q = -q_r_flip, q_l
relmr::JVec MR_Indy7_DualArm::get_q_rel(relmr::JVec q){
    relmr::JVec q_rel = relmr::JVec::Zero();
    q_rel(0) = -q(11);
    q_rel(1) = -q(10);
    q_rel(2) = -q(9);
    q_rel(3) = -q(8);
    q_rel(4) = -q(7);
    q_rel(5) = -q(6);
    q_rel(6)  = q(0);
    q_rel(7)  = q(1);
    q_rel(8)  = q(2);
    q_rel(9)  = q(3);
    q_rel(10) = q(4);
    q_rel(11) = q(5);
    return q_rel;
}
// q = -q_r_flip, q_l
relmr::JVec MR_Indy7_DualArm::get_qdot_rel(relmr::JVec q){
    relmr::JVec q_rel = relmr::JVec::Zero();
    q_rel(0) = -q(11);
    q_rel(1) = -q(10);
    q_rel(2) = -q(9);
    q_rel(3) = -q(8);
    q_rel(4) = -q(7);
    q_rel(5) = -q(6);
    q_rel(6)  = q(0);
    q_rel(7)  = q(1);
    q_rel(8)  = q(2);
    q_rel(9)  = q(3);
    q_rel(10) = q(4);
    q_rel(11) = q(5);
    return q_rel;
}
 MR_Indy7_DualArm::MR_Indy7_DualArm(){
    this->jointnum = 12;
    this->L = new MR_Indy7();
    this->R = new MR_Indy7();
    this->L->MRSetup();
    this->R->MRSetup();
    this->L->q = JVec::Zero();
    this->R->q = JVec::Zero();
    this->q = relJVec::Zero();
    this->L->g << 0 ,8.487,-4.9;
    this->R->g << 0 ,8.487,-4.9;
    this->HinfSim_Kp = relmr::Matrixnxn::Identity();
    this->HinfSim_Kv = relmr::Matrixnxn::Identity();
    this->HinfSim_Ki = relmr::Matrixnxn::Identity();
    this->HinfSim_K_gamma = relmr::Matrixnxn::Identity();

    for (int i=0; i<12; ++i)
    {
        switch(i)
        {

        case 0:
            HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 50+1.0/800.0 ;
            break;

        case 1:
            HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 30+1.0/600.0 ;

            break;

        case 2:
            HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 30.0+1.0/500.0 ;

            break;

        case 3:
            HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 3.0+1.0/500.0 ;

            break;

        case 4:
			HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 3.0+1.0/500.0 ;

            break;

        case 5:
            HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 0.1+1.0/600.0 ;
            break;
		case 6:
			HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 50+1.0/800.0 ;
            break;	
		case 7:
			HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 30+1.0/600.0 ;

            break;					
		case 8:
			HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 30.0+1.0/500.0 ;

            break;	
		case 9:
			HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 3.0+1.0/500.0 ;

            break;					
		case 10:
			HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 3.0+1.0/500.0 ;

            break;	
		case 11:
			HinfSim_Kp(i,i) = 100.0;
            HinfSim_Kv(i,i) = 20.0;
            HinfSim_K_gamma(i,i) = 0.1+1.0/600.0 ;
            break;					
        }
    }	
 };
relmr::JVec MR_Indy7_DualArm::ForwardDynamics(const relmr::JVec q,const relmr::JVec qdot,const relmr::JVec tau,const Vector6d Ftip_r,const Vector6d Ftip_l){
	relmr::JVec qddot;
	mr::JVec q_l,q_r,qdot_l,qdot_r,qddot_l,qddot_r,tau_r,tau_l;
	q_r = q.segment<6>(0);
	q_l = q.segment<6>(6);	
	tau_r = tau.segment<6>(0);
	tau_l = tau.segment<6>(6);	
	qdot_r = qdot.segment<6>(0);
	qdot_l = qdot.segment<6>(6);	
	qddot_r = this->R->ForwardDynamics(q_r,qdot_r,tau_r,Ftip_r);
	qddot_l = this->L->ForwardDynamics(q_l,qdot_l,tau_l,Ftip_l);
	qddot.segment<6>(0) = qddot_r;
	qddot.segment<6>(6) = qddot_l;
	return qddot;
 }
relmr::MassMat MR_Indy7_DualArm::MassMatrix(const relmr::JVec q){
	relmr::MassMat M=relmr::MassMat::Zero();
	mr::JVec q_l,q_r;
	q_r = q.segment<6>(0);
	q_l = q.segment<6>(6);	
	M.topLeftCorner<JOINTNUM, JOINTNUM>() = this->R->MassMatrix(q_r);
	M.bottomRightCorner<JOINTNUM, JOINTNUM>() = this->L->MassMatrix(q_l);
	return M;
 }
 relmr::JVec MR_Indy7_DualArm::HinfControlSim(const relmr::JVec q,const relmr::JVec qdot,const relmr::JVec q_des,const relmr::JVec qdot_des,const relmr::JVec qddot_des,relmr::JVec& eint){
    relmr::JVec e = q_des-q;
    relmr::JVec edot = qdot_des-qdot;	
	relmr::MassMat Mmat = this->MassMatrix(q);
	relmr::JVec C = this->VelQuadraticForces(q,qdot);
	relmr::JVec G = this->GravityForces(q);
    relmr::JVec qddot_ref = qddot_des+HinfSim_Kv*edot+HinfSim_Kp*e;
	relmr::JVec qdot_ref = qdot_des+HinfSim_Kv*e+HinfSim_Kp*eint;
	relmr::JVec Cref = this->VelQuadraticForces(q,qdot_ref);

    relmr::JVec torq = Mmat*qddot_ref+Cref+G+(HinfSim_K_gamma)*(edot + HinfSim_Kv*e + HinfSim_Kp*eint);	
	//relmr::JVec torq = Mmat*ddq_ref+C+G;	
	//relmr::JVec torq = C+G;	
	return torq;
 }
void MR_Indy7_DualArm::FKinBody(const relmr::JVec q,const relmr::JVec qdot){
	mr::JVec q_l,q_r,qdot_r,qdot_l;
	q_r = q.segment<6>(0);
	q_l = q.segment<6>(6);
    qdot_r = qdot.segment<6>(0);
	qdot_l = qdot.segment<6>(6);
	this->T0r = this->R->T_b(q_r);
	this->T0l = this->L->T_b(q_l);
	this->Tbr = this->Tbr0*this->T0r;
	this->Tbl = this->Tbl0*this->T0l;
    this->Jb_r = this->R->J_b(q_r);
    this->Jb_l = this->L->J_b(q_l);
    this->Jbdot_r = this->R->Jdot_b(this->Jb_r,qdot_r);
    this->Jbdot_l = this->L->Jdot_b(this->Jb_l,qdot_l);
 }
relmr::JVec MR_Indy7_DualArm::GravityForces(const relmr::JVec q){
	relmr::JVec grav;
	mr::JVec q_l,q_r;
	q_r = q.segment<6>(0);
	q_l = q.segment<6>(6);
	mr::JVec grav_l = this->L->Gravity(q_l);
	mr::JVec grav_r = this->R->Gravity(q_r);
	grav.segment<6>(0) = grav_r;
	grav.segment<6>(6) = grav_l;
	return grav;
 }
 relmr::JVec MR_Indy7_DualArm::VelQuadraticForces(const relmr::JVec q,const relmr::JVec qdot){
	relmr::JVec c;
	mr::JVec q_l,q_r,qdot_l,qdot_r;
	q_r = q.segment<6>(0);
	q_l = q.segment<6>(6);
	qdot_r = qdot.segment<6>(0);
	qdot_l = qdot.segment<6>(6);
	mr::JVec c_l = this->L->VelQuadraticForces(q_l,qdot_l);
	mr::JVec c_r = this->R->VelQuadraticForces(q_r,qdot_r);
	c.segment<6>(0) = c_r;
	c.segment<6>(6) = c_l;
	return c;
 };
void MR_Indy7_DualArm::MRSetup(){
	Json::Value rootr;
	bool ret = ReadMRData_("MR_info.json",rootr);
	for(int i =0;i<6 ; i++){
		for(int j =0;j<this->jointnum;j++){
			this->Slist(i,j) = rootr["relSlist"][i][j].asDouble();
			this->Blist(i,j) = rootr["relBlist"][i][j].asDouble();
		}
	}	
    cout<<"=================relSlist================="<<endl;
    cout<<this->Slist<<endl;
    cout<<"=================relBlist================="<<endl;
    cout<<this->Blist<<endl;
	
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["relM"][i][j].asDouble();
		}
	}	

	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->Tbr0(i,j) = rootr["Tbl"][i][j].asDouble();
		}
	}    
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->Tbl0(i,j) = rootr["Tbr"][i][j].asDouble();
		}
	}    	
    cout<<"=================relM================="<<endl;
    cout<<this->M<<endl;    
    cout<<"=================Tbl0================="<<endl;
    cout<<this->Tbl0<<endl;   
    cout<<"=================Tbr0================="<<endl;
    cout<<this->Tbr0<<endl;    	 
	cout<<"END MRSetup"<<endl;

}
