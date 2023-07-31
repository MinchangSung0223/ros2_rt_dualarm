#include "MR_Indy7.h"

bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData(const char* filename,Json::Value &rootr){
	cout<<"START ReadMRData"<<endl;
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile(filename, readBuffer, BufferLength)) {
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


MR_Indy7::MR_Indy7() {
    // Constructor implementation

	this->Slist.resize(6,6);
	this->Blist.resize(6,6);	
	this->Glist;	
	this->Mlist;			
	this->M.resize(4,4);	    
    this->q.resize(6);	
    this->q_des.resize(6);	
    this->dq_des.resize(6);	
    this->ddq_des.resize(6);	
    this->dq.resize(6);	
    this->g.resize(3);
    this->torq.resize(6);

    this->g<<0,0,-9.8;
    this->Kp = mr::Matrix6d::Zero();
    this->Kv = mr::Matrix6d::Zero();
    this->Ki = mr::Matrix6d::Zero();
    this->Hinf_Kp = mr::Matrix6d::Zero();
    this->Hinf_Kv = mr::Matrix6d::Zero();
    this->Hinf_K_gamma = mr::Matrix6d::Zero();
    this->eef_mass = 0;
    double orn_scale = 0.01;
    this->Imp_A = 1*mr::Matrix6d::Identity();
    Imp_A(0,0)= 300.0;
    Imp_A(1,1)= 300.0;
    Imp_A(2,2)= 300.0;
    Imp_A(3,3)= 200.0;
    Imp_A(4,4)= 200.0;
    Imp_A(5,5)= 200.0;
    this->Imp_D = 20*mr::Matrix6d::Identity();
    Imp_D(0,0)= 15000.0;
    Imp_D(1,1)= 15000.0;
    Imp_D(2,2)= 15000.0;
    Imp_D(3,3)= 15000.0;
    Imp_D(4,4)= 15000.0;
    Imp_D(5,5)= 15000.0;
    this->Imp_K = 100*mr::Matrix6d::Identity();
    Imp_K(0,0)= 100000.0;
    Imp_K(1,1)= 100000.0;
    Imp_K(2,2)= 100000.0;
    Imp_K(3,3)= 100000.0;
    Imp_K(4,4)= 100000.0;
    Imp_K(5,5)= 100000.0;
    this->invImp_A = this->Imp_A.inverse();
    mr::JVec invL2sqr;
    invL2sqr<<1000,1000,800,600,600,600;
    for (int i=0; i<6; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 50+1.0/invL2sqr(0) ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 50+1.0/invL2sqr(1) ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 50.0+1.0/invL2sqr(2) ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 50.0+1.0/invL2sqr(3) ;

            break;
        case 4:
              Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 50.0+1.0/invL2sqr(4) ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 50.0+1.0/invL2sqr(5) ;

            break;
        }
    }
   for (int i=0; i<6; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 100.0;
            Kv(i,i) = 5.0;
            Ki(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 100.0;
            Kv(i,i) = 5.0;
            Ki(i,i)=10.0;
            break;
        case 2:
            Kp(i,i) = 100.0;
            Kv(i,i) = 5.0;
            Ki(i,i)=5.0;
            break;
        case 3:
            Kp(i,i) = 60.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=3.0;
            break;
        case 4:
            Kp(i,i) = 60.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=3.0;
            break;
        case 5:
            Kp(i,i) = 60.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=1.0;
            break;
        }
    }
}

JVec MR_Indy7::Gravity( JVec q){
         return mr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist,this->eef_mass) ; 
}
void MR_Indy7::saturationMaxTorque(JVec &torque, JVec MAX_TORQUES){
    for(int i =0;i<JOINTNUM;i++){
        if(abs(torque(i))> MAX_TORQUES(i)){
            if(torque(i)>0) torque(i) = MAX_TORQUES(i);
            else torque(i) = -MAX_TORQUES(i);
        }
    }
}

Matrix6xn MR_Indy7::Mmat(JVec q)
{
    return mr::MassMatrix(q,this->Mlist, this->Glist, this->Slist,this->eef_mass);
}

JVec MR_Indy7::Cvec(JVec q, JVec dq)
{
    return mr::VelQuadraticForces(q, dq,this->Mlist, this->Glist, this->Slist,this->eef_mass);
}

JVec MR_Indy7::Gvec(JVec q)
{
    return mr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist,this->eef_mass) ; 
}

Jacobian MR_Indy7::J_s(JVec q)
{
    return mr::JacobianSpace(this->Slist, q);
}

Jacobian MR_Indy7::J_b(JVec q)
{
    return mr::JacobianBody(this->Blist, q);
}
Jacobian MR_Indy7::Jdot_b(mr::Jacobian J_b,JVec qdot)
{
    return mr::dJacobianBody(J_b, qdot);
}
SE3 MR_Indy7::T_s(JVec q)
{
    return mr::FKinSpace(this->M, this->Slist, q);
}

SE3 MR_Indy7::T_b(JVec q)
{
    return mr::FKinBody(this->M, this->Blist, q);
}
JVec MR_Indy7::ForwardDynamics(const JVec q,const JVec qdot,const JVec tau,Vector6d Ftip){
   return mr::ForwardDynamics(q,qdot,tau,this->g,Ftip,this->Mlist,this->Glist,this->Slist,this->eef_mass);
}
MassMat MR_Indy7::MassMatrix(const JVec q){    
    MassMat M =mr::MassMatrix(q,this->Mlist, this->Glist, this->Slist) ;

    return M; 
}

JVec MR_Indy7::VelQuadraticForces( const JVec q,const JVec dq){
         return mr::VelQuadraticForces(q,dq,this->Mlist, this->Glist, this->Slist) ; 
}
JVec MR_Indy7::ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des){
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    MassMat Mmat = mr::MassMatrix(q,this->Mlist, this->Glist, this->Slist,this->eef_mass);
    JVec H=InverseDynamics(q, dq, JVec::Zero(),this->g,Vector6d::Zero(), this->Mlist,this->Glist, this->Slist,this->eef_mass);
    JVec ddq_ref = Kv*edot+Kp*e;
    JVec torq = Mmat*ddq_ref+H;
    return torq;
}
JVec MR_Indy7::ComputedTorquePIDControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec& eint){
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    MassMat Mmat = mr::MassMatrix(q,this->Mlist, this->Glist, this->Slist,this->eef_mass);
    JVec H=InverseDynamics(q, dq, JVec::Zero(),this->g,Vector6d::Zero(), this->Mlist,this->Glist, this->Slist,this->eef_mass);
    JVec ddq_ref = Kv*edot+Kp*e+Ki*eint;
    JVec torq = Mmat*ddq_ref+H;
    return torq;
}
JVec MR_Indy7::HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec& eint){
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    MassMat Mmat = mr::MassMatrix(q,this->Mlist, this->Glist, this->Slist,this->eef_mass);
    JVec C = mr::VelQuadraticForces(q, dq,this->Mlist, this->Glist, this->Slist,this->eef_mass);
    JVec G = mr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist,this->eef_mass) ; 
    JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
    JVec torq = Mmat*ddq_ref+C+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    return torq;
}
Vector6d flip_(Vector6d V){
	Vector6d V_flip;
	V_flip<<V(3),V(4),V(5),V(0),V(1),V(2);
	return V_flip;
}

JVec MR_Indy7::ImpedanceControl( JVec q,JVec qdot,Vector6d Ftip, SE3 X, Jacobian Jb, Jacobian Jbdot,SE3 X_des, JVec V_des,JVec Vdot_des,Vector6d F_des,MassMat Mmat, JVec  C, JVec G){
    
        SE3 invX = TransInv(X);
        se3 X_err = invX*X_des;
        se3 invX_err = TransInv(X_err);
        Vector6d V = Jb*qdot;
        Vector6d V_err = V_des - Adjoint(invX_err)*V;
        Vector6d F_err = 0.5*(F_des - Adjoint(X_err)*Ftip);
        Vector6d lambda = flip_(se3ToVec(MatrixLog6(X_err)));
		Vector6d dlambda = dlog6(-lambda)*flip_(V_err);
		Vector6d gamma = dexp6(-lambda).transpose()*flip_(F_err);

		Matrix6d A_lambda = dexp6(-lambda).transpose()*Imp_A*dexp6(-lambda);
		Matrix6d D_lambda = dexp6(-lambda).transpose()*Imp_D*dexp6(-lambda) + A_lambda*ddlog6(-lambda,-dlambda);
		Matrix6d K_lambda = dexp6(-lambda).transpose()*Imp_K*dexp6(-lambda);
		//Matrix6d KV = dlog6(-lambda)*(invImp_A*Imp_D*dexp6(-lambda) + dexp6(-lambda)*ddlog6(-lambda,-dlambda));
		//Matrix6d KP = dlog6(-lambda)*invImp_A*Imp_K*dexp6(-lambda);
		//Matrix6d KG = dlog6(-lambda)*invImp_A*dlog6(-lambda).transpose();

        Matrix6d KV = invImp_A*Imp_D;
		Matrix6d KP = invImp_A*Imp_K;
		Matrix6d KG = invImp_A;


        Vector6d ddlambda_ref = -KV*dlambda -KP*lambda + KG*gamma;
		Vector6d dV_ref = Adjoint(X_err)*(Vdot_des- flip_(dexp6(-lambda)*ddlambda_ref)  + ad(V_err)*V_des - flip_(ddexp6(-lambda,-dlambda)*dlambda));
        double eps = 0.001;
        JVec ddq_ref = Jb.transpose()*(Jb*Jb.transpose()+eps*Matrix6d::Identity()).inverse()*(dV_ref - Jbdot*qdot);
        //JVec ddq_ref = Jb.inverse()*(dV_ref - Jbdot*qdot);
        JVec tau_c = Mmat*ddq_ref + C+G ;
        JVec tau = tau_c + Jb.transpose()*Ftip;
        //JVec tau = tau_c + Jb.transpose()*Ftip;

    return tau;
}


void MR_Indy7::MRSetup(){
	Json::Value rootr;
	bool ret = ReadMRData("MR_info.json",rootr);
	for(int i =0;i<6 ; i++){
		for(int j =0;j<6;j++){
			this->Slist(i,j) = rootr["Slist"][i][j].asDouble();
			this->Blist(i,j) = rootr["Blist"][i][j].asDouble();
		}
	}	
    cout<<"=================Slist================="<<endl;
    cout<<this->Slist<<endl;
    cout<<"=================Blist================="<<endl;
    cout<<this->Blist<<endl;
	for(int i = 0;i< rootr["Mlist"].size(); i++){
		MatrixXd M = MatrixXd::Identity(4,4);
		for(int j = 0;j< rootr["Mlist"][0].size(); j++){
			for(int k = 0;k< rootr["Mlist"][0][0].size(); k++){
				M(j,k) = rootr["Mlist"][i][j][k].asDouble();
			}
		}
        cout<<"=================M"<<i<<"============================"<<endl;
        cout<<M<<endl;

		char str[50];		
		this->Mlist.push_back(M);
	}
	for(int i = 0;i< rootr["Glist"].size(); i++){
		MatrixXd G = MatrixXd::Identity(6,6);
		for(int j = 0;j< rootr["Glist"][0].size(); j++){
			for(int k = 0;k< rootr["Glist"][0][0].size(); k++){
				G(j,k) = rootr["Glist"][i][j][k].asDouble();
			}
		}
        cout<<"=================G"<<i<<"============================"<<endl;
        cout<<G<<endl;
		char str[50];		
		this->Glist.push_back(G);	}	
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["M"][i][j].asDouble();
		}
	}	
    cout<<"=================M================="<<endl;
    cout<<this->M<<endl;    
	cout<<"END MRSetup"<<endl;
}
