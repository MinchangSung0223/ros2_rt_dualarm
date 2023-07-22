#include "FTread.h"

void check(const std::string& id, canStatus stat)
{
    if (stat != canOK) {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        std::cout << id << ": failed, stat=" << stat << " (" << buf << ")\n";
    }
}

std::string busStatToStr(const unsigned long flag)
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
        //std::cout << "CAN Status Event: " << busStatToStr(data->info.status.busStatus) << "\n";
        break;

    case canEVENT_ERROR:
       // std::cout << "CAN Error Event\n";
        break;

    case canEVENT_TX:
        //std::cout << "CAN Tx Event\n";
        break;

    case canEVENT_RX:
        //std::cout << "CAN Rx Event\n";
        break;
    }
    return;
}
FTread::FTread(){}
FTread::FTread(int channel, int canId_R,int canId_L){

        this->FT_R = Vector6d::Zero();
        this->filtered_FT_R = Vector6d::Zero();
        this->prev_filtered_FT_R = Vector6d::Zero();
        this->prev_FT_R = Vector6d::Zero();
        this->init_FT_R = Vector6d::Zero();
        
        this->FT_L = Vector6d::Zero();
        this->filtered_FT_L = Vector6d::Zero();
        this->prev_filtered_FT_L = Vector6d::Zero();
        this->prev_FT_L = Vector6d::Zero();
        this->init_FT_L = Vector6d::Zero();        
        
        this->init_flag = 0;
        this->channel = channel;
        this->canId_R = canId_R;
        this->canId_L = canId_L;        

        this->Fs = 100; //Hz
        this->dt = 0.01;
        this->wc = 105; 
        canInitializeLibrary();
        hnd = canOpenChannel(channel,
                                canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
        if (hnd < 0) {
            check("canOpenChannel", (canStatus)hnd);
        }
        stat = canSetNotify(hnd, notifyCallback,
                            canNOTIFY_RX | canNOTIFY_TX | canNOTIFY_ERROR | canNOTIFY_STATUS |
                                canNOTIFY_ENVVAR,
                            (char *)0);
        check("canSetNotify", stat);

        stat = canSetBusParams(hnd, canBITRATE_1M, 0, 0, 0, 0, 0);
        check("canSetBusParams", stat);
        if (stat != canOK) {
        
           cout<<"canSetBusParams Error"<<endl;
        }

        stat = canBusOn(hnd);
        check("canBusOn", stat);
        if (stat != canOK) {
            cout<<"canBus Error"<<endl;
        }

        long id = 0x102;
        unsigned char sendMsg[8] = {this->canId_R, 0x03, 0x01,0x00,0x00,0x00,0x00,0x00};

        stat = canWrite(hnd, id, sendMsg, sizeof(sendMsg)/sizeof(sendMsg[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }
        unsigned char sendMsg2[8] = {this->canId_L, 0x03, 0x01,0x00,0x00,0x00,0x00,0x00};

        stat = canWrite(hnd, id, sendMsg2, sizeof(sendMsg2)/sizeof(sendMsg2[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }        
        
        this->initialize();

}
void FTread::setId(){
        long id = 0x102;
        unsigned char sendMsg[8] = {this->canId_R, 0x01, 0x02,0x00,0x00,0x00,0x00,0x00};

        stat = canWrite(hnd, id, sendMsg, sizeof(sendMsg)/sizeof(sendMsg[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }
	sendMsg[0] = this->canId_L;

        stat = canWrite(hnd, id, sendMsg, sizeof(sendMsg)/sizeof(sendMsg[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }                

}
void FTread::setBias(){
        long id = 0x102;
        unsigned char sendMsg[8] = {this->canId_R, 0x02, 0x01,0x00,0x00,0x00,0x00,0x00};


        stat = canWrite(hnd, id, sendMsg, sizeof(sendMsg)/sizeof(sendMsg[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }
	sendMsg[0] = this->canId_L;

        stat = canWrite(hnd, id, sendMsg, sizeof(sendMsg)/sizeof(sendMsg[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }                

}
void FTread::clearBias(){
        long id = 0x102;
        unsigned char sendMsg[8] = {this->canId_R, 0x02, 0x02,0x00,0x00,0x00,0x00,0x00};


        stat = canWrite(hnd, id, sendMsg, sizeof(sendMsg)/sizeof(sendMsg[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }
	sendMsg[0] = this->canId_L;

        stat = canWrite(hnd, id, sendMsg, sizeof(sendMsg)/sizeof(sendMsg[0]), canMSG_STD);
        check("canWrite", stat);
        if (stat != canOK) {
            cout<<"canWrite Error"<<endl;
        }                

}
void FTread::initialize(){
    int init_count = 100;
    Vector6d FT_R=Vector6d::Zero();
    Vector6d FT_L=Vector6d::Zero();    
    Vector6d filtered_FT_R=Vector6d::Zero();
    Vector6d filtered_FT_L=Vector6d::Zero();    
    cout<<"FT Sensor Initialize...";

        for(int i = 0; i<init_count;i++){
		long id;
		unsigned char msg[8];
		unsigned int dlc;
		unsigned int flag;
		unsigned long time;
        
           stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_R){
			FT_R(0) = (msg[0]*256.0+msg[1])/100.0-300.0-init_FT_R(0);
			FT_R(1) =  (msg[2]*256.0+msg[3])/100.0-300.0-init_FT_R(1);
			FT_R(2) = (msg[4]*256.0+msg[5])/100.0-300.0-init_FT_R(2);   	        
		}

	  stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_R+1){
			FT_R(3) = (msg[0]*256+msg[1])/500.0-50.0-init_FT_R(3);
			FT_R(4) = (msg[2]*256+msg[3])/500.0-50.0-init_FT_R(4);
			FT_R(5) = (msg[4]*256+msg[5])/500.0-50.0-init_FT_R(5);
		 }

           stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_L){
			FT_L(0) = (msg[0]*256.0+msg[1])/100.0-300.0-init_FT_L(0);
			FT_L(1) =  (msg[2]*256.0+msg[3])/100.0-300.0-init_FT_L(1);
			FT_L(2) = (msg[4]*256.0+msg[5])/100.0-300.0-init_FT_L(2);   	        
		}

	  stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_L+1){
			FT_L(3) = (msg[0]*256+msg[1])/500.0-50.0-init_FT_L(3);
			FT_L(4) = (msg[2]*256+msg[3])/500.0-50.0-init_FT_L(4);
			FT_L(5) = (msg[4]*256+msg[5])/500.0-50.0-init_FT_L(5);
		 }

            double alpha  =(1.0/(1.0+this->dt*this->wc));;
            filtered_FT_R = alpha*filtered_FT_R + (1-alpha)*FT_R;
            filtered_FT_L = alpha*filtered_FT_L + (1-alpha)*FT_L;               

        }

        this->init_FT_L = filtered_FT_L;
        this->init_FT_R = filtered_FT_R;        

    
    cout<<"init R : "<<init_FT_R.transpose()    <<endl;
    cout<<"init L : "<<init_FT_L.transpose()    <<endl;    
    cout<<"Done"<<endl;
}
void FTread:: setCutOffFreq(double wc){
    this->wc = wc;
}
void FTread::print(Vector6d vec){

        std::cout <<std::showpos<<std::setw(6) <<std::fixed<< std::setprecision(4)<<vec(0)<<"  ";
        std::cout <<std::showpos<<std::setw(6)  <<std::fixed<< std::setprecision(4)<< vec(1)<<"  ";
        std::cout <<std::showpos<<std::setw(6)  <<std::fixed<< std::setprecision(4)<< vec(2)<<"  ";
        std::cout <<std::showpos<<std::setw(6)  <<std::fixed<< std::setprecision(4)<< vec(3)<<"  ";
        std::cout <<std::showpos<<std::setw(6)  <<std::fixed<< std::setprecision(4)<< vec(4)<<"  ";
        std::cout <<std::showpos<<std::setw(6)  <<std::fixed<< std::setprecision(4)<< vec(5)<<"\n";

}
 FTread::~FTread(){
    
 }
void FTread::readData(){
    
        long id;
        unsigned char msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;
        
           stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_R){
			FT_R(0) = (msg[0]*256.0+msg[1])/100.0-300.0-init_FT_R(0);
			FT_R(1) =  (msg[2]*256.0+msg[3])/100.0-300.0-init_FT_R(1);
			FT_R(2) = (msg[4]*256.0+msg[5])/100.0-300.0-init_FT_R(2);   	        
		}

	  stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_R+1){
			FT_R(3) = (msg[0]*256+msg[1])/500.0-50.0-init_FT_R(3);
			FT_R(4) = (msg[2]*256+msg[3])/500.0-50.0-init_FT_R(4);
			FT_R(5) = (msg[4]*256+msg[5])/500.0-50.0-init_FT_R(5);
		 }

           stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_L){
			FT_L(0) = (msg[0]*256.0+msg[1])/100.0-300.0-init_FT_L(0);
			FT_L(1) =  (msg[2]*256.0+msg[3])/100.0-300.0-init_FT_L(1);
			FT_L(2) = (msg[4]*256.0+msg[5])/100.0-300.0-init_FT_L(2);   	        
		}

	  stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(id == this->canId_L+1){
			FT_L(3) = (msg[0]*256+msg[1])/500.0-50.0-init_FT_L(3);
			FT_L(4) = (msg[2]*256+msg[3])/500.0-50.0-init_FT_L(4);
			FT_L(5) = (msg[4]*256+msg[5])/500.0-50.0-init_FT_L(5);
		 }
       
        double alpha  =(1.0/(1.0+this->dt*this->wc));
        this->filtered_FT_R = alpha*this->filtered_FT_R + (1-alpha)*this->FT_R;
        this->filtered_FT_L = alpha*this->filtered_FT_L + (1-alpha)*this->FT_L;                    
        this-> prev_FT_R = this->FT_R;
        this-> prev_FT_L = this->FT_L;        
}