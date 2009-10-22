#include <string>
#include "SputnikSDK.hh"
#include <cmath>
#include <algorithm>
//#include <vector>
#include <iostream>

#define MAXUSI 65536

void * taking_photo(void *ptr) {
  Sputnik_t *sputnik = (Sputnik_t *)ptr;
  while (sputnik->video_T>0&& sputnik->active) {
    ((DrRobotVideo_t*)sputnik->board[MEDIA]->device[VIDEO])->TakePhoto();
    if (sputnik->active)
      usleep(sputnik->video_T*1000*1000); // every video_T second take photo
  }
  return NULL;
}

unsigned char DeviceID2NR(unsigned char ID){
  switch (ID) {
  case 123:return 0;
  case 125:return 1;
  case 124:return 2;
  case  10:return 0;
  case   9:return 1;
  }	
  return 0;
}

Sputnik_t::Sputnik_t()
{
  boardSize=2;
  board=new DrRobotBoard_t*[boardSize];

  board[CONTROL] = new DrRobotBoard_t(CONTROL_ID,0.1);
  board[CONTROL]->deviceSize=3;
  board[CONTROL]->device=new DrRobotDevice_t*[board[CONTROL]->deviceSize];
  board[CONTROL]->device[MOTORS]=
    new DrRobotMotors_t(MOTORS_ID,board[CONTROL]);
  board[CONTROL]->device[CUSTOMDATA]=
    new DrRobotCustomData_t(CUSTOMDATA_ID,board[CONTROL]);
  board[CONTROL]->device[SENSORS]=
    new DrRobotSensors_t(SENSORS_ID,board[CONTROL]);


  board[MEDIA]=new DrRobotBoard_t(MEDIA_ID,0.1);
  board[MEDIA]->device=new DrRobotDevice_t*[3];
  board[MEDIA]->deviceSize=3;
  board[MEDIA]->device[AUDIO]=new DrRobotAudio_t(AUDIO_ID,board[MEDIA]);
  board[MEDIA]->device[VIDEO]=new DrRobotVideo_t(VIDEO_ID,board[MEDIA]);
  board[MEDIA]->device[POWERCONTROLER]=
    new DrRobotPowerControler_t(POWERCONTROLER_ID,board[MEDIA]);
  

  //SonarEMAAlpha=0.33;
  SonarEMAAlpha=1;
  SonarEMA[0]=255; SonarEMA[1]=255; SonarEMA[2]=255;
  //IREMAAlpha=0.2;
  IREMAAlpha=1;
  IREMA[0]=80; IREMA[1]=80; IREMA[2]=80; IREMA[3]=80;
  IREMA[4]=80; IREMA[5]=80; IREMA[6]=80; IREMA[7]=80;

  okno=20;
  HMEMAAlpha=0.5;
  HMresult = new unsigned char[2*okno];
  HAEMAAlpha=0.5;
  HAresult = new unsigned char[2*okno];

  SetVideo_T(0.5);
  InVoiceBuffer=NULL;
  InVoiceBufferSize=0;
  img = cvCreateImage(cvSize(176,144),IPL_DEPTH_8U,3);
}

Sputnik_t::~Sputnik_t()
{
  for (int i=0;i<boardSize;i++)
  	delete board[i];
  delete [] board;
}
bool Sputnik_t::Connect(const char* servername,unsigned int portno1,unsigned int portno2){
    bool result;
    result=board[CONTROL]->Connect(servername,portno1);
    if (result)
	result=board[MEDIA]->Connect(servername,portno2);
    return result;
}

void Sputnik_t::Disconnect()
{
  for(int i=0;i<boardSize;i++)
	board[i]->Disconnect();
}

void Sputnik_t::Start()
{
  for(int i=0;i<boardSize;i++)
	board[i]->start();
  
  face.EyesV=100;
  face.EyesH=100;
  face.NeckV=100;
  face.NeckH=100;
  face.Mouth=100;
  MoveEyesVH(0,0,10);
  MoveNeckVH(-40,0,10);
  MoveMouth(0,10);
  active=true;
  pthread_create( &videoThread, NULL, &taking_photo, this);
  board[CONTROL]->device[MOTORS]->EnableSensorSending();
  board[CONTROL]->device[CUSTOMDATA]->EnableSensorSending();
  board[CONTROL]->device[SENSORS]->EnableSensorSending();  
  board[MEDIA]->device[POWERCONTROLER]->EnableSensorSending();
  //board[CONTROL]->device[MOTORS]->DisableSensorSending();
  //board[CONTROL]->device[CUSTOMDATA]->DisableSensorSending();
  //board[CONTROL]->device[SENSORS]->DisableSensorSending();  
  //board[MEDIA]->device[POWERCONTROLER]->DisableSensorSending();
}

void Sputnik_t::Stop()
{
  active=false;
  pthread_join(videoThread,NULL);
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->ServoDisableAll();
  for(int i=0;i<boardSize;i++)
	board[i]->stop();
}

void Sputnik_t::MoveEyesVH(char positionV, char positionH, unsigned char speed)
{
  positionV=positionV>100?100:positionV;
  positionV=positionV<-100?-100:positionV;
  positionH=positionH>100?100:positionH;
  positionH=positionH<-100?-100:positionH;
  char dV=positionV-face.EyesV;
  char dH=positionH-face.EyesH;
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->
    ServoTimeCtrAll(0x8000,0x8000,0x8000,0x8000,
		    (1.5+0.4*(0.01*positionV))*2250,
		    (1.5+0.4*(0.01*positionH))*2250,
		    ((100-speed)*(std::max(abs(dV),abs(dH))/2)+64));
  //(1.0-sqrt(0.01*speed))*(16*256*sqrt(0.01*std::max(abs(dV),abs(dH))))+64);
  face.EyesV=positionV;
  face.EyesH=positionH;
}
void Sputnik_t::MoveNeckVH(char positionV, char positionH, unsigned char speed)
{
  positionV=positionV>100?100:positionV;
  positionV=positionV<-100?-100:positionV;
  positionH=positionH>100?100:positionH;
  positionH=positionH<-100?-100:positionH;
  char dV=positionV-face.NeckV;
  char dH=positionH-face.NeckH;
  if (dH!=0 || dV!=0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->
      ServoTimeCtrAll((1.3+0.6*(0.01*positionV))*2250,
		      (1.5+0.8*(0.01*positionH))*2250,
		      0x8000,0x8000,0x8000,0x8000,
		      ((100-speed)*(std::max(abs(dV),abs(dH))/2)+64));
  face.NeckV=positionV;
  face.NeckH=positionH;
}

void Sputnik_t::MoveMouth(unsigned char position,unsigned char speed)
{
  position=position>100?100:position;
  char dMouth=position-face.Mouth;
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->
    ServoTimeCtrAll(0x8000,0x8000,
		    (1.23+0.27*(0.01*position))*2250,
		    0x8000,0x8000,0x8000,
		    ((100-speed)*(abs(dMouth))/2+64));
  //(1.0-sqrt(0.01*speed))*(16*256*(0.01*abs(dMouth)))+64);
  face.Mouth=position;
}

unsigned char Sputnik_t::GetSonar(unsigned char channel)
{
  unsigned char result;
  if (channel>2)
    return 0;
  result=((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetSonar(channel);
  SonarEMA[channel]=SonarEMA[channel]+SonarEMAAlpha*(result-SonarEMA[channel]);
  return  SonarEMA[channel];
}

unsigned char Sputnik_t::GetIR(unsigned char channel)
{
  unsigned short int result;
  unsigned char distance;
  if (channel>7)
    return 0;
  if (channel==0)
    result=((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetIRRange();
  else if (channel==7)
    result=((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetTemperature();
  else
    result=((DrRobotCustomData_t*)board[CONTROL]->device[CUSTOMDATA])->GetAD(channel+1);
  distance=(22.0/(result*3.0/4095.0-0.15))+2;
  if (distance>80)
    distance=80;

  IREMA[channel]=IREMA[channel]+IREMAAlpha*(distance-IREMA[channel]);
  return  IREMA[channel];
}


unsigned char Sputnik_t::GetHumanMotion(unsigned char channel)
{
  unsigned char result_cpy[okno];
  unsigned char motion;
  int i;
  
  if (channel>1)
    return 0;
  for (i=0;i<okno-1;i++)
    HMresult[i+okno*channel]=HMresult[i+1+okno*channel];
  HMresult[okno-1+okno*channel]=std::abs(1.0*((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetHumanMotion(channel)-2048)*100/2048;
  for (i=0;i<okno;i++)
    result_cpy[i]=HMresult[i+okno*channel];
  motion=*(std::max_element(result_cpy,result_cpy+okno-1));
  if (motion<10)
    motion=0;
  if (motion>100)
    motion=100;
  HMEMA[channel]=HMEMA[channel]+HMEMAAlpha*(motion-HMEMA[channel]);
  return  HMEMA[channel];
}

unsigned char Sputnik_t::GetOverheat(unsigned char channel)
{
  unsigned char result;
  if (channel>1)
    return 0;
  result=100-1.0*(((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetOverheatAD(channel)-980)/11.6;
  return result;
}

float Sputnik_t::GetBattery(unsigned char channel)
{
  unsigned char maxv=9;
  if (channel>2)
    return 0;
  if (channel==1)
    maxv=24;
  return 1.0*(((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetBatteryAD(channel))*maxv/4095;
}

float Sputnik_t::GetVoltage(unsigned char channel)
{
  float result;
  if (channel>1)
    return 0;
  if (channel==0)
    result=1.0*(((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetRefVoltage())*6/4095;
  else
    result=1.0*(((DrRobotSensors_t*)board[CONTROL]->device[SENSORS])->GetPotVoltage())*6/4095;
  return result;
}

short int Sputnik_t::GetPotentiometer(unsigned char channel)
{
  unsigned short int result;
  if (channel>1 || sensor_type==ENCODER)
    return 0;
  if (sensor_type==POT_SINGLE)
    result=1.0*(((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->GetPot(channel)-2048)/4095*333+180;
  else // POT_DUAL
    result=1.0*(((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->GetPot(channel)-2214)/2214*180+180;
  return result;
}

float Sputnik_t::GetCurrent(unsigned char channel)
{
  if (channel>1)
    return 0;
  return 1.0*((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->GetCurrent(channel)/728;
}

char Sputnik_t::GetEncoderDir(unsigned char channel)
{
  if (channel>1)
    return 0;
  return ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->GetDir(channel);
}

unsigned short int Sputnik_t::GetEncoderPulse(unsigned char channel)
{
  if (channel>1)
    return 0;
  return ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->GetPulse(channel);
}

unsigned short int Sputnik_t::GetEncoderSpeed(unsigned char channel)
{
  if (channel>1)
    return 0;
  return ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->GetSpeed(channel);
}

void Sputnik_t::SetMotorPolarity(unsigned char channel,char polarity)
{
  if (channel>1)
    return;
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetPolarity(channel,polarity);
}

void Sputnik_t::ResumeMotors()
{
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->Resume(0);
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->Resume(1);
}

void Sputnik_t::SuspendMotors()
{
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->Suspend(0);
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->Suspend(1);
}

void Sputnik_t::SetPositionControlPID(unsigned short int Kp, 
			   unsigned short Kd, 
			   unsigned short int Ki_x100){
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetPositionControlPID(0,Kp,Kd,Ki_x100);
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetPositionControlPID(1,Kp,Kd,Ki_x100);
}

void Sputnik_t::SetVelocityControlPID(unsigned short int Kp, 
			   unsigned short Kd, 
			   unsigned short int Ki_x100){
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetVelocityControlPID(0,Kp,Kd,Ki_x100);
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetVelocityControlPID(1,Kp,Kd,Ki_x100);
}

void Sputnik_t::SetSensorUsage(unsigned char SensorType){
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetSensorUsage(0,SensorType);
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetSensorUsage(1,SensorType);
}

void Sputnik_t::SetControlMode(unsigned char ControlMode){
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetControlMode(0,ControlMode);
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->SetControlMode(1,ControlMode);
}

void Sputnik_t::PositionCtrSingle(unsigned char channel,
			    unsigned short int cmdValue,
			    unsigned short int timePeriod){
  if (channel>1)
    return;
  if (timePeriod>0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PositionTimeCtr(channel,cmdValue,timePeriod);
  else
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PositionNonTimeCtr(channel,cmdValue);    
}

void Sputnik_t::VelocityCtrSingle(unsigned char channel,
			    short int cmdValue,
			    unsigned short int timePeriod){
  if (channel>1)
    return;
  if (timePeriod>0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->VelocityTimeCtr(channel,channel==0?MAXUSI-cmdValue:cmdValue,timePeriod);
  else
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->VelocityNonTimeCtr(channel,channel==0?MAXUSI-cmdValue:cmdValue);    
}

void Sputnik_t::PwmCtrSingle(unsigned char channel,
		       unsigned short int cmdValue,
		       unsigned short int timePeriod){
  if (channel>1)
    return;
  if (timePeriod>0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PwmTimeCtr(channel,16363+cmdValue,timePeriod);
  else
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PwmNonTimeCtr(channel,16363+cmdValue);    
}

void Sputnik_t::PositionCtr(unsigned short int cmdValue0,
			    unsigned short int cmdValue1,
			    unsigned short int timePeriod){
  if (timePeriod>0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PositionTimeCtrAll(cmdValue0,cmdValue1,0x8000,0x8000,0x8000,0x8000,timePeriod);
  else
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PositionNonTimeCtrAll(cmdValue0,cmdValue1,0x8000,0x8000,0x8000,0x8000);    
}

void Sputnik_t::VelocityCtr(short int cmdValue0,
			    short int cmdValue1,
			    unsigned short int timePeriod){
    if (timePeriod>0)
	((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->VelocityTimeCtrAll(MAXUSI-cmdValue0-1,cmdValue1,0x8000,0x8000,0x8000,0x8000,timePeriod);
	//((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->VelocityTimeCtrAll(cmdValue0,cmdValue1,0x8000,0x8000,0x8000,0x8000,timePeriod);
    else
	((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->VelocityNonTimeCtrAll(MAXUSI-cmdValue0-1,cmdValue1,0x8000,0x8000,0x8000,0x8000);    
	//((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->VelocityNonTimeCtrAll(cmdValue0,cmdValue1,0x8000,0x8000,0x8000,0x8000);    
}

void Sputnik_t::PwmCtr(unsigned short int cmdValue0,
			    unsigned short int cmdValue1,
			    unsigned short int timePeriod){
  if (timePeriod>0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PwmTimeCtrAll(16363+cmdValue0,16363+cmdValue1,0x8000,0x8000,0x8000,0x8000,timePeriod);
  else
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->PwmNonTimeCtrAll(16363+cmdValue0,16363+cmdValue1,0x8000,0x8000,0x8000,0x8000);    
}

void Sputnik_t::EnableServos()
{
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->ServoEnableAll();
}

void Sputnik_t::DisableServos()
{
  ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->ServoDisableAll();
}


void Sputnik_t::ServoCtrSingle(unsigned char channel, 
			       unsigned short int cmdValue,
			       unsigned short int timePeriod){
  if (timePeriod>0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->ServoTimeCtr(channel,cmdValue,timePeriod);
  else
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->ServoNonTimeCtr(channel,cmdValue);
}

void Sputnik_t::ServoCtr(char neckV,char neckH,unsigned char mouth,
			 char eyeV, char eyeH,
			 unsigned short int timePeriod){
  if (timePeriod>0)
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->
      ServoTimeCtrAll((1.3+0.6*(0.01*neckV))*2250,
		      (1.5+0.8*(0.01*neckH))*2250,
		      (1.23+0.27*(0.01*mouth))*2250,0x8000,
		      (1.5+0.4*(0.01*eyeV))*2250,
		      (1.5+0.4*(0.01*eyeH))*2250,
		      timePeriod);
  else
    ((DrRobotMotors_t*)board[CONTROL]->device[MOTORS])->
      ServoNonTimeCtrAll((1.3+0.6*(0.01*neckV))*2250,
			 (1.5+0.8*(0.01*neckH))*2250,
			 (1.23+0.27*(0.01*mouth))*2250,0x8000,
			 (1.5+0.4*(0.01*eyeV))*2250,
			 (1.5+0.4*(0.01*eyeH))*2250);
}

void Sputnik_t::DisplayPGM(char* bmpFileName){
  
  ((DrRobotCustomData_t*)board[CONTROL]->device[CUSTOMDATA])->
    DisplayPGM(bmpFileName);
}


unsigned short int  Sputnik_t::GetBatVolt(unsigned char channel){
  if (channel>1)
    return 0;
  return ((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetBatVolt(channel);
}

unsigned short int  Sputnik_t::GetBatTemp(unsigned char channel){
  if (channel>1)
    return 0;
  return ((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetBatTemp(channel);
}

unsigned short int  Sputnik_t::GetDCINVolt(){
  return ((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetDCINVolt();
}

bool  Sputnik_t::Charging(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetStatus()&&0x04);
}

bool Sputnik_t::Powerfail(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetStatus()&&0x08);
}

bool  Sputnik_t::DCINCompOut(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetStatus()&&0x10);
}
bool  Sputnik_t::LowerPower(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetStatus()&&0x20);
}
bool  Sputnik_t::Fault(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetStatus()&&0x40);
}

unsigned short int Sputnik_t::GetRefVolt(){
  return ((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetRefVolt();
}

bool Sputnik_t::PoweredDCIN(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetStatus()&&0x20);
}

bool Sputnik_t::PoweredBat2(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetPowerPath()&&0x40);
}

bool Sputnik_t::PoweredBat1(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetPowerPath()&&0x80);
}

bool Sputnik_t::ChargeBat2(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetChargePath()&&0x40);
}

bool Sputnik_t::ChargeBat1(){
  return (((DrRobotPowerControler_t*)board[MEDIA]->device[POWERCONTROLER])->GetChargePath()&&0x80);
}


void Sputnik_t::SetVideo_T(float T){
  video_T=T;
}

IplImage* Sputnik_t::getIplImage(){
  int i,j;
  for(i=0;i<144;i++) {
      for(j=0;j<176;j++) {
	  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0]=
	    *(((DrRobotVideo_t*)board[MEDIA]->device[VIDEO])->rgbArr+i*176*3+j*3+0);
	  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1]=
	    *(((DrRobotVideo_t*)board[MEDIA]->device[VIDEO])->rgbArr+i*176*3+j*3+1);
	  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2]=
	    *(((DrRobotVideo_t*)board[MEDIA]->device[VIDEO])->rgbArr+i*176*3+j*3+2);
	}
  }
  //return ((DrRobotVideo_t*)board[MEDIA]->device[VIDEO])->getIplImage();
  return img;
}
  
	
	unsigned char* Sputnik_t::getRGBImage(){
	  return ((DrRobotVideo_t*)board[MEDIA]->device[VIDEO])->rgbArr;
	}
      
	unsigned int Sputnik_t::getWidth(){
	  return ((DrRobotVideo_t*)board[MEDIA]->device[VIDEO])->width;
	}
	
	unsigned int Sputnik_t::getHeight(){
	  return ((DrRobotVideo_t*)board[MEDIA]->device[VIDEO])->height;
	}

void Sputnik_t::StartRecord(unsigned char VoiceSegment){
  ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->StartRecord(VoiceSegment);
}
void Sputnik_t::StopRecord(){
  ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->StopRecord();
}
void Sputnik_t::StartAudioPlay(){
  if (InVoiceBufferSize>0) {
    ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->StartAudioPlay();
    ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->SendVoiceBuffer(InVoiceBuffer,InVoiceBufferSize);
    //((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->StopAudioPlay();
  }
}
void Sputnik_t::StopAudioPlay(){
    ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->StopAudioPlay();
}

void Sputnik_t::SaveWAV(char * FileName){
  ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])
    ->SaveWAV(FileName,
	      ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->
	      GetAudioBuffer(),
	      ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->
	      GetAudioBufferLength());
}
void Sputnik_t::ReadWAV(char* FileName){
  int VoiceLength;
  ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])
    ->ReadWAV(FileName,&InVoiceBuffer,&InVoiceBufferSize,&VoiceLength);
}
short* &Sputnik_t::GetInVoiceBuffer(){
  return InVoiceBuffer;
}
long &Sputnik_t::GetInVoiceBufferSize(){
  return InVoiceBufferSize;
}
short* Sputnik_t::GetOutVoiceBuffer(){
  return ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->GetAudioBuffer();
}
long Sputnik_t::GetOutVoiceBufferSize(){
  return ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->
    GetAudioBufferLength();
}

void Sputnik_t::ClearOutVoiceBuffer(){
  return ((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->
    ClearAudioBuffer();
}

void Sputnik_t::SetSensorUsage(unsigned char SensorType){
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetSensorUsage(0, SensorType);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetSensorUsage(1, SensorType);
}
void Sputnik_t::SetControlMode(unsigned char ControlMode){
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetControlMode(0,ControlMode);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetControlMode(1,ControlMode);
}
void Sputnik_t::SetPositionControlPID(unsigned short int Kp, unsigned short Kd, unsigned short int Ki_x100){
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetPositionControlPID(0,Kp,Kd,Ki_x100);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetPositionControlPID(1,Kp,Kd,Ki_x100);
}
void Sputnik_t::SetVelocityControlPID(unsigned short int Kp, unsigned short Kd, unsigned short int Ki_x100){
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetVelocityControlPID(0,Kp,Kd,Ki_x100);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetVelocityControlPID(1,Kp,Kd,Ki_x100);
}

/*
 * void Sputnik_t::CopyOutToInVoiceBuffer(){
 *  delete []InVoiceBuffer;
 *  InVoiceBufferSize=((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->
 *    GetAudioBufferLength();
 *  memcpy((char*)InVoiceBuffer,
 *	 (char*)((DrRobotAudio_t*)board[MEDIA]->device[AUDIO])->
 *	 GetAudioBuffer(), 
 *	 InVoiceBufferSize*sizeof(short));
 * }
 */
