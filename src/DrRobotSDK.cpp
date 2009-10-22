#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include "DrRobotSDK.hh"
#include "uyvy2rgb.hh"
#include <algorithm>

#include <fcntl.h>

#define LOOP_DELAY_LONG  200000
#define LOOP_DELAY_SHORT 5000

/*****************************************************************************/
/*            Range and Distance Sensors                                     */
/*****************************************************************************/

DrRobotSensors_t::DrRobotSensors_t(unsigned char dID, DrRobotBoard_t *board): DrRobotDevice_t(dID,board){
}

void DrRobotSensors_t::RefreshData(unsigned char *dataBuffer,unsigned long dataLength){
  memcpy((char*)&(SensorData),dataBuffer,sizeof(SensorData));
}


void DrRobotSensors_t::SystemSensorSending(int PacketNumber){
  unsigned char length;
  unsigned char *data;  
  if (PacketNumber<0)  {
    length=0;
    data=NULL;
  }
  else {
    length=1;
    if (PacketNumber>255) PacketNumber=255;
    data=(unsigned char *)calloc(1,sizeof(unsigned char));
    *data=PacketNumber;
  }
  SendData(125,length,data);
}

void DrRobotSensors_t::EnableSensorSending(){
  SystemSensorSending(-1);
}

void DrRobotSensors_t::DisableSensorSending(){
  SystemSensorSending(0);
}

unsigned char DrRobotSensors_t::GetSonar(unsigned char channel){
  return SensorData.Sonar[channel];
}

unsigned short int DrRobotSensors_t::GetHumanAlarm(unsigned char channel) {
  switch (channel) {
  case 0: return SensorData.HumanAlarm1;
  case 1: return SensorData.HumanAlarm2;    
  default: return 0;
  }
}
unsigned short int DrRobotSensors_t::GetHumanMotion(unsigned char channel) {
  switch (channel) {
  case 0: return SensorData.HumanMotion1;
  case 1: return SensorData.HumanMotion2;
  default: return 0;
  }
}


unsigned short int DrRobotSensors_t::GetTiltingX(){
  return SensorData.TiltingX;
}
unsigned short int DrRobotSensors_t::GetTiltingY(){
  return SensorData.TiltingY;
}


unsigned short int DrRobotSensors_t::GetOverheatAD(unsigned char channel){
  return SensorData.Overheat[channel];
}

unsigned short int DrRobotSensors_t::GetTemperature(){
  return SensorData.Temperature;
}

unsigned char DrRobotSensors_t::GetIRCode(unsigned char channel){
  return SensorData.IRCode[channel];
}

unsigned short int DrRobotSensors_t::GetIRRange(){
  return SensorData.IRRange;
}

unsigned short int DrRobotSensors_t::GetBatteryAD(unsigned char channel){
  return SensorData.BatteryAD[channel];
}

unsigned short int DrRobotSensors_t::GetRefVoltage(){
  return SensorData.RefVoltage;
}
unsigned short int DrRobotSensors_t::GetPotVoltage(){
  return SensorData.PotVoltage;
}


/******************************************************************************/
/*                  Motor Control Sensors                                     */
/******************************************************************************/

DrRobotMotors_t::DrRobotMotors_t(unsigned char dID, DrRobotBoard_t *board): DrRobotDevice_t(dID,board){
}

void DrRobotMotors_t::RefreshData(unsigned char *dataBuffer,unsigned long dataLength){
  memcpy((char*)&(MotorData),dataBuffer,sizeof(MotorData));
}

void DrRobotMotors_t::SystemSensorSending(int PacketNumber){
  unsigned char length;
  unsigned char *data;
  if (PacketNumber<0)  {
    length=0;
  }
  else {
    length=1;
    if (PacketNumber>255) PacketNumber=255;
    data=(unsigned char *)calloc(1,sizeof(unsigned char));
    *data=PacketNumber;
  }
  SendData(123,length,data);
}

void DrRobotMotors_t::EnableSensorSending(){
  SystemSensorSending(-1);
}

void DrRobotMotors_t::DisableSensorSending(){
  SystemSensorSending(0);
}

unsigned short int DrRobotMotors_t::GetPot(unsigned char channel){
  return MotorData.Pot[channel];
}

unsigned short int DrRobotMotors_t::GetCurrent(unsigned char channel){
  return MotorData.Current[channel];
}

char DrRobotMotors_t::GetDir(unsigned char channel){
  //return MotorData.Dir;
  switch (channel)
    {
    case 0: return -(((0x01&MotorData.Dir)==1)*2-1); break;
    case 1: return ((0x02&MotorData.Dir)==2)*2-1; break;
    }
  return 0;
}

unsigned short int DrRobotMotors_t::GetPulse(unsigned char channel){
  switch (channel) 
    {
    case 0: return MotorData.Pulse1;
    case 1: return MotorData.Pulse2;
    }
  return 0;
}

unsigned short int DrRobotMotors_t::GetSpeed(unsigned char channel){
  switch (channel) 
    {
    case 0: return MotorData.Speed1;
    case 1: return MotorData.Speed2;
    }
  return 0;
}


/*****************************************************************************/
/*                        Motor Control                                      */
/*****************************************************************************/

void DrRobotMotors_t::SetPolarity(unsigned char channel, char polarity){
  unsigned char length=3;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0x06;
  *(data+1)=channel;
  switch (polarity) {
  case 1: *(data+2)=1; break;
  case -1: *(data+2)=0xFF; break;
  default: return;
  }
  SendData(7,length,data);  
}

void DrRobotMotors_t::Resume( unsigned char channel) {
  unsigned char length=2;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=1;
  *(data+1)=channel;
  SendData(30,length,data);  
}

void DrRobotMotors_t::ResumeAll() {
  unsigned char j;
  for (j=0;j<6;j++)
    Resume( j);
}

void DrRobotMotors_t::Suspend( unsigned char channel) {
  unsigned char length=2;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0;
  *(data+1)=channel;
  SendData(30,length,data);  
}

void DrRobotMotors_t::SuspendAll() {
  unsigned char j;
  for (j=0;j<6;j++)
    Suspend( j);
}

void DrRobotMotors_t::SetPositionControlPID( 
					    unsigned char channel,
					    unsigned short int Kp, 
					    unsigned short Kd, 
					    unsigned short int Ki_x100){
  unsigned char length=5;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0x07;
  *(data+1)=channel;
  *(data+2)=0x01; //Kp
  *(data+3)=Kp%256;
  *(data+4)=Kp/256;
  SendData(7,length,data);
  *data=0x07;
  *(data+1)=channel;
  *(data+2)=0x02; //Kd
  *(data+3)=Kd%256;
  *(data+4)=Kd/256;
  SendData(7,length,data);
  *data=0x07;
  *(data+1)=channel;
  *(data+2)=0x03; //Ki_x100
  *(data+3)=Ki_x100/100;
  *(data+4)=Ki_x100%100;
  SendData(7,length,data);
}
void DrRobotMotors_t::SetVelocityControlPID( 
					    unsigned char channel,
					    unsigned short int Kp, 
					    unsigned short Kd, 
					    unsigned short int Ki_x100){
  unsigned char length=5;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0x08;
  *(data+1)=channel;
  *(data+2)=0x01; //Kp
  *(data+3)=Kp%256;
  *(data+4)=Kp/256;
  SendData(7,length,data);
  *data=0x08;
  *(data+1)=channel;
  *(data+2)=0x02; //Kd
  *(data+3)=Kd%256;
  *(data+4)=Kd/256;
  SendData(7,length,data);
  *data=0x08;
  *(data+1)=channel;
  *(data+2)=0x03; //Ki_x100
  *(data+3)=Ki_x100/100;
  *(data+4)=Ki_x100%100;
  SendData(7,length,data);
}


void DrRobotMotors_t::SetSensorUsage( unsigned char channel,
				      unsigned char SensorType){
  unsigned char length=3;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0x0d;
  *(data+1)=channel;
  *(data+2)=SensorType;
  SendData(7,length,data);
}

void DrRobotMotors_t::SetControlMode( unsigned char channel,
				      unsigned char ControlMode){
  unsigned char length=3;
  unsigned char *data;
 
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0x0e;
  *(data+1)=channel;
  *(data+2)=ControlMode;
  SendData(7,length,data);
}

void DrRobotMotors_t::PositionTimeCtr( unsigned char channel,
				       unsigned short int cmdValue,
				       unsigned short int timePeriod){
  unsigned char length=6;
  unsigned char *data;
  
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  *(data+3)=28;
  *(data+4)=(unsigned char)(timePeriod%256);
  *(data+5)=(unsigned char)(timePeriod/256);
  SendData(3,length,data);
}

void DrRobotMotors_t::PositionNonTimeCtr( unsigned char channel,
					  unsigned short int cmdValue){
  unsigned char length=3;
  unsigned char *data;
  
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  SendData(3,length,data);
}

void DrRobotMotors_t::VelocityTimeCtr( unsigned char channel,
				       unsigned short int cmdValue,
				       unsigned short int timePeriod){
  unsigned char length=6;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  *(data+3)=28;
  *(data+4)=(unsigned char)(timePeriod%256);
  *(data+5)=(unsigned char)(timePeriod/256);
  SendData(26,length,data);
}

void DrRobotMotors_t::VelocityNonTimeCtr( unsigned char channel,
					  unsigned short int cmdValue){
  unsigned char length=3;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  SendData(26,length,data);
}

void DrRobotMotors_t::PwmTimeCtr( unsigned char channel,
				  unsigned short int cmdValue,
				  unsigned short int timePeriod){
  unsigned char length=6;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  *(data+3)=28;
  *(data+4)=(unsigned char)(timePeriod%256);
  *(data+5)=(unsigned char)(timePeriod/256);
  SendData(5,length,data);
}
void DrRobotMotors_t::PwmNonTimeCtr( unsigned char channel,
				     unsigned short int cmdValue){
  unsigned char length=3;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  SendData(5,length,data);
}

void DrRobotMotors_t::PositionTimeCtrAll( short int cmd1,
					  short int cmd2, short int cmd3,
					  short int cmd4, short int cmd5,
					  short int cmd6, 
					  unsigned short int timePeriod){
  unsigned char length=14;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  *(data+12)=(unsigned char)(timePeriod%256);
  *(data+13)=(unsigned char)(timePeriod/256);
  SendData(4,length,data);
}

void DrRobotMotors_t::PositionNonTimeCtrAll( short int cmd1,
					     short int cmd2, short int cmd3,
					     short int cmd4, short int cmd5,
					     short int cmd6){
  unsigned char length=12;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  SendData(4,length,data);
}


void DrRobotMotors_t::VelocityTimeCtrAll( unsigned short int cmd1,
					  unsigned short int cmd2, unsigned short int cmd3,
					  unsigned short int cmd4, unsigned short int cmd5,
					  unsigned short int cmd6, 
					  unsigned short int timePeriod){
  unsigned char length=14;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  *(data+12)=(unsigned char)(timePeriod%256);
  *(data+13)=(unsigned char)(timePeriod/256);
  SendData(27,length,data);
}

void DrRobotMotors_t::VelocityNonTimeCtrAll( unsigned short int cmd1,
					     unsigned short int cmd2, unsigned short int cmd3,
					     unsigned short int cmd4, unsigned short int cmd5,
					     unsigned short int cmd6){
  unsigned char length=12;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  SendData(27,length,data);
}

void DrRobotMotors_t::PwmTimeCtrAll( short int cmd1,
				     short int cmd2, short int cmd3,
				     short int cmd4, short int cmd5,
				     short int cmd6, 
				     unsigned short int timePeriod){
  unsigned char length=14;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  *(data+12)=(unsigned char)(timePeriod%256);
  *(data+13)=(unsigned char)(timePeriod/256);
  SendData(6,length,data);
}

void DrRobotMotors_t::PwmNonTimeCtrAll( short int cmd1,
					short int cmd2, short int cmd3,
					short int cmd4, short int cmd5,
					short int cmd6){
  unsigned char length=12;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  SendData(6,length,data);
}

/*****************************************************************************/
/*              RC Servo Motor Control                                       */
/*****************************************************************************/



void DrRobotMotors_t::ServoEnable( unsigned char channel) {
  unsigned char length=2;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=1;
  *(data+1)=6+channel;
  SendData(30,length,data);  
}

void DrRobotMotors_t::ServoEnableAll() {
  unsigned char j;
  for (j=0;j<6;j++)
    ServoEnable(j);
}

void DrRobotMotors_t::ServoDisable( unsigned char channel) {
  unsigned char length=2;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0;
  *(data+1)=6+channel;
  SendData(30,length,data);  
}

void DrRobotMotors_t::ServoDisableAll() {
  unsigned char j;
  for (j=0;j<6;j++)
    ServoDisable(j);
}

void DrRobotMotors_t::ServoTimeCtr( unsigned char channel, 
				    unsigned short int cmdValue,
				    unsigned short int timePeriod){
  unsigned char length=6;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  *(data+3)=28;
  *(data+4)=(unsigned char)(timePeriod%256);
  *(data+5)=(unsigned char)(timePeriod/256);
  SendData(28,length,data);
}
void DrRobotMotors_t::ServoNonTimeCtr( unsigned char channel, 
				       unsigned short int cmdValue){
  unsigned char length=3;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=channel;
  *(data+1)=cmdValue%256;
  *(data+2)=cmdValue/256;
  SendData(28,length,data);
}

void DrRobotMotors_t::ServoTimeCtrAll( short int cmd1, short int cmd2,
				       short int cmd3, short int cmd4, short int cmd5,
				       short int cmd6, unsigned short int timePeriod){
  unsigned char length=14;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  *(data+12)=(unsigned char)(timePeriod%256);
  *(data+13)=(unsigned char)(timePeriod/256);
  SendData(29,length,data);
}

void DrRobotMotors_t::ServoNonTimeCtrAll( short int cmd1, 
					  short int cmd2, short int cmd3, short int cmd4, 
					  short int cmd5, short int cmd6){
  unsigned char length=12;
  unsigned char *data;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=cmd1%256;
  *(data+1)=cmd1/256;
  *(data+2)=cmd2%256;
  *(data+3)=cmd2/256;
  *(data+4)=cmd3%256;
  *(data+5)=cmd3/256;
  *(data+6)=cmd4%256;
  *(data+7)=cmd4/256;
  *(data+8)=cmd5%256;
  *(data+9)=cmd5/256;
  *(data+10)=cmd6%256;
  *(data+11)=cmd6/256;
  SendData(29,length,data);
}

/******************************************************************************/
/*                  Custom Ports                                              */
/******************************************************************************/

DrRobotCustomData_t::DrRobotCustomData_t(unsigned char dID, DrRobotBoard_t *board): DrRobotDevice_t(dID,board){
}

void DrRobotCustomData_t::RefreshData(unsigned char *dataBuffer,unsigned long dataLength){
  memcpy((char*)&(CustomData),dataBuffer,sizeof(CustomData));
}

void DrRobotCustomData_t::SystemSensorSending(int PacketNumber){
  unsigned char length;
  unsigned char *data;
  if (PacketNumber<0)  {
    length=0;
  }
  else {
    length=1;
    if (PacketNumber>255) PacketNumber=255;
    data=(unsigned char *)calloc(1,sizeof(unsigned char));
    *data=PacketNumber;
  }
  SendData(124,length,data);
}

void DrRobotCustomData_t::EnableSensorSending(){
  SystemSensorSending(-1);
}

void DrRobotCustomData_t::DisableSensorSending(){
  SystemSensorSending(0);
}

unsigned short int DrRobotCustomData_t::GetAD(unsigned char channel){
  return CustomData.Channel[channel];
}

unsigned char DrRobotCustomData_t::GetDIN(){
  return CustomData.Digital; 
}

void DrRobotCustomData_t::SetDIN(unsigned char value) {
  SendData(22,1,&value);
}
/*****************************************************************************/
/*                   Multimedia Control                                      */
/*****************************************************************************/

#define LINE_LENGTH 80

unsigned char DrRobotCustomData_t::ReadPGM(FILE *p, int image_pgm[64][128], int *dimX, 
					   int *dimY, int *grayness){
  char s[LINE_LENGTH];
  int znak,koniec=0,i,j;
  int pixel;

  if (p==NULL) {
    return(1); // Wrong file pointer
  }

  if (fgets(s,LINE_LENGTH,p)==NULL) koniec=1;
  if ( (s[0]!='P') || (s[1]!='2') || koniec) {
    return(2); // Wrong file type - no magic number
  }

  do {
    if ((znak=fgetc(p))=='#') { // skip comments
      if (fgets(s,LINE_LENGTH,p)==NULL)
	koniec=1;
    }  
    else {
      ungetc(znak,p);
    }
  } while (! koniec && znak=='#');

  /* reading dimentions */ 
  if (fscanf(p,"%d %d",dimX,dimY)!=2) {
    return(3); // lack of dimentions
  }       
  if (fscanf(p,"%d",grayness)!=1) {
    fprintf(stderr,"Blad: Brak liczby stopni szarosci\n");
    return(4); // lack of grayness
  }       
  //image_pgm=calloc((*dimX)*(*dimY),sizeof(int));
  /* writing image to the bffer image_pgm*/
  for (j=0;j<*dimY;j++) {
    for (i=0;i<*dimX;i++) {
      if (fscanf(p,"%d",&pixel)!=1) {
	return(5); // wrong dimmentions
      }
      else {
	image_pgm[j][i]=(pixel<=*grayness/2); // converting to mono
      }
    }
  }
  return 0;
}

unsigned char DrRobotCustomData_t::DisplayPGM(char *bmpFileName){
  FILE *plik;
  int image_pgm[64][128];
  int dimX,dimY, grayness;
  unsigned char length=65;
  unsigned char *data;
  int x,y;
  unsigned char frame=0;
  unsigned char value=0;
  unsigned char multiplyier=128;
  unsigned char result=0;
  
  
	 
  plik=fopen(bmpFileName,"r");

  result=ReadPGM(plik, image_pgm, &dimX, &dimY, &grayness);
  if (result>0)
    return result;
  fclose(plik);
  if (dimX!=128) return 11;
  if (dimY!=64) return 12;

  data=(unsigned char*)calloc(length,sizeof(unsigned char));  
  while (frame<16) 
    {
      *data=frame;
      for(x=0;x<dimX/2;x++) {
	value=0;
	multiplyier=128;
	for(y=7;y>=0;y--) {
	  value+=multiplyier*image_pgm[y+frame/2*8][x+(dimX/2)*(frame%2)];
	  multiplyier/=2;
	}
	*(data+x+1)=value;
      }
      //result+=(1<<frame)*(SendData(23,length,data)>0);
      SendData(23,length,data);
      frame++;
    }
  return result;
}


/****************************************************************************/
/*                  Low Level Protection                                    */
/****************************************************************************/

void DrRobotCustomData_t::EnableBumperProtection(){
  unsigned char length=2;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0x13;
  *(data+1)=1;
  SendData(7,length,data);
}

void DrRobotCustomData_t::DisableBumperProtection(){
  unsigned char length=2;
  unsigned char *data;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=0x13;
  *(data+1)=0;
  SendData(7,length,data);
}


/******************************************************************************/
/*                  Power Controler                                           */
/******************************************************************************/


DrRobotPowerControler_t::DrRobotPowerControler_t(unsigned char dID, DrRobotBoard_t *board): DrRobotDevice_t(dID,board){
}

void DrRobotPowerControler_t::RefreshData(unsigned char *dataBuffer,unsigned long dataLength){
  memcpy((char*)&(PowerControler),dataBuffer,sizeof(PowerControler));
}

void DrRobotPowerControler_t::SystemSensorSending(int PacketNumber){
  unsigned char length;
  unsigned char *data;
  if (PacketNumber<0)  {
    length=0;
  }
  else {
    length=1;
    if (PacketNumber>255) PacketNumber=255;
    data=(unsigned char *)calloc(1,sizeof(unsigned char));
    *data=PacketNumber;
  }
  SendData(124,length,data);
}

void DrRobotPowerControler_t::EnableSensorSending(){
  SystemSensorSending(-1);
}

void DrRobotPowerControler_t::DisableSensorSending(){
  SystemSensorSending(0);
}

unsigned short int DrRobotPowerControler_t::GetBatVolt(unsigned char channel){
  if (channel==0)
    return PowerControler.Bat1Volt;
  else if (channel==1)
    return PowerControler.Bat2Volt;
  else return 0;
}

unsigned short int DrRobotPowerControler_t::GetBatTemp(unsigned char channel){
  if (channel==0)
    return PowerControler.Bat1Temp;
  else if (channel==1)
    return PowerControler.Bat2Temp;
  else return 0;
}

unsigned short int DrRobotPowerControler_t::GetDCINVolt(){
  return PowerControler.DCINVolt;
}

unsigned char DrRobotPowerControler_t::GetStatus(){
  return PowerControler.Status;
}

unsigned short int DrRobotPowerControler_t::GetRefVolt(){
  return PowerControler.RefVolt;
}

unsigned char DrRobotPowerControler_t::GetPowerPath(){
  return PowerControler.PowerPath;
}

unsigned char DrRobotPowerControler_t::GetChargePath(){
  return PowerControler.ChargePath;
}


/*****************************************************************************/
/*                                                                           */
/*                            PMB5010                                        */
/*                                                                           */
/*****************************************************************************/

#define min(X,Y) ((X)>(Y)?(Y):(X))

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */

struct WAVEFileHeader{
  unsigned short wFormatTag;
  unsigned short nChannels;
  unsigned long nSamplesPerSec;
  unsigned long nAvgBytesPerSec;
  unsigned short nBlockAlign;
  unsigned short wBitsPerSample;
};

struct  FileHeader
{
  unsigned long lRiff;
  unsigned long lFileSize;
  unsigned long lWave ;
  unsigned long lFormat;
  unsigned long lFormatLength;
};

struct ChunkHeader
{
  unsigned long lType;
  unsigned long lLen;
};

#pragma pack(pop) 

#define MaxImageSize		18000

DrRobotVideo_t::DrRobotVideo_t(unsigned char dID, DrRobotBoard_t *board): DrRobotDevice_t(dID,board){
  imageBufferPtr = 0;
  takingPhotoFlag = 0;
  Last_Line_Num= 0;
  Line_Num_Wrong=0;
  height=144;
  width=176;
  //img = cvCreateImage(cvSize(176,144),IPL_DEPTH_8U,3);
}

void DrRobotVideo_t::RefreshData(unsigned char *dataBuffer,unsigned long dataLength)
{
  int i;
  unsigned char lineNumber = dataBuffer[0];//dataBuffer[INDEX_DATA];
  unsigned char Length = dataLength;//dataBuffer[INDEX_LENGTH];
  //printf(" Line: %d %d \n",(int) lineNumber,(int)dataLength);
  //For get rid of the residal 
  if(lineNumber ==0) {
    takingPhotoFlag = 2; //Find the first line
    Last_Line_Num = 0;
    Line_Num_Wrong = 0;
  }
  
  if(takingPhotoFlag == 2) {
    if((lineNumber!=0) && (lineNumber!=0xff) && 
       (lineNumber !=(Last_Line_Num+1))) {
      Line_Num_Wrong = 1;
      //printf("Last line num: %d,,, received line num:%d\n",
      //	     Last_Line_Num,lineNumber);
    }
    
    switch(lineNumber){ // old format: 0xFF represents the end of 176*144 picture.
    case 0xFF: {
      for(int i=0;i< Length - 3;i++) {
	//imageBuffer[imageBufferPtr] = dataBuffer[INDEX_DATA+ 3+i];
	imageBuffer[imageBufferPtr] = dataBuffer[3+i];
	//TRACE("Last line : %d\n",imageBuffer[imageBufferPtr]);
	imageBufferPtr ++;
      }
      //TRACE("Image size : %d\n",imageBufferPtr);
      
      ////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////
      // should be in separate thread because it is time consuming
      // end not crutial for incoming data package recognition
      if(!Line_Num_Wrong)
	{
	  DealWithVideo();
	}
      //else
      //	printf("Wrong video line\n");
      
      ////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////
      //btestImage=1;
      imageBufferPtr=0;
      
      break;}
      
    default: {
      for(i=0;i< Length - 1;i++) {
	if(imageBufferPtr < MaxImageSize) {
	  //imageBuffer[imageBufferPtr] = dataBuffer[INDEX_DATA + 1+i];
	  imageBuffer[imageBufferPtr] = dataBuffer[1+i];	  
	  imageBufferPtr ++;
	}
      }	
      break;}
    }
    Last_Line_Num = lineNumber;
  }
  
}

void DrRobotVideo_t::DealWithVideo()
{
  //int i,j;
  UYVY2RGB(70,imageBuffer,imageBufferPtr,rgbArr,144,176);
  /*
  for(i=0;i<144;i++)
    {
      for(j=0;j<176;j++)
	{
	  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0]=
	    *(rgbArr+i*176*3+j*3+0);
	  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1]=
	    *(rgbArr+i*176*3+j*3+1);
	  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2]=
	    *(rgbArr+i*176*3+j*3+2);
	}
    }
  */
}

/*
IplImage* DrRobotVideo_t::getIplImage(){
  return img;
}
*/

void DrRobotVideo_t::TakePhoto(){
  SendData(0x20,0,NULL);
}


DrRobotAudio_t::DrRobotAudio_t(unsigned char dID, DrRobotBoard_t *board): DrRobotDevice_t(dID,board){
  adpcm_init(); 
  audioBufIndex=0;
  lastAudioLength=0;
  audioSegment = 4;
}

void DrRobotAudio_t::RefreshData(unsigned char *dataBuffer,unsigned long dataLength){
  DealWithAudio(dataBuffer, dataLength);
}

void DrRobotAudio_t::DealWithAudio(unsigned char * lpInData, int nLen)
{
  lpOutData = new short[nLen*2];
  // decode the data into an internal buffer with 'ADPCM decoding method'
  adpcm_decoder( lpInData, lpOutData, nLen*2) ;
  memcpy((char *)audioBuf+audioBufIndex, lpOutData, 256*2);
  audioBufIndex += 512;
  if( audioBufIndex >= audioSegment * 4096 * 10){
    lastAudioLength=audioBufIndex;
    memcpy((char *)lastAudio, (char*)audioBuf, audioBufIndex*sizeof(short));
    audioBufIndex = 0;
    StopRecord();
  }
  delete lpOutData;
}

/* 
 * VoiceSegment from 1 to 10, number of 0.256 miliseconds of recorded voice
 */
void DrRobotAudio_t::StartRecord(unsigned char VoiceSegment){
  unsigned char length=1;
  unsigned char *data;
  audioSegment=VoiceSegment;
  data=(unsigned char*)calloc(length,sizeof(unsigned char));
  *data=VoiceSegment;
  SendData(0x0B,length,data);
}

void DrRobotAudio_t::StopRecord(){
  SendData(0x0C,0,NULL);
}


void DrRobotAudio_t::StartAudioPlay(){
  SendData(0x0D,0,NULL);
}


void DrRobotAudio_t::StopAudioPlay(){
  SendData(0x0E,0,NULL);
}

void DrRobotAudio_t::SendAudio(unsigned char *data,
			       unsigned char length) {
  SendData(0x0A,length,data);
}


void DrRobotAudio_t::adpcm_init(){
  state.valprev = 0; /* Previous output value */
  state.index = 0; /* Index into stepsize table */
}



void DrRobotAudio_t::adpcm_decoder(unsigned char *indata, short *outdata,
				   int len)
{
  static int indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
  };
  static int stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
  };
  signed char *inp;       /* Input buffer pointer */
  short *outp;            /* output buffer pointer */
  int sign;               /* Current adpcm sign bit */
  int delta;              /* Current adpcm output value */
  int step;               /* Stepsize */
  int valpred;            /* Predicted value */
  int vpdiff;             /* Current change to valpred */
  int index;              /* Current step change index */
  int inputbuffer;        /* place to keep next 4-bit value */
  int bufferstep;         /* toggle between inputbuffer/input */
  outp = outdata;
  inp = (signed char *)indata;
  valpred = state.valprev;
  index = state.index;
  step = stepsizeTable[index];
  bufferstep = 0;
  for ( ; len > 0 ; len-- ) {
    /* Step 1 - get the delta value */
    if ( bufferstep ) {
      delta = inputbuffer & 0xf;
    } else {
      inputbuffer = *inp++;
      delta = (inputbuffer >> 4) & 0xf;
    }
    bufferstep = !bufferstep;
    /* Step 2 - Find new index value (for later) */
    index += indexTable[delta];
    if ( index < 0 ) index = 0;
    if ( index > 88 ) index = 88;
    /* Step 3 - Separate sign and magnitude */
    sign = delta & 8;
    delta = delta & 7;
    /* Step 4 - Compute difference and new predicted value */
    /*
     * Computes 'vpdiff = (delta+0.5)*step/4', but see comment
     * in adpcm_coder.
     */
    vpdiff = step >> 3;
    if ( delta & 4 ) vpdiff += step;
    if ( delta & 2 ) vpdiff += step>>1;
    if ( delta & 1 ) vpdiff += step>>2;
    if ( sign )
      valpred -= vpdiff;
    else
      valpred += vpdiff;
    /* Step 5 - clamp output value */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;
    /* Step 6 - Update step value */
    step = stepsizeTable[index];
    /* Step 7 - Output value */
    *(outp++) = valpred;
  }
  state.valprev = valpred;
  state.index = index;
}


// sample point to the RAW PCM audio data, bLength is the compressed 
// audio data length,
//
void DrRobotAudio_t::adpcm_coder(short *indata, unsigned char *outdata,
				 int len)
{
  static int indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
  };
  static int stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
  };
  short *inp;                 /* Input buffer pointer */
  unsigned char *outp;          /* output buffer pointer */
  int val;                    /* Current input sample value */
  int sign;                   /* Current adpcm sign bit */
  int delta;                  /* Current adpcm output value */
  int diff;                   /* Difference between val and valprev */
  int step;                   /* Stepsize */
  int valpred;                /* Predicted output value */
  int vpdiff;                 /* Current change to valpred */
  int index;                  /* Current step change index */
  int outputbuffer;           /* place to keep previous 4-bit value */
  int bufferstep;             /* toggle between outputbuffer/output */
  outp = outdata;
  inp = indata;
  valpred = state.valprev;
  index = state.index;
  step = stepsizeTable[index];
  bufferstep = 1;
  for ( ; len > 0 ; len-- ) {
    val = *inp++;
    /* Step 1 - compute difference with previous value */
    diff = val - valpred;
    sign = (diff < 0) ? 8 : 0;
    if ( sign ) diff = (-diff);
    /* Step 2 - Divide and clamp */
    /* Note:
    ** This code *approximately* computes:
    ** delta = diff*4/step;
    ** vpdiff = (delta+0.5)*step/4;
    ** but in shift step bits are dropped. The net result of this is
    ** that even if you have fast mul/div hardware you cannot put it to
    ** good use since the fixup would be too expensive.
    */
    delta = 0;
    vpdiff = (step >> 3);
    if ( diff >= step ) {
      delta = 4;
      diff -= step;
      vpdiff += step;
    }
    step >>= 1;
    if ( diff >= step ) {
      delta |= 2;
      diff -= step;
      vpdiff += step;
    }
    step >>= 1;
    if ( diff >= step ) {
      delta |= 1;
      vpdiff += step;
    }
    /* Step 3 - Update previous value */
    if ( sign )
      valpred -= vpdiff;
    else
      valpred += vpdiff;
    /* Step 4 - Clamp previous value to 16 bits */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;
    /* Step 5 - Assemble value, update index and step values */
    delta |= sign;
    index += indexTable[delta];
    if ( index < 0 ) index = 0;
    if ( index > 88 ) index = 88;
    step = stepsizeTable[index];
    /* Step 6 - Output value */
    if ( bufferstep ) {
      outputbuffer = (delta << 4) & 0xf0;
    } else {
      *outp++ = (delta & 0x0f) | outputbuffer;
    }
    bufferstep = !bufferstep;
  }
  /* Output last step, if needed */
  if ( !bufferstep )
    *outp++ = outputbuffer;
  state.valprev = valpred;
  state.index = index;
}

void DrRobotAudio_t::SaveWAV(char * FileName, short * audioBuf, long audioBufIndex) 
{
  struct WAVEFileHeader waveFormatHeader;
  struct  FileHeader audioFileHeader;
  struct ChunkHeader ch;

  audioFileHeader.lRiff = 0x46464952;  //RIFF
  audioFileHeader.lFileSize = (sizeof(audioFileHeader) + 
			       sizeof(waveFormatHeader) + sizeof(ch) 
			       + audioBufIndex*sizeof(short))-8; 
  audioFileHeader.lWave = 0x45564157;			//WAVE
  audioFileHeader.lFormat = 0x20746D66;		//.fmt
  audioFileHeader.lFormatLength = sizeof(waveFormatHeader);
  
  waveFormatHeader.wFormatTag = 1; // PCM           
  waveFormatHeader.nChannels = 1;  // Mono            
  waveFormatHeader.nSamplesPerSec = 8000; //8k/s     
  waveFormatHeader.nAvgBytesPerSec = 16000; // 16kHz
  waveFormatHeader.nBlockAlign = 2;  
  waveFormatHeader.wBitsPerSample = 16; //16 bits
  
  ch.lType = 0x61746164;  
  ch.lLen =  audioBufIndex;//*sizeof(short);
  
  printf("Opening file \n");
  FILE *  audiofile = fopen(FileName, "wb");
  
  fwrite(&audioFileHeader, sizeof(audioFileHeader),1,audiofile);
  fwrite(&waveFormatHeader,sizeof(waveFormatHeader),1,audiofile); 
  fwrite(&ch, sizeof(ch),1,audiofile); 	
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
  fwrite( audioBuf, sizeof(short),audioBufIndex, audiofile );   
#pragma pack(pop)
  fclose(audiofile);
}


void DrRobotAudio_t::ReadWAV(char* lpFilePath,short** VoiceBuffer, 
			     long* VoiceBufferSize, int *VoiceLength)
{
  struct WAVEFileHeader waveFormatHeader;
  struct  FileHeader audioFileHeader;
  struct ChunkHeader ch;
  FILE  *rP;
  char *extraData;
  long extraDataLength;
  if ((rP = fopen(lpFilePath,"rb")) == NULL) {
    printf("No such file\n");
    return;
  }

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
  memset((void*)&waveFormatHeader,0,sizeof(waveFormatHeader));
  memset((void*)&audioFileHeader,0,sizeof(audioFileHeader));
  memset((void*)&ch,0,sizeof(ch));

  fread(&audioFileHeader, sizeof(audioFileHeader),1,rP);
  /*
    for (unsigned char i=0;i<sizeof(audioFileHeader);i++)
    printf("%x ",*((char*)(&audioFileHeader)+i));
    printf("\n");
  */
  fread(&waveFormatHeader, sizeof(waveFormatHeader),1,rP);
  /*
    for (int i=0;i<sizeof(waveFormatHeader);i++)
    printf("%x ",*((char*)(&waveFormatHeader)+i));
    printf("\n");
  */
  extraDataLength=audioFileHeader.lFormatLength-16;
  
  if (extraDataLength>0){
    extraData=(char*)calloc(extraDataLength,sizeof(char));
    fread(extraData, sizeof(char),extraDataLength,rP);
    free(extraData);
  }

  extraDataLength=4;
  extraData=(char*)calloc(extraDataLength,sizeof(char));
  fread(extraData, sizeof(char),extraDataLength,rP);
  
  if (extraData[0]==0x66 && extraData[1]==0x61 &&
      extraData[2]==0x63 && extraData[3]==0x74) {
    fread(extraData, sizeof(char),extraDataLength,rP);
    fread(extraData, sizeof(char),extraDataLength,rP);
    fread(&ch, sizeof(ch),1,rP);
  }
  else if (*((long*)extraData)==0x61746164) {
    memcpy((char*)&ch,extraData,extraDataLength);
    fread(&ch.lLen,sizeof(ch.lLen),1,rP);
  }
  free(extraData);

  
  //fread(&ch, sizeof(ch),1,rP); 	
  *VoiceBuffer = (short*)calloc(ch.lLen/2,sizeof(short));
  //#pragma pack(pop)
  //#pragma pack(push)  /* push current alignment to stack */
  //#pragma pack(2)     /* set alignment to 1 byte boundary */
  *VoiceBufferSize = fread(*VoiceBuffer,sizeof(short), ch.lLen, rP);
  //#pragma pack(pop)
  /*
   * if(*VoiceBufferSize < 16000)
   *   VoiceLength = 1;
   * else if(*VoiceBufferSize < 16000 * 20)
   *   VoiceLength = 2;
   * else if(*VoiceBufferSize < 16000 * 32)
   *   VoiceLength = 3;
   * else 
   *   VoiceLength = 4;
   */

  
  fclose(rP);
}


#define DP_MAX		  128		     //audio data packet size
void DrRobotAudio_t::SendVoiceBuffer(short* VoiceBuffer, 
				     long VoiceBufferSize){
  short sample[DP_MAX*2];
  long Offset=0;
  unsigned char bLength;
  unsigned char dataToSend[2*DP_MAX];
  static char audioBuf[4*DP_MAX];//1024000];
  static long audioBufIndex=0;
  short *lpOutData;

  struct timespec delay;
  audioDelay=20; //ms

  while ( (Offset < VoiceBufferSize-2) ) {
    bLength = min( DP_MAX, (VoiceBufferSize - Offset)/2 ); 
    memcpy(sample, VoiceBuffer +  Offset, bLength*4); 
    /*
      adpcm_coder:
      sample: 1short(2bytes) * 128 * 2 = 128 * 2 words  
      encoded: 128 bytes
      The third argument is the number of samples. 
    */
    adpcm_coder(sample,dataToSend, bLength*2);


    lpOutData = new short[bLength*2];
    adpcm_decoder( dataToSend, lpOutData, bLength*2) ;
    audioBufIndex = bLength;
    memcpy((char *)audioBuf, lpOutData, audioBufIndex*2);

    SendAudio(dataToSend,bLength);
    Offset += bLength * 2;
    //packet size is 128
    //In order to play 16k*16bits*mono music, after adpcm compression,
    //you need to send music to robot on this speed at least: 
    //16k*2bytes/4 per second=8kbytes/s=64kbps
    //A packet (132 bytes) includes 128 bytes audio data. 
    //So you need to send 8kBytes/128 packet/second= 62 packet/second
    //So send 20 packet, you need at least 1/3 second.
    //If it's less than 1/3 second, such as 1/4 second (cause we need some
    //compression time, etc.), it's good, but you do not need to 
    //send faster than 1/4 second too much, cause you need to spare bandwidth
    //to other data. That's why we need sleep for a while.
    //If it's more than 1/4 second, it's not good, cause we can not play 
    //music smoothly.
    
    //wait 10 ms
    usleep(audioDelay*1000);
    
  }
}

short * DrRobotAudio_t::GetAudioBuffer(){
  return lastAudio;
}

long DrRobotAudio_t::GetAudioBufferLength(){
  return lastAudioLength;
}

void DrRobotAudio_t::ClearAudioBuffer(){
  memset(lastAudio,0,lastAudioLength*sizeof(short));
  lastAudioLength=0;
}

void DrRobotAudio_t::SlowDownAudio()
{
  audioDelay+=20;
}


/**********************************************************************/
/*                         NEW API                                    */
/**********************************************************************/




DrRobotDevice_t::DrRobotDevice_t(unsigned char dID,DrRobotBoard_t *board)
{
  this->dID=dID;
  this->board=board;
}

void DrRobotDevice_t::SendData(unsigned char ddID,unsigned short dataLength,unsigned char* data){
  board->SendData(ddID,dataLength,data);
}

Listener_t::Listener_t(DrRobotBoard_t *board,
		       unsigned long int totalsize=70000):totalsize(totalsize)
{
  this->board=board;
  dl=0;
  data=new unsigned char[totalsize];
  active=false;
  pthread_mutex_init(&mutex, NULL);
  analizerThread=NULL;
  listenerThread=NULL;
}

Listener_t::~Listener_t()
{
  delete []data;
}

void Listener_t::start()
{
  int iret=0;
  active=true; 
  iret=pthread_create( &analizerThread, NULL, &data_analizer, this);
  iret+=pthread_create( &listenerThread, NULL, &listener_function, this);
  if (iret>0)
    active=false; 
}

void Listener_t::stop()
{
  active=false;
  pthread_join( listenerThread, NULL);
  pthread_join( analizerThread, NULL);
}

void * data_analizer(void *ptr) {
  Listener_t *listener = (Listener_t *)ptr;
  unsigned char dataFrame[270];
  unsigned int rdl;
  unsigned char wrongFrame=0;
  while (listener->active) {    
    wrongFrame=0;
    if (listener->data==NULL || listener->dl==0){
      usleep(LOOP_DELAY_LONG);
      continue;
    }
    if (listener->dl<9){
      usleep(LOOP_DELAY_LONG);
      continue;
    }
    rdl=listener->data[5]+9;
    if (listener->dl<rdl){
      usleep(LOOP_DELAY_LONG);
      continue;
    }
    if (listener->data[0]!=94 || listener->data[1]!=02) {
      //printf("wrong header\n");
      wrongFrame+=1;
    }
    if (listener->data[listener->data[5]+7]!=94 || 
	listener->data[listener->data[5]+8]!=13) {
      //printf("wrong footer\n");
      wrongFrame+=10;
    }     
    if (listener->board->CalculateCRC(listener->data+2,listener->data[5]+4) 
	!= listener->data[listener->data[5]+6]) {
      wrongFrame+=100;
      //printf("wrong CRC\n");
    }
    if (wrongFrame>0) {
      if (wrongFrame%2==1){ // we did't found proper header lets search for it
	unsigned int idm=0;
	while (idm <listener->dl-1 && (listener->data[idm]!=94 || listener->data[idm+1]!=02))
	  idm++;
	//if (idm==listener->dl-2) // we reach the end of the data in the buffor
	//  rdl=listener->dl;      // so we flush all data from the buffer
	//else
	rdl=idm;
      }
      pthread_mutex_lock(&listener->mutex); // prevent multiacces to board->data 
      listener->dl=listener->dl-rdl;
      memmove(listener->data,listener->data+rdl,listener->dl); // remove coppied data
      pthread_mutex_unlock(&listener->mutex); // remove the lock
    }
    else {
      pthread_mutex_lock(&listener->mutex); // prevent multiacces to board->data 
      memcpy(dataFrame,listener->data,rdl);   // copy frame with data 
      listener->dl=listener->dl-rdl;
      memmove(listener->data,listener->data+rdl,listener->dl); // remove coppied data
      pthread_mutex_unlock(&listener->mutex); // remove the lock
      // send frame with data to recognition function
      listener->DealWithPacket(dataFrame,rdl);
    }
    if (listener->dl>0)
      usleep(LOOP_DELAY_SHORT);
    else
      usleep(LOOP_DELAY_LONG);
  } 
  return NULL;
}


void * listener_function(void *ptr){
  Listener_t *listener = (Listener_t *)ptr;
  unsigned char* buffor;
  int bl;
  buffor=new unsigned char[listener->totalsize];
  bl=0; listener->dl=0;
  while (listener->active) {
    bl=recv(listener->board->getSockfd(), buffor, listener->totalsize*sizeof(unsigned char), 0);
    if (bl<=0) {
      usleep(LOOP_DELAY_SHORT);
      continue;
    }
    if (listener->dl+bl<listener->totalsize){ // if there is enough space move data from
      pthread_mutex_lock( &listener->mutex );
      memcpy(listener->data+listener->dl,buffor,bl); // the socket buffor to 
      //std::copy(buffor,buffor+bl,listener->data+listener->dl); // the socket buffor to 
      listener->dl=listener->dl+bl;                  // the data buffor
      pthread_mutex_unlock( &listener->mutex );
    }
    usleep(LOOP_DELAY_SHORT);
  } 
  // terminate data analizer thread
  if (buffor!=NULL) 
    free(buffor);
  return NULL;
}


bool Listener_t::DealWithPacket(unsigned char* data, int nLen){
  switch (data[INDEX_TYPE]) {
  case COMTYPE_SYSTEM: {
    switch (data[INDEX_DATA]){
    case DATA_ACK:
      //printf("receive acknowledgement packet!\n");
      break;
    case DATA_PING:
      //printf("receive ping packet! \n");
      board->SendAck();
      //printf("send acknowledge!\n");
      break;
    case DATA_URGENT_DATA:
      //printf("receive urgent packet!\n");
      board->SendAck();
      break;
    case DATA_SKIPPACKET:
      //printf("receive skip packet!\n");
      board->SendAck();
      break;
    default:
      //printf("invalid packet data in system packet, discard it!\n");
      return false;
    }
    break ;
  }
  case COMTYPE_SENSOR:
    //printf("receive Sensor data packet!\n");
    break ;
  case COMTYPE_AUDIO: {
    //printf("receive Audio data packet!\n");
    board->device[DeviceID2NR(data[INDEX_TYPE])]->RefreshData(data+INDEX_DATA, data[INDEX_LENGTH]);
    //StopRecord(board->sockfd);
    break ;
  }
  case COMTYPE_VIDEO: {
    //printf("receive Video data packet!\n");
    board->device[DeviceID2NR(data[INDEX_TYPE])]->RefreshData(data+INDEX_DATA, data[INDEX_LENGTH]); 
    break;
  }
  case COMTYPE_MOT_MOTOR_SENSOR:
    board->device[DeviceID2NR(data[INDEX_TYPE])]->RefreshData(data+INDEX_DATA, data[INDEX_LENGTH]);
    //printf("receive motor sensor data packet!\n");
    break;
  case COMTYPE_MOT_USER_SENSOR:
    board->device[DeviceID2NR(data[INDEX_TYPE])]->RefreshData(data+INDEX_DATA, data[INDEX_LENGTH]);
    //printf("receive user sensor data packet! %d %d\n", data[INDEX_TYPE],data[INDEX_LENGTH]);
    break;
  case COMTYPE_MOT_ULTRASONIC_SENSOR:
    board->device[DeviceID2NR(data[INDEX_TYPE])]->RefreshData(data+INDEX_DATA, data[INDEX_LENGTH]);
    //printf("receive user ultrasonic data packet!\n");
    break;
  case AUDIO_PLAY_BUFFER_READY:
    printf("Please Slow down the Transmit\n");
    ((DrRobotAudio_t*)board->device[DeviceID2NR(data[INDEX_TYPE])])->SlowDownAudio();
    break;
  case CMTYPE_ADPCM_RESET: {
    //printf("recieve ADPCM reset\n");
    ((DrRobotAudio_t*)(board->device[DeviceID2NR(data[INDEX_TYPE])]))->adpcm_init();
    break ;
  }
  default:
    //printf("invalid packet data type(%#2X), discard it!\n", (unsigned char)data[INDEX_TYPE] );
    printf("invalid packet data type(%d), discard it!\n", (unsigned char)data[INDEX_TYPE] );
    return false;
  }
  return true;
}

void *heartbeat_function(void *ptr){
  DrRobotBoard_t *board=(DrRobotBoard_t *)ptr;
  while (board->active) {
    board->SendPing();
    usleep(board->heartbeat_T*1000*1000);
  }
  return NULL;
}

DrRobotBoard_t::DrRobotBoard_t(unsigned char robotID,float T):
  rID(robotID),listener(this),heartbeat_T(T)
{
}

DrRobotBoard_t::~DrRobotBoard_t(){
  for (unsigned char it=0;it<deviceSize;it++)
    delete device[it];
  delete [] device;
}

unsigned char DrRobotBoard_t::CalculateCRC(unsigned char *lpBuffer, unsigned char nSize)
{
  unsigned char shift_reg, sr_lsb, data_bit, v;
  int i,j;
  unsigned char fb_bit;
  
  shift_reg = 0; // initialize the shift register
  
  for(i=0;i<nSize;i++)    {
    v = (unsigned char)(lpBuffer[i]&0x0000FFFF);
    for(j=0;j<8;j++)  {// for each bit
      data_bit = v & 0x01; // isolate least sign bit
      sr_lsb = shift_reg & 0x01;
      fb_bit = (data_bit^sr_lsb) & 0x01; // calculate the feed back bit
      shift_reg = shift_reg >> 1;
      if (fb_bit == 1)
	shift_reg = shift_reg ^ 0x8C;
      v = v >> 1;
    }
  }
  return shift_reg;
}


void DrRobotBoard_t::SendData(unsigned char did,unsigned short dataLength, unsigned char* data)
{
  int dl=0,i=0;
  //unsigned char buffer[dataLength+8];
  unsigned char * buffer=new unsigned char[dataLength+9];
  bzero(buffer,dataLength+9);    
  buffer[dl++]=94;  // STX
  buffer[dl++]=2;   //
  buffer[dl++]=rID; // RID 
  buffer[dl++]=(rID==1?0:seq++);   // Reserved
  buffer[dl++]=did; // DID
  buffer[dl++]=dataLength; //LENGTH
  for (i=0;i<dataLength;i++)// DATA 
    buffer[dl++]=*(data+i);
  buffer[dl++]=CalculateCRC(&buffer[2],buffer[5]+4); // CHECKSUM
  buffer[dl++]=94; // ETX
  buffer[dl++]=13;    
  write(sockfd,buffer,dl);
  delete []buffer;
}

void DrRobotBoard_t::SendPing(void)
{
  int dl=0;
  unsigned char buffer[10];
  bzero(buffer,10);    
  buffer[dl++]=94;  // STX
  buffer[dl++]=2;   //
  buffer[dl++]=rID; // RID 
  buffer[dl++]=(rID==1?0:seq++);   // Reserved
  buffer[dl++]=255; // DID
  buffer[dl++]=1; //LENGTH
  buffer[dl++]=(rID==1?1:0); 
  buffer[dl++]=CalculateCRC(&buffer[2],buffer[5]+4); // CHECKSUM
  buffer[dl++]=94; // ETX
  buffer[dl++]=13;  
  write(sockfd,buffer,dl);
}

void DrRobotBoard_t::SendAck(void)
{
  int dl=0;
  unsigned char buffer[10];
  bzero(buffer,10);    
  buffer[dl++]=94;  // STX
  buffer[dl++]=2;   //
  buffer[dl++]=rID; // RID 
  buffer[dl++]=(rID==1?0:seq++);   // Reserved
  buffer[dl++]=(rID==1?255:1); // DID
  buffer[dl++]=1; //LENGTH
  buffer[dl++]=1;
  buffer[dl++]=CalculateCRC(&buffer[2],buffer[5]+4); // CHECKSUM
  buffer[dl++]=94; // ETX
  buffer[dl++]=13;
  write(sockfd,buffer,dl);
}

bool DrRobotBoard_t::Connect(const char* servername,unsigned int portno)
{
  struct sockaddr_in serv_addr; /* <netinet/in.h> */
  struct hostent *server;
  server = gethostbyname(servername);
  if (server == NULL)
    return false;
  sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // UDP
  if (sockfd < 0) 
    return false;
  //bzero((unsigned char *) &serv_addr, sizeof(serv_addr));
  memset((char *)&serv_addr,0,sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  //bcopy((unsigned char *)server->h_addr, 
  //	(unsigned char *)&serv_addr.sin_addr,
  //	server->h_length);
  memcpy(&serv_addr.sin_addr, server->h_addr, server->h_length);
  serv_addr.sin_port = htons(portno);


  if (connect(sockfd,(const struct sockaddr *)&serv_addr,
	      (socklen_t)sizeof(serv_addr)) < 0) {
    close(sockfd);
    return false;
  }

  int flags;
#define MAXBUF 100
  char buf[MAXBUF];
  int buflen=1;
  flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

  SendPing();
  usleep(100000);
  //if (recv(sockfd,buf,MAXBUF,0)<=0)
  //  return false;
  return true;
}

bool DrRobotBoard_t::Disconnect()
{
  //shutdown(sockfd,SHUT_RDWR);
  return close(sockfd);
}


void DrRobotBoard_t::start()
{
  listener.start();
  active=listener.isActive();   
  pthread_create( &heartBeat, NULL, &heartbeat_function, this);
}

void DrRobotBoard_t::stop()
{
  listener.stop();
  active=listener.isActive();
  pthread_join( heartBeat, NULL);
}
