#include <iostream>
#include <string.h>
#include "SputnikSDK.hh"

#include <allegro.h>

//#include "cv.h"
//#include "highgui.h"



void Fill_Bitmap(BITMAP* bitmap, IplImage* image){
  int nl= image->height;
  int nc= image->width * image->nChannels;
  int step= image->widthStep;
  unsigned char* data=reinterpret_cast<unsigned char*>(image->imageData);
  for(int i=0; i<nl; i++){
    for(int j=0; j<nc; j+= image->nChannels){                 
      putpixel(bitmap,j/3,i,makecol(data[j],data[j+1],data[j+2]));
    }
    data+= step;
  } 
}


void init() {
  int depth, res;
  allegro_init();
  depth = desktop_color_depth();
  if (depth == 0) depth = 32;
  set_color_depth(depth);
  res = set_gfx_mode(GFX_AUTODETECT_WINDOWED, 640, 480, 0, 0);
  if (res != 0) {
    allegro_message(allegro_error);
    exit(-1);
  }

  install_timer();
  install_keyboard();
  install_mouse();
}


void deinit() {
  clear_keybuf();
}


int main(int argc, char *argv[])
{
  Sputnik_t *sputnik=new Sputnik_t();
  std::string IP("192.168.0.208");
  if (!sputnik->Connect(IP.c_str(),10001,10002)) {
      std::cout<<"ERROR: Conection failure"<<std::endl;
      return -1;
  }
  
  sputnik->Start();
  std::string LCDimage("PGM/lirec.pgm");
  sputnik->DisplayPGM((char*)LCDimage.c_str());
  //sputnik->board[CONTROL]->device[MOTORS]->EnableSensorSending();
  //sputnik->board[CONTROL]->device[CUSTOMDATA]->EnableSensorSending();
  //sputnik->board[CONTROL]->device[SENSORS]->EnableSensorSending();  
  //sputnik->board[MEDIA]->device[POWERCONTROLER]->EnableSensorSending();


  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->ServoEnableAll();

  init();
  BITMAP *bufor = create_bitmap_ex(32,640,480); 
  BITMAP *plotHM = create_bitmap_ex(32,200,100);
  BITMAP *camera = create_bitmap_ex(32,176,144);
  unsigned short int humanM0[200];
  unsigned short int humanM1[200];
  int i;
  unsigned short int v1=0,v2=0;
  char neckV=0;
  char neckH=0;
  char eyeV=0;
  char eyeH=0;

  for (i=0;i<200;i++){
    humanM0[i]=0;
    humanM1[i]=0;
  }
  
  //std::string WAVIn("./WAV/happy8k.wav");
  std::string WAVIn("./WAV/play2.wav");
  //std::string WAVOut("./WAV/out.wav");
  sputnik->ReadWAV((char*)WAVIn.c_str());
  //sputnik->StartAudioPlay();
  //sputnik->StartRecord();
  //rest(5000);
  //sputnik->StopRecord();
  //rest(5000);
  //sputnik->SaveWAV((char*)WAVOut.c_str());

   ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->ResumeAll();

  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetPolarity(0,1);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetPolarity(1,1);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetSensorUsage(0,ENCODER);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetSensorUsage(1,ENCODER);
  //((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetControlMode(0,PWM_CONTROL);
  //((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetControlMode(1,PWM_CONTROL);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetControlMode(0,VELOCITY_CONTROL);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetControlMode(1,VELOCITY_CONTROL);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetVelocityControlPID(0,10,5,1000);
  ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetVelocityControlPID(1,10,5,1000);
  //  cvNamedWindow("Sputnik", CV_WINDOW_AUTOSIZE);

  while (sputnik->isActive()) {

    clear_keybuf();    
    acquire_screen();    
    
    for (i=0;i<199;i++){
      humanM0[i]=humanM0[i+1];
      humanM1[i]=humanM1[i+1];
    }
    humanM0[199]=sputnik->GetHumanMotion(0);
    humanM1[199]=sputnik->GetHumanMotion(1);
    //neck=humanM1[199]-humanM0[199];
  
    clear_to_color(bufor, makecol(0,0,0));

    clear_to_color(plotHM, makecol(255,255,255));
    clear_to_color(camera, makecol(255,255,255));

    Fill_Bitmap(camera,sputnik->getIplImage());

    textprintf_ex(bufor, font,400,50,makecol( 255, 255, 255),-1,"Human Motion:");
    textprintf_ex(bufor, font,510,50,makecol( 255, 0, 0),-1,"[0]");
    textprintf_ex(bufor, font,540,50,makecol( 0, 0, 255),-1,"[1]");
    for (i=0;i<200;i++) {
      line(plotHM,i,100-humanM0[i],(i+1),100-humanM0[i+1],makecol( 255, 0, 0));
      line(plotHM,i,100-humanM1[i],(i+1),100-humanM1[i+1],makecol( 0, 0, 255));
    }
    
    textprintf_ex( bufor, font,10, 5,makecol( 255, 255, 255),-1,
		   "Sputnik  [unit] [ [0] | [1] | [2] | [3] | [4] | [5] | [6] | [7] ]");
    textprintf_ex( bufor, font,10, 20,makecol( 255, 255, 255),-1,
		   "IR:      [ cm ] [ %3d | %3d | %3d | %3d | %3d | %3d | %3d | %3d ]",
		   sputnik->GetIR(0), sputnik->GetIR(1), sputnik->GetIR(2),
		   sputnik->GetIR(3), sputnik->GetIR(4), sputnik->GetIR(5),
		   sputnik->GetIR(6), sputnik->GetIR(7));
    textprintf_ex( bufor, font,10, 30,makecol( 255, 255, 255),-1,
		   "Sonar:   [ cm ] [ %3d | %3d | %3d ]", sputnik->GetSonar(0),
		   sputnik->GetSonar(1), sputnik->GetSonar(2));
    textprintf_ex( bufor, font,10, 40,makecol( 255, 255, 255),-1,
		   "Humam:   [  %% ] [ %3d | %3d ]",
		   sputnik->GetHumanMotion(0), sputnik->GetHumanMotion(1));

    textprintf_ex( bufor, font,10, 60,makecol( 255, 255, 255),-1,
		   "Overheat:[  C ] [ %3d | %3d ]",
		   sputnik->GetOverheat(0),
		   sputnik->GetOverheat(1));
    textprintf_ex( bufor, font,10, 70,makecol( 255, 255, 255),-1,
		   "Battery: [  V ] [ %3.1f | %3.1f | %3.1f ]",
		   sputnik->GetBattery(0),
		   sputnik->GetBattery(1),
		   sputnik->GetBattery(2));
    textprintf_ex( bufor, font,10, 80,makecol( 255, 255, 255),-1,
		   "Voltage: [  V ] [ %3.1f | %3.1f ]",
		   sputnik->GetVoltage(0),
		   sputnik->GetVoltage(1));
    textprintf_ex( bufor, font,10, 90,makecol( 255, 255, 255),-1,
		   "Potentio:[  o ] [ %3d | %3d ]",
		   sputnik->GetPotentiometer(0),
		   sputnik->GetPotentiometer(1));

    textprintf_ex( bufor, font,10, 100,makecol( 255, 255, 255),-1,
		   "Current: [  A ] [ %1.3f | %1.3f ]",
		   sputnik->GetCurrent(0),
		   sputnik->GetCurrent(1));

    textprintf_ex( bufor, font,10, 110,makecol( 255, 255, 255),-1,
		   "BatVolt: [    ] [ %d | %d ]",
		   sputnik->GetBatVolt(0),
		   sputnik->GetBatVolt(1));

    textprintf_ex( bufor, font,10, 120,makecol( 255, 255, 255),-1,
		   "Charging: %d  DCIN: %d",
		   sputnik->Charging(),
		   sputnik->GetDCINVolt());
    textprintf_ex( bufor, font,10, 130,makecol( 255, 255, 255),-1,
		   "Powered: DCIN %d Bat1 %d  Bat2 %d",
		   sputnik->PoweredDCIN(),
		   sputnik->PoweredBat1(),
		   sputnik->PoweredBat2());

    textprintf_ex( bufor, font,10, 140,makecol( 255, 255, 255),-1,
		   "Charge Bat1 %d  Bat2 %d",
		   sputnik->ChargeBat1(),
		   sputnik->ChargeBat2());

    textprintf_ex( bufor, font,10, 150,makecol( 255, 255, 255),-1,
		   "Voice %ld",  sputnik->GetOutVoiceBufferSize());

   textprintf_ex( bufor, font,10, 170,makecol( 255, 255, 255),-1,
		   "Encoder P: 0-> %d  1-> %d",
		   sputnik->GetEncoderPulse(0),
		   sputnik->GetEncoderPulse(1));

   textprintf_ex( bufor, font,10, 180,makecol( 255, 255, 255),-1,
		   "Encoder S: 0-> %d  1-> %d",
		   sputnik->GetEncoderSpeed(0),
		   sputnik->GetEncoderSpeed(1));

   textprintf_ex( bufor, font,10, 190,makecol( 255, 255, 255),-1,
		   "Encoder D: 0-> %d  1-> %d",
		   sputnik->GetEncoderDir(0),
		   sputnik->GetEncoderDir(1));
    
    textprintf_ex( bufor, font,10, 200,makecol( 255, 255, 255),-1,
		   "Speed: [ ?? ] [ %6d | %6d ]",
		   v1,v2);
/*
    if ( sputnik->GetOutVoiceBufferSize()>0){
      sputnik->SaveWAV((char*)WAVOut.c_str());
      sputnik->ClearOutVoiceBuffer();
    }
*/
     
    if (key[KEY_UP]) {v1+=50; v2+=50;}        
    else if (key[KEY_DOWN]) {v1-=50;v2-=50;}
    else if (key[KEY_RIGHT]) {v1+=10;v2-=10;}
    else if (key[KEY_LEFT]) {v1-=10;v2+=10;}
    else if (key[KEY_SPACE]) {v1=0;v2=0;}
    
    sputnik->VelocityCtr(v1,v2);	
    
    /*   
    if (key[KEY_UP]) {v1=15000; v2=15000;}        
    else if (key[KEY_DOWN]) {v1=65536-15000;v2=65536-15000;}
    else if (key[KEY_SPACE]) {v1=0;v2=0;}
    sputnik->PwmCtr(65*v1,65*v2);	
    */

    if (key[KEY_A]) neckH-=5;
    if (key[KEY_D]) neckH+=5;
    if (key[KEY_S]) neckV+=5;
    if (key[KEY_W]) neckV-=5;
    if (key[KEY_Q]) eyeH-=5;
    if (key[KEY_E]) eyeH+=5;
    if (key[KEY_1]) eyeV+=5;
    if (key[KEY_3]) eyeV-=5;
    if (key[KEY_2]) {eyeV=0;eyeH=0;neckH=0;neckV=0;}

    sputnik->ServoCtr(neckV, neckH,0,eyeV, eyeH,100);   

    //((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->PwmNonTimeCtrAll(16363-v1*10,16363+v2*10,0,0,0,0);
    //sputnik->MoveNeckVH(0,neck,20);

    
    blit( plotHM, bufor, 0,0,400,70, 200,100);
    blit( camera, bufor, 0,0,400,270, 176,144);
    blit( bufor, screen, 0,0,0,0, 640,480);
    release_screen();

    rest(50);
    if (key[KEY_ESC]){
      sputnik->Stop();  
      continue;
    }

    //cvShowImage("Sputnik", sputnik->getIplImage() );

    //if( (cvWaitKey(10) & 255) == 27 ) 
    //sputnik->Stop();
  }

  destroy_bitmap(plotHM); 
  destroy_bitmap(bufor); 
  destroy_bitmap(camera); 
  deinit();

  delete sputnik;
  return 0;
}
END_OF_MAIN()
