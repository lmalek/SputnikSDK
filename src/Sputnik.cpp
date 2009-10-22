#include <iostream>
#include <string.h>
#include "SputnikSDK.hh"

#include <allegro.h>


#include <limits>
#include <cmath>
#include <sys/time.h>

#define PI 3.14159265


#define BOK_W_PIX (15.0)
#define BOK_W_CM (20.0)
#define PIX2CM (BOK_W_CM/BOK_W_PIX)
#define CM2PIX (BOK_W_PIX/BOK_W_CM)

#define SENS_SIZE 3

const int Tab_n=500;
double Tab_u[2][Tab_n];

const double r=0.17/2;
const double L=0.268;

inline int signof(double a) { return (a == 0) ? 0 : (a<0 ? -1 : 1); }

void DrawSputnik(BITMAP *sensory, int x, int y,Sputnik_t *sputnik, int neckH, int eyeH){
  static int glowa_w=60*CM2PIX; //cm
  static int glowa_h=3*glowa_w; //cm
  
  static BITMAP *glowa = create_bitmap_ex(32,glowa_w,glowa_h);
  static BITMAP *oko = create_bitmap_ex(32,40,40);
  static int d=BOK_W_PIX;
  static int glowa_x=glowa_w/2;
  static int glowa_y=glowa_h-glowa_x-glowa_w;
  static int oko_x=20;
  static int oko_y=20;
  static float sqrt2=sqrt(2);
  static float sens_space=0.5+sqrt2/2+0.05; // 0.05 przerwa od robota
  static int sens_size=SENS_SIZE; // rozmiar kulki reprezentujacej sensor
  int tmp;
  static int robot_shape[16]={x-0.5*d,y+(0.5+sqrt2/2)*d,
			      x+0.5*d,y+(0.5+sqrt2/2)*d,		
			      x+(0.5+sqrt2/2)*d,y+0.5*d,
			      x+(0.5+sqrt2/2)*d,y-0.5*d,	       
			      x+0.5*d,y-(0.5+sqrt2/2)*d,
			      x-0.5*d,y-(0.5+sqrt2/2)*d,     
			      x-(0.5+sqrt2/2)*d,y-0.5*d,
			      x-(0.5+sqrt2/2)*d,y+0.5*d};
  static int robot_top[26]={x-0.5*d,y+(0.5+sqrt2/2)*d,
			    x+0.5*d,y+(0.5+sqrt2/2)*d,			
			    x+0.5*d,y+(0.25+sqrt2/2)*d,
			    x+(0.5+sqrt2/2)*d,y+0.5*d,			
			    x+(0.5+sqrt2/2)*d,y,
			    x+sqrt2/2*d,y,				
			    x+sqrt2/2*d,y-sqrt2/2*d,
			    x-sqrt2/2*d,y-sqrt2/2*d,			
			    x-sqrt2/2*d,y,
			    x-(0.5+sqrt2/2)*d,y,			
			    x-(0.5+sqrt2/2)*d,y+0.5*d,
			    x-0.5*d,y+(0.25+sqrt2/2)*d, 
			    x-0.5*d,y+(0.5+sqrt2/2)*d};

  clear_to_color(glowa,makecol(255,0,255));
  clear_to_color(oko,makecol(255,0,255));

  //
  // Rysunek sputnika
  //
  // podstawa
  polygon(sensory,8,robot_shape,makecol( 238, 118, 0));
  polygon(sensory,13,robot_top,makecol( 190, 190, 190));
  arc(sensory,x,y,itofix(21),itofix(128-21),
      (sqrt2/2+0.5)*d,makecol( 190, 190, 190));
  triangle(sensory, x,y,x-(sqrt2/2+0.5)*d*0.86,y-(sqrt2/2+0.5)*d*0.5,		
	   x+(sqrt2/2+0.5)*d*0.86,y-(sqrt2/2+0.5)*d*0.5,makecol(190,190,190)); 
  floodfill(sensory, x,y-0.8*(sqrt2/2+0.5)*d,makecol(190,190,190));
  // ekran
  rectfill(sensory, x-0.25*d, y-0.8*d, x+0.25*d, y-0.6*d, makecol(25,25,112)); 
  // swiatelka
  rectfill(sensory, x-0.45*d, y-1.15*d, x-0.35*d, y-1*d, makecol(255,160,0)); 
  rectfill(sensory, x+0.45*d, y-1.15*d, x+0.35*d, y-1*d, makecol(255,160,0)); 
  // czujniki ruchu
  circlefill(sensory, x-sqrt2/2*d, y-0.6*d,0.1*d, makecol(255,255,255)); 
  circlefill(sensory, x+sqrt2/2*d, y-0.6*d,0.1*d, makecol(255,255,255)); 
  
  // OCZY
  circlefill(oko, oko_x, oko_y, 0.2*d, makecol(255,255,255));
  rectfill(oko, oko_x-0.05*d,oko_y-0.28*d,oko_x+0.05*d,oko_y-0.2*d,
	   makecol(65,105,205)); 
  // uwzględnioamy obracanie sie oczu
  rotate_sprite(glowa,oko, glowa_x+0.4*d-oko_x, glowa_y-0.3*d+0.2*d-oko_y,
		ftofix(1.0*eyeH/100*32));
  rotate_sprite(glowa,oko, glowa_x-0.4*d-oko_x, glowa_y-0.3*d+0.2*d-oko_y,
		ftofix(1.0*eyeH/100*32));
  // glowa
  rectfill(glowa, glowa_x-sqrt2/2*d,glowa_y-0.3*d+0.15*d,
	   glowa_x+sqrt2/2*d,glowa_y-0.3*d+0.25*d,makecol(211,211,211)); 
  rectfill(glowa, glowa_x-0.15*d,glowa_y-0.3*d,
	   glowa_x+0.15*d,glowa_y-0.3*d+sqrt2/2*d,makecol(211,211,211)); 
  
  //
  // sensory
  //
  // sonary
  tmp=sputnik->GetSonar(0);
  circlefill(sensory, x-sqrt2/2*(sens_space*d+tmp*CM2PIX),
	     y-sqrt2/2*(sens_space*d+tmp*CM2PIX),sens_size,makecol(30,144,255)); 
  tmp=sputnik->GetSonar(1);
  circlefill(sensory, x,y-sens_space*d-tmp*CM2PIX,sens_size,makecol(30,144,255)); 
  tmp=sputnik->GetSonar(2);
  circlefill(sensory, x+sqrt2/2*(sens_space*d+tmp*CM2PIX),
	     y-sqrt2/2*(sens_space*d+tmp*CM2PIX),sens_size,makecol(30,144,255)); 
  // IR
  tmp=sputnik->GetIR(0);
  circlefill(sensory, x-sqrt2/2*(sens_space*d+tmp*CM2PIX),
	     y-sqrt2/2*(sens_space*d+tmp*CM2PIX),sens_size,makecol(255,0,0));     
  tmp=sputnik->GetIR(1);
  circlefill(sensory, x-0.25*d,y-sens_space*d-tmp*CM2PIX,sens_size,
	     makecol(255,0,0)); 
  tmp=sputnik->GetIR(2);
  circlefill(sensory, x+0.25*d,y-sens_space*d-tmp*CM2PIX,sens_size,
	     makecol(255,0,0)); 
  tmp=sputnik->GetIR(3);
  circlefill(sensory, x+sqrt2/2*(sens_space*d+tmp*CM2PIX),
	     y-sqrt2/2*(sens_space*d+tmp*CM2PIX),sens_size,makecol(255,0,0)); 
  tmp=sputnik->GetIR(4); //prawy bok
  circlefill(sensory, x+sens_space*d+tmp*CM2PIX,y,sens_size,makecol(255,0,0)); 
  tmp=sputnik->GetIR(5); //tyl
  circlefill(sensory, x,y+sens_space*d+tmp*CM2PIX,sens_size,makecol(255,0,0)); 
  tmp=sputnik->GetIR(6); //prawy bok
  circlefill(sensory, x-sens_space*d-tmp*CM2PIX,y,sens_size,makecol(255,0,0)); 
  tmp=sputnik->GetIR(7); //glowa
  circlefill(glowa, glowa_x,glowa_y-0.4*d-tmp*CM2PIX,
	     sens_size,makecol(255,100,0)); 
  
  // uwzględniamy obracanie się głowy
  rotate_sprite(sensory,glowa, x-glowa_x,y-glowa_y+0.3*d,
		ftofix(1.0*neckH/100*64));
}

void set_control(){
  for (int i=0;i<Tab_n;i++){
    Tab_u[0][i]=0.2;//0.2*sin(2.0*PI*i/Tab_n); // liniowa w m/s MAX 1m/s
    Tab_u[1][i]=1*cos(2.0*PI*i/Tab_n);   // obrotu platformy w rad/s
  }
}


double control(int i,timeval t,timeval T){
  double t_d=t.tv_sec+(t.tv_usec/1000000.0);
  double T_d=T.tv_sec+(T.tv_usec/1000000.0);
  int index=(1.0*Tab_n*t_d/T_d);
  if (i<0||i>1) return 0;
  if (index<0 || index>=Tab_n) return 0;
  if (i==0)
    return (Tab_u[0][index]/r-L/2/r*Tab_u[1][index])*600/PI;
  if (i==1)
    return (Tab_u[0][index]/r+L/2/r*Tab_u[1][index])*600/PI;
}


void Fill_Bitmap(BITMAP* bitmap, IplImage* image,int scale=1){
  int nl= scale*image->height;
  int nc= scale*image->width * image->nChannels;
  int step= image->widthStep;
  unsigned char* data=reinterpret_cast<unsigned char*>(image->imageData);
  for(int i=0; i<nl; i++){
    for(int j=0; j<nc; j+= image->nChannels){                 
      for (int k=0; k<scale;k++)
	for (int l=0; l<scale;l++)
	putpixel(bitmap,scale*(j/3)+l,scale*i+k,makecol(data[j],data[j+1],data[j+2]));
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
  res = set_gfx_mode(GFX_AUTODETECT_WINDOWED, 800, 600, 0, 0);
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
  init();

  int P=6, D=0, I=0; // nastawy regulatora PID

  BITMAP *bufor = create_bitmap_ex(32,800,600); 
  BITMAP *plotHM_0 = create_bitmap_ex(32,200,100);
  BITMAP *plotHM_1 = create_bitmap_ex(32,200,100);
  int scale=2;
  BITMAP *camera = create_bitmap_ex(32,176*scale,144*scale);
  BITMAP *sensory = create_bitmap_ex(32,400,560);
  BITMAP *mapa = create_bitmap_ex(32,400,560);
  int robot_x=200;
  int robot_y=360;
  unsigned short int humanM0[200];
  unsigned short int humanM1[200];
  int i;
  unsigned short int v1=0,v2=0;
  char neckV=0;
  char neckH=0;
  char eyeV=0;
  char eyeH=0;
  short enc_0;
  short enc_1;
  long encL_0=0;
  long encL_1=0;
  long encL_0_old=0;
  long encL_1_old=0;

  short max_short=std::numeric_limits<short>::max();

  double x=0,y=0,w=PI/2; 
  double dt; // delta czasu - na potrzeby kinematyki
  double ul, ur; // prędkości kontowe lewego i prawego kola - do kinematyki
  timeval t0,t1; // czasy do obliczania przyrostu czasu na potrzeby liczenia kinematyki
  timeval t,T,DT; // czasy potrzebne do sterowania za pomocą sterowania z funkcji

  double F_x=0, F_y=0; // sily potencjalowe od widocznych przeszkod
  float sin45=sqrt(2)/2;
  float cos45=sin45;
  

  Sputnik_t *sputnik=new Sputnik_t();
  std::string IP("192.168.0.208");
  if (!sputnik->Connect(IP.c_str(),10001,10002)) {
    std::cout<<"ERROR: Conection failure"<<std::endl;
    return -1;
  }
  
  sputnik->Start();
  std::string LCDimage("PGM/lirec.pgm");
  sputnik->DisplayPGM((char*)LCDimage.c_str());

  sputnik->EnableServos();   
  sputnik->ResumeMotors();
   
  sputnik->SetSensorUsage(ENCODER);
  sputnik->SetControlMode(VELOCITY_CONTROL);
  sputnik->SetVelocityControlPID(10,5,10);

   
  set_control();

  for (i=0;i<200;i++){
    humanM0[i]=0;
    humanM1[i]=0;
  }
  clear_to_color(mapa,makecol(255,0,255));

  gettimeofday(&t1,NULL);
  gettimeofday(&t,NULL);
  DT.tv_sec=5;
  DT.tv_usec=0;
  T.tv_sec=t.tv_sec;
  T.tv_usec=t.tv_usec;
  encL_0=max_short-sputnik->GetEncoderPulse(0);
  encL_1=sputnik->GetEncoderPulse(1);
  while (sputnik->isActive()) {
    clear_keybuf();    
    acquire_screen();        

    clear_to_color(bufor, makecol(0,0,0));

    clear_to_color(plotHM_0, makecol(255,255,255));
    clear_to_color(plotHM_1, makecol(255,255,255));
    //clear_to_color(camera, makecol(255,255,255));
    clear_to_color(sensory, makecol(255,255,255));




    /*
     * CZUJNIKI RUCHU
     */
    for (i=0;i<199;i++){
      humanM0[i]=humanM0[i+1];
      humanM1[i]=humanM1[i+1];
    }
    humanM0[199]=sputnik->GetHumanMotion(0);
    humanM0[199]=sputnik->GetIR(0);
    humanM1[199]=sputnik->GetHumanMotion(1);
  
    /*
     *  SILY POTENCJALOWE
     */
    /*F_x=1/(sin45*(1.0*(sputnik->GetSonar(2)-sputnik->GetSonar(0))/255/6+ \
	       1.0*(sputnik->GetIR(3)-sputnik->GetIR(0))/80/3)+			\
	   2.0*(sputnik->GetIR(4)-sputnik->GetIR(6))/80/3+1);
    */
    F_x=pow(1.0*(sputnik->GetIR(3)-sputnik->GetIR(0))/80/3+2.0*(sputnik->GetIR(4)-sputnik->GetIR(6))/80/3+1.0*(sputnik->GetSonar(2)-sputnik->GetSonar(0))/255/6,4);
    F_x=10*signof(F_x)*tan(PI/2*F_x);
    /*
    F_y=1/(cos45*(1.0*(sputnik->GetSonar(0)+sputnik->GetSonar(2))/255/6+	\
	       1.0*(sputnik->GetIR(0)+sputnik->GetIR(3))/80/3)+			\
      1.0*sputnik->GetSonar(0)/255/6+					\
      1.0*(sputnik->GetIR(1)+sputnik->GetIR(2)+sputnik->GetIR(7))/80/3		\
	 -1.0*sputnik->GetIR(5)/80+1);
    */
      
      
      
     
    /*
     *  RECZNE STEROWANIE ROBOTEM
     */    
    if (key[KEY_UP]) {v1+=50; v2+=50;}        
    else if (key[KEY_DOWN]) {v1-=50;v2-=50;}
    else if (key[KEY_RIGHT]) {v1+=10;v2-=10;}
    else if (key[KEY_LEFT]) {v1-=10;v2+=10;}
    else if (key[KEY_SPACE]) {v1=0;v2=0;}


    /*
     *   STEROWANIE ROBOTEM
     */
    gettimeofday(&t,NULL);
    t.tv_sec-=T.tv_sec;
    t.tv_usec-=T.tv_usec;
    //v1=control(0,t,DT);
    //v2=control(1,t,DT);
    sputnik->VelocityCtr(v1,v2);	
    

    /*
     *  RECZNE STEROWANIE GLOWA
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

    /*
     *  STEROWANIE GLOWA
     */
    if (neckH>100) neckH=100;
    if (neckH<-100) neckH=-100;
    if (neckV>100) neckV=100;
    if (neckV<-100) neckV=-100;
    if (eyeH>100) eyeH=100;
    if (eyeH<-100) eyeH=-100;
    if (eyeV>100) eyeV=100;
    if (eyeV<-100) eyeV=-100;
    sputnik->ServoCtr(neckV, neckH,0,eyeV, eyeH,100);   

    
    /*
     *  ENKODERY W PROGRAMIE
     */
    encL_0_old=encL_0;
    encL_1_old=encL_1;

    enc_0=max_short-sputnik->GetEncoderPulse(0);
    enc_1=sputnik->GetEncoderPulse(1);

    if ((encL_0%max_short-enc_0)>(max_short/2))
      encL_0+=max_short;
    if ( (enc_0-encL_0%max_short) > (max_short/2))
    encL_0-=max_short;
    encL_0=(encL_0/max_short)*max_short+enc_0;
    
    if ((encL_1%max_short-enc_1)>(max_short/2))
      encL_1+=max_short;
    if ((enc_1-encL_1%max_short) > (max_short/2))
    encL_1-=max_short;
    encL_1=(encL_1/max_short)*max_short+enc_1;

    /*
     *  ODOMETRIA W OPARCIU O KINEMATYKE
     */
    t0=t1;
    gettimeofday(&t1,NULL);
    dt=(1.0*t1.tv_sec+t1.tv_usec/1000000.0)-(1.0*t0.tv_sec+t0.tv_usec/1000000.0);
    
    ul=1.0*sputnik->GetEncoderDir(0)*sputnik->GetEncoderSpeed(0)/600*PI;
    ur=1.0*sputnik->GetEncoderDir(1)*sputnik->GetEncoderSpeed(1)/600*PI;
    x+=r/2*(ul+ur)*cos(w)*dt;
    y+=r/2*(ul+ur)*sin(w)*dt;
    w+=r/L*(ur-ul)*dt;
    

    /*
     *  WYŚWIETLANIE
     */ 
   textprintf_ex( bufor, font,430, 230,makecol( 255, 255, 255),-1,
	       "V : 0-> %d  1-> %d",
		  v1,v2);
    
    textprintf_ex( bufor, font,430, 240,makecol( 255, 255, 255),-1,
		   "Encoder S: 0-> %d  1-> %d",
		   sputnik->GetEncoderSpeed(0),
		   sputnik->GetEncoderSpeed(1));
    
    textprintf_ex( bufor, font,430, 250,makecol( 255, 255, 255),-1,
		   "P=%d  D=%d Ix100=%d ]",P,D,I);

    textprintf_ex( bufor, font,430, 260,makecol( 255, 255, 255),-1,
		   		   "[ %le  %le %le ]",x,y,w);

    textprintf_ex( bufor, font,430, 270,makecol( 255, 255, 255),-1,
		   		   "[ %le  %le ]",F_x,F_y);

    /*
     *  RYSOWANIE
     */
    Fill_Bitmap(camera,sputnik->getIplImage(),scale); // kamer

    for (i=0;i<200;i++) { // czujniki ruchu
      line(plotHM_0,i,100-humanM0[i],(i+1),100-humanM0[i+1],makecol( 255, 0, 0));
      line(plotHM_1,i,100-humanM1[i],(i+1),100-humanM1[i+1],makecol( 0, 0, 255));
    }

    DrawSputnik(sensory, robot_x, robot_y,sputnik, neckH, eyeH); // sputnik
    
    // trajektoria
    //circlefill(mapa, robot_x+x*BOK_W_PIX/15*100,robot_y-y*BOK_W_PIX/15*100,1,makecol(0,0,0)); 
    circlefill(sensory, robot_x+F_x,robot_y-F_y,3,makecol(0,0,0)); 

    blit( plotHM_0, bufor, 0,0,500,10, 200,100);
    blit( plotHM_1, bufor, 0,0,500,120, 200,100);
    blit( camera, bufor, 0,0,430,288, 176*scale,144*scale);
    blit( sensory, bufor, 0,0,15,15, 400,560);
    masked_blit( mapa, bufor, 0,0,15,15, 400,560);
    blit( bufor, screen, 0,0,0,0, 800,600);
    release_screen();

    /*
     *  WYJSCIE
     */
    rest(10);
    if (key[KEY_ESC]){
      sputnik->Stop();  
      continue;
    }

  }

  destroy_bitmap(plotHM_0); 
  destroy_bitmap(plotHM_1); 
  destroy_bitmap(bufor); 
  destroy_bitmap(camera); 
  destroy_bitmap(sensory); 
  deinit();

  delete sputnik;
  return 0;
}
END_OF_MAIN()


    /*   
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
    */
/*
    if ( sputnik->GetOutVoiceBufferSize()>0){
      sputnik->SaveWAV((char*)WAVOut.c_str());
      sputnik->ClearOutVoiceBuffer();
    }
*/
    /*
    if (key[KEY_M]) {v1=700; v2=700;}         //P=10 D=5 I=10
    else if (key[KEY_N]) {v1=300;v2=300;}
    
    if (key[KEY_U]) {P++;}        
    else if (key[KEY_J]) {P--;}
    else if (key[KEY_I]) {D++;}
    else if (key[KEY_K]) {D--;}
    else if (key[KEY_O]) {I++;}
    else if (key[KEY_L]) {I--;}


    
    ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetVelocityControlPID(0,P,D,I);
    ((DrRobotMotors_t*)sputnik->board[CONTROL]->device[MOTORS])->SetVelocityControlPID(1,P,D,I);
    */
    /*
 textprintf_ex( bufor, font,430, 230,makecol( 255, 255, 255),-1,
	       "Encoder P: 0-> %d  1-> %d [ %ld %ld ]",
	       enc_0,enc_1,encL_0,encL_1);
    */ 

    /*
      textprintf_ex( bufor, font,430, 240,makecol( 255, 255, 255),-1,
		   "Encoder P: 0-> %d  1-> %d",
		   sputnik->GetEncoderPulse(0),
		   sputnik->GetEncoderPulse(1));
    */
    /*    
    textprintf_ex( bufor, font,430, 250,makecol( 255, 255, 255),-1,
		   "Encoder D: 0-> %d  1-> %d",
		   sputnik->GetEncoderDir(0),
		   sputnik->GetEncoderDir(1));
    */
    /*
    textprintf_ex( bufor, font,430, 250,makecol( 255, 255, 255),-1,
		   "[ %lf  %lf %le ]",ul,ur,dt);
		   */
