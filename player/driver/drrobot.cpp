/** @ingroup drivers 
 * @{ 
 * @defgroup driver_drrobot drrobot
 * @brief drrobot mobile robots
 *
 * drrobot robot from DrRobot
 * @par Compile-time dependencies
 * - none
 * @par Provides
 * The drrobot driver provides the following device interfaces, some of
 * them named:
 * - "odometry" @ref interface_position2d
 * - This interface returns odometry data, and accepts velocity commands.
 * - @ref interface_power
 * - Returns the current battery voltage (12 V when fully charged).
 * - @ref interface_sonar
 * - Returns data from sonar arrays 
 * - @ref interface_ir
 * - Returns data from IR range sensor
 *
 * @par Supported configuration requests
 *
 * - "odometry" @ref interface_position2d :
 * - PLAYER_POSITION_SET_ODOM_REQ
 * - PLAYER_POSITION_MOTOR_POWER_REQ
 * - PLAYER_POSITION_RESET_ODOM_REQ
 * - PLAYER_POSITION_GET_GEOM_REQ
 * - PLAYER_POSITION_VELOCITY_MODE_REQ
 * - @ref interface_sonar :
 * - PLAYER_SONAR_POWER_REQ
 * - PLAYER_SONAR_GET_GEOM_REQ
 * - @ref interface_ir :
 * - PLAYER_IR_POWER_REQ
 * - PLAYER_IR_POSE_REQ
 *
 * @par Configuration file options
 * 
 * - port (string)
 * - Default: "/dev/ttyS0"
 * - use_tcp (boolean)
 * - Defaut: 0
 * - Set to 1 if a TCP connection should be used instead of serial port (e.g. Amigobot
 *   with ethernet-serial bridge device attached)
 * - tcp_remote_host (string)
 * - Default: "localhost"
 * - Remote hostname or IP address to connect to if using TCP
 * - tcp_remote_port_A (integer)
 * - Default: 10001
 * - Remote port to connect to if using TCP
 * - Serial port used to communicate with the robot.
 * - tcp_remote_port_B (integer)
 * - Default: 10002
 * - Remote port to connect to if using TCP
 * - Serial port used to communicate with the robot
 * - max_xspeed (length)
 * - Default: 0.5 m/s
 * - Maximum translational velocity
 * - max_yawspeed (angle)
 * - Default: 100 deg/s
 * - Maximum rotational velocity
 * - max_xaccel (length)
 * - Default: 0
 * - Maximum translational acceleration, in length/sec/sec; nonnegative.
 *    Zero means use the robot's default value.
 * - max_xdecel (length)
 * - Default: 0
 * - Maximum translational deceleration, in length/sec/sec; nonpositive.
 *   Zero means use the robot's default value.
 * - max_yawaccel (angle)
 * - Default: 0
 * - Maximum rotational acceleration, in angle/sec/sec; nonnegative.
 *   Zero means use the robot's default value.
 * - pos_kp (integer)
 * - Default: -1
 * - Positional PID setting; proportional gain.
 *   Negative means use the robot's default value.
 * - pos_kd (integer)
 * - Default: -1
 * - Positional PID setting; derivative gain.
 *   Negative means use the robot's default value.
 * - pos_ki (integer)
 * - Default: -1
 * - Positional PID setting; integral gain.
 *   Negative means use the robot's default value.
 * - vel_kp (integer)
 * - Default: -1
 * - Velocitional PID setting; proportional gain.
 *   Negative means use the robot's default value.
 * - vel_kd (integer)
 * - Default: -1
 * - Velocitional PID setting; derivative gain.
 *   Negative means use the robot's default value.
 * - vel_ki (integer)
 * - Default: -1
 * - Velocitional PID setting; integral gain.
 *   Negative means use the robot's default value.
 * - max_yawdecel (angle)
 * - Default: 0
 * - Maximum rotational deceleration, in angle/sec/sec; nonpositive.
 *   Zero means use the robot's default value.
 *
 * @par Example
 *
 * @verbatim
 * driver
 * (
 *   name "drrobot"
 *   provides ["odometry::position:0" "ir:0" "sonar:0" "power:0"]
 * )
 * @endverbatim
 *
 * @author Lukasz Malek
 * @} 
 */

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>  /* for abs() */
#include <netinet/in.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>

#include "drrobot.h"

#define RAD2SPUTNIK(X) ((int)((100.0*(X)/M_PI*4)))

////////////////////////////////////////////////////////////////////////////////

Driver* DRROBOT_Init(ConfigFile* cf, int section) {
  return (Driver*)(new DRROBOT(cf,section));
}

void DRROBOT_Register(DriverTable* table) {
  char name[10]="drrobot";
  table->AddDriver(name, DRROBOT_Init);
}


extern "C" {
  int player_driver_init(DriverTable* table) {
    puts("Sputnik driver initializing.");
    DRROBOT_Register(table);
    puts("Sputnik driver done.");
    return 0;
  }
}

////////////////////////////////////////////////////////////////////////////////

DRROBOT::DRROBOT(ConfigFile* cf, int section)
        : Driver(cf,section,true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
  // zero ids, so that we'll know later which interfaces were requested
  memset(&this->position_id, 0, sizeof(player_devaddr_t));
  memset(&this->sonar_id, 0, sizeof(player_devaddr_t));
  memset(&this->ir_id, 0, sizeof(player_devaddr_t));
  memset(&this->power_id, 0, sizeof(player_devaddr_t));
  memset(&this->actarray_id, 0, sizeof(player_devaddr_t));

  // set by default no subscriptions to the devices
  this->position_subscriptions = this->sonar_subscriptions = 0;
  this->ir_subscriptions = this->actarray_subscriptions = 0;

  this->pulse = -1;

  // Do we create a robot position interface?
  if(cf->ReadDeviceAddr(&(this->position_id), section, "provides",
                        PLAYER_POSITION2D_CODE, -1, NULL) == 0) {
    if(this->AddInterface(this->position_id) != 0) { 
      this->SetError(-1); return; 
    }
  }

  // Do we create a sonar interface?
  if(cf->ReadDeviceAddr(&(this->sonar_id), section, "provides",
                      PLAYER_SONAR_CODE, -1, NULL) == 0) {
    if(this->AddInterface(this->sonar_id) != 0) {
      this->SetError(-1); return;
    }
  }

  // Do we create a ir interface?
  if(cf->ReadDeviceAddr(&(this->ir_id), section, "provides",
                      PLAYER_IR_CODE, -1, NULL) == 0) {
    if(this->AddInterface(this->ir_id) != 0) {
      this->SetError(-1); return;
    }
  }

  // Do we create a power interface?
  if(cf->ReadDeviceAddr(&(this->power_id), section, "provides",
                      PLAYER_POWER_CODE, -1, NULL) == 0) {
    if(this->AddInterface(this->power_id) != 0) {
      this->SetError(-1); return;
    }
  }

  // Do we create a limb interface?
  if(cf->ReadDeviceAddr(&(this->actarray_id), section, "provides",
                      PLAYER_ACTARRAY_CODE, -1, NULL) == 0) {
    if(this->AddInterface(this->actarray_id) != 0) {
      this->SetError(-1); return;
    }
    // Stop actarray messages in the queue from being overwritten
    this->InQueue->AddReplaceRule (this->actarray_id, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_POS_CMD, false);
    this->InQueue->AddReplaceRule (this->actarray_id, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_SPEED_CMD, false);
    this->InQueue->AddReplaceRule (this->actarray_id, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_HOME_CMD, false);
  }

  // build the table of robot parameters.
  //::initialize_robot_params();

  // Read config file options
  this->pulse = cf->ReadFloat(section,"pulse",-1);
  this->pos_kp = cf->ReadInt(section, "pos_kp", -1);
  this->pos_kd = cf->ReadInt(section, "pos_kd", -1);
  this->pos_ki = cf->ReadInt(section, "pos_ki", -1);
  this->vel_kp = cf->ReadInt(section, "vel_kp", -1);
  this->vel_kd = cf->ReadInt(section, "vel_kd", -1);
  this->vel_ki = cf->ReadInt(section, "vel_ki", -1);

  this->psos_serial_port = cf->ReadString(section,"port",DEFAULT_DRROBOT_PORT);
  //this->psos_use_tcp = cf->ReadBool(section, "use_tcp", false); 
  // TODO after ReadBool added
  this->psos_use_tcp = cf->ReadInt(section, "use_tcp", 0);
  this->psos_tcp_host = cf->ReadString(section, "tcp_remote_host", 
				       DEFAULT_DRROBOT_TCP_REMOTE_HOST);
  this->psos_tcp_port_A = cf->ReadInt(section, "tcp_remote_port_A", 
				      DEFAULT_DRROBOT_TCP_REMOTE_PORT_A);
  this->psos_tcp_port_B = cf->ReadInt(section, "tcp_remote_port_B", 
				      DEFAULT_DRROBOT_TCP_REMOTE_PORT_B);
  this->psos_fd = -1;

}

////////////////////////////////////////////////////////////////////////////////

int DRROBOT::Setup()
{
  int i;
  int baud=B115200;   // we conect at B115200
  int currbaud = 0;

  struct termios term;
  unsigned char command;
  int flags;
  bool sent_close = false;
  enum
  {
    NO_SYNC,
    READY
  } psos_state;

  psos_state = NO_SYNC;

  int cnt;

  sputnik=new Sputnik_K_t();
  

  if(this->psos_use_tcp) {
    if (!sputnik->Connect(this->psos_tcp_host,10001,10002)) {
      perror("ERROR: Conection failure\n");
      return -1;
    } 
    else {
      perror("Conected!!!\n");
      psos_state != READY;
    }
  }
  else { 
    // Serial port: -- not implemented yet
  } // end TCP socket or serial port.


  // robot Sputnik configuration, TODO: make it changable outside the code
  robotParams.AngleConvFactor=0.001534;
  robotParams.DiffConvFactor=0.0056; //
  robotParams.IRNum=7; //
  robotParams.RobotLength=0.405; //
  robotParams.RobotWidth=0.405; //
  robotParams.SonarNum=3; //
  robotParams.SwitchToBaudRate=115200; //
  robotParams.Vel2Divisor=20; //
  // sonars
  robotParams.sonar_pose[0].x=0.130;
  robotParams.sonar_pose[0].y=0.140;
  robotParams.sonar_pose[0].th=45;
  robotParams.sonar_pose[1].x=0.190;
  robotParams.sonar_pose[1].y=0.0;
  robotParams.sonar_pose[1].th=0;
  robotParams.sonar_pose[2].x=0.130;
  robotParams.sonar_pose[2].y=-0.140;
  robotParams.sonar_pose[2].th=-45;
  // IR
  robotParams.ir_pose[0].x=0.160; robotParams.ir_pose[0].y=0.100; robotParams.ir_pose[0].th=45;
  robotParams.ir_pose[1].x=0.190; robotParams.ir_pose[1].y=0.040;  robotParams.ir_pose[1].th=15;
  robotParams.ir_pose[2].x=0.190; robotParams.ir_pose[2].y=-0.040; robotParams.ir_pose[2].th=-15;
  robotParams.ir_pose[3].x=0.160; robotParams.ir_pose[3].y=-0.100;robotParams.ir_pose[3].th=-45;
  robotParams.ir_pose[4].x=0.0;   robotParams.ir_pose[4].y=-0.120;robotParams.ir_pose[4].th=-90;
  robotParams.ir_pose[5].x=-0.190;robotParams.ir_pose[5].y=0.0;   robotParams.ir_pose[5].th=180;
  robotParams.ir_pose[6].x=0.0;   robotParams.ir_pose[6].y=0.120; robotParams.ir_pose[6].th=90;
  robotParams.ir_pose[7].x=0.0;   robotParams.ir_pose[7].y=0.0;   robotParams.ir_pose[7].th=0;
  // head servos
  robotParams.head.actuators_count=6;
  drrobot_data.head.actuators_count=6; // this data is send when asked
				       // for the actuators number
  for (int i=0;i<robotParams.head.actuators_count;i++) {
    robotParams.head.actuators[i].type = PLAYER_ACTARRAY_TYPE_ROTARY; 
    robotParams.head.actuators[i].hasbrakes = 0; 
    robotParams.head.actuators[i].config_speed = M_PI/10;  // [rad/s]
    robotParams.head.actuators[i].home = 0; 
    //
    drrobot_data.head.actuators[i].position=0;
  }
  // neckV - pion
  robotParams.head.actuators[0].min = -M_PI/6; 
  robotParams.head.actuators[0].max = M_PI/6;
  // neckH - poziom
  robotParams.head.actuators[1].min = -M_PI/4.5; 
  robotParams.head.actuators[1].max = M_PI/4.5;
  // mouth
  robotParams.head.actuators[2].min = 0; 
  robotParams.head.actuators[2].max = M_PI/6;
  // empty
  robotParams.head.actuators[3].min = 0; 
  robotParams.head.actuators[3].max = 0;
  // eyeV
  robotParams.head.actuators[4].min = -M_PI/5.5; 
  robotParams.head.actuators[4].max = M_PI/5.5;
  // eyeH
  robotParams.head.actuators[5].min = -M_PI/5.5; 
  robotParams.head.actuators[5].max = M_PI/5.5;

  sputnik->Start(); 
  SendPulse();
  
  /* now spawn reading thread */
  this->StartThread();
  return(0);
}

int DRROBOT::Shutdown()
{
  if(this->psos_fd == -1)
    return(0);

  sputnik->Stop();
  this->StopThread();
  delete sputnik;
  puts("DRROBOT has been shutdown");
  return(0);
}

DRROBOT::~DRROBOT (void) {
}

int DRROBOT::Subscribe(player_devaddr_t id) {
  int setupResult;
  
  // do the subscription
  if((setupResult = Driver::Subscribe(id)) == 0)  {
    // also increment the appropriate subscription counter
    if(Device::MatchDeviceAddress(id, this->position_id))
      this->position_subscriptions++;
    else if(Device::MatchDeviceAddress(id, this->sonar_id))
      this->sonar_subscriptions++;
    else if(Device::MatchDeviceAddress(id, this->ir_id))
      this->ir_subscriptions++;
    else if(Device::MatchDeviceAddress(id, this->actarray_id))
      this->actarray_subscriptions++;
  }
  return(setupResult);
}

int DRROBOT::Unsubscribe(player_devaddr_t id) {
  int shutdownResult;
  
  // do the unsubscription
  if((shutdownResult = Driver::Unsubscribe(id)) == 0) {
    // also decrement the appropriate subscription counter
    if(Device::MatchDeviceAddress(id, this->position_id)) {
      this->position_subscriptions--;
      assert(this->position_subscriptions >= 0);
    }
    else if(Device::MatchDeviceAddress(id, this->sonar_id)) {
      this->sonar_subscriptions--;
      assert(this->sonar_subscriptions >= 0);
    }
    else if(Device::MatchDeviceAddress(id, this->ir_id)) {
      this->ir_subscriptions--;
      assert(this->ir_subscriptions >= 0);
    }
    else if(Device::MatchDeviceAddress(id, this->actarray_id)) {
      this->actarray_subscriptions--;
      assert(this->actarray_subscriptions >= 0);
    }
  }
  return(shutdownResult);
}

////////////////////////////////////////////////////////////////////////////////

void DRROBOT::PutData(void) {
  // TODO: something smarter about timestamping, now is NULL

  // put odometry data
  this->Publish(this->position_id, NULL, 
		PLAYER_MSGTYPE_DATA,
                PLAYER_POSITION2D_DATA_STATE,
                (void*)&(this->drrobot_data.position),
                sizeof(player_position2d_data_t),
                NULL);

  // put sonar data
  this->Publish(this->sonar_id, NULL,
                PLAYER_MSGTYPE_DATA,
                PLAYER_SONAR_DATA_RANGES,
                (void*)&(this->drrobot_data.sonar),
                sizeof(player_sonar_data_t),
                NULL);

  // put ir data
  this->Publish(this->ir_id, NULL,
                PLAYER_MSGTYPE_DATA,
                PLAYER_IR_DATA_RANGES,
                (void*)&(this->drrobot_data.ir),
                sizeof(player_ir_data_t),
                NULL);

  // put power data
  this->Publish(this->power_id, NULL,
                PLAYER_MSGTYPE_DATA,
                PLAYER_POWER_DATA_STATE,
                (void*)&(this->drrobot_data.power),
                sizeof(player_power_data_t),
                NULL);


  // put actarray data
  this->Publish(this->actarray_id, NULL,
                PLAYER_MSGTYPE_DATA,
                PLAYER_ACTARRAY_DATA_STATE,
                (void*)&(this->drrobot_data.head),
                sizeof(player_actarray_data_t),
                NULL);
}

void DRROBOT::Main() {
  int last_sonar_subscrcount=0;
  int last_ir_subscrcount=0;
  int last_position_subscrcount=0;
  double currentTime;
  struct timeval timeVal;

  for(;;) {
    pthread_testcancel();
    // handle pending messages
    ProcessMessages();
    // Check if need to send a pulse to the robot
    if (this->pulse != -1) {
      gettimeofday (&timeVal, NULL);
      currentTime = static_cast<double> (timeVal.tv_sec) + (static_cast<double> (timeVal.tv_usec) / 1e6);
      if ((currentTime - lastPulseTime) > this->pulse) {
        SendPulse ();
        // Update the time of last pulse/command
        lastPulseTime = currentTime;
      }
    }
    RefreshData();
    PutData();
    usleep(DRROBOT_CYCLETIME_USEC);
  }
}

void DRROBOT::RefreshData() {
    /*
     *  ENCODERS
     */
    sputnik->encL_0_old = sputnik->encL_0; 
    sputnik->encL_1_old = sputnik->encL_1;

    sputnik->enc_0 = max_short-sputnik->GetEncoderPulse(0);
    sputnik->enc_1 = sputnik->GetEncoderPulse(1);

    // left encoder total value
    if ( (sputnik->encL_0 % max_short-sputnik->enc_0) > (max_short/2) )
      sputnik->encL_0 += max_short;
    if ( (sputnik->enc_0-sputnik->encL_0 % max_short) > (max_short/2) )
      sputnik->encL_0 -= max_short;
    sputnik->encL_0 = (sputnik->encL_0/max_short) * max_short + sputnik->enc_0;
    
    // right encoder total value
    if ( (sputnik->encL_1 % max_short-sputnik->enc_1 ) > (max_short/2) )
      sputnik->encL_1 += max_short;
    if ( (sputnik->enc_1-sputnik->encL_1 % max_short) > (max_short/2) )
      sputnik->encL_1 -= max_short;
    sputnik->encL_1 = (sputnik->encL_1/max_short) * max_short + sputnik->enc_1;

    /*
     *  ODOMETRIA W OPARCIU O KINEMATYKE
     */
    sputnik->t0 = sputnik->t1;
    gettimeofday(&sputnik->t1,NULL);
    sputnik->dt = (1.0*sputnik->t1.tv_sec+sputnik->t1.tv_usec/1000000.0)-
      (1.0*sputnik->t0.tv_sec+sputnik->t0.tv_usec/1000000.0);
    
    sputnik->ul = 1.0*sputnik->GetEncoderDir(0)*sputnik->GetEncoderSpeed(0)/600*PI;
    sputnik->ur = 1.0*sputnik->GetEncoderDir(1)*sputnik->GetEncoderSpeed(1)/600*PI;
    sputnik->x += robot_R/2*(sputnik->ul+sputnik->ur)*cos(sputnik->w)*sputnik->dt;
    sputnik->y += robot_R/2*(sputnik->ul+sputnik->ur)*sin(sputnik->w)*sputnik->dt;
    sputnik->w += robot_R/robot_L*(sputnik->ur-sputnik->ul)*sputnik->dt;

    //TODO: tu wpisac przekopiowanie danych ze struktury sputnika do strukury playera
    //TODO: nalezalo by to przerzucic do odrebnej funkcji
    this->drrobot_data.sonar.ranges_count = 3;
    this->drrobot_data.sonar.ranges[0] = 0.01*sputnik->GetSonar(0); // [cm]->[m]							       
    this->drrobot_data.sonar.ranges[1] = 0.01*sputnik->GetSonar(1); 
    this->drrobot_data.sonar.ranges[2] = 0.01*sputnik->GetSonar(2);

    this->drrobot_data.ir.ranges_count = 8;
    this->drrobot_data.ir.ranges[0] = 0.01*sputnik->GetIR(0);
    this->drrobot_data.ir.ranges[1] = 0.01*sputnik->GetIR(1); 
    this->drrobot_data.ir.ranges[2] = 0.01*sputnik->GetIR(2);
    this->drrobot_data.ir.ranges[3] = 0.01*sputnik->GetIR(3);
    this->drrobot_data.ir.ranges[4] = 0.01*sputnik->GetIR(4); 
    this->drrobot_data.ir.ranges[5] = 0.01*sputnik->GetIR(5);
    this->drrobot_data.ir.ranges[6] = 0.01*sputnik->GetIR(6);
    this->drrobot_data.ir.ranges[7] = 0.01*sputnik->GetIR(7);

    this->drrobot_data.position.pos.px = sputnik->x;
    this->drrobot_data.position.pos.py = sputnik->y;
    this->drrobot_data.position.pos.pa = sputnik->w;

    this->drrobot_data.position.vel.px = robot_R/2 * (sputnik->ul + sputnik->ur) * cos(sputnik->w);
    this->drrobot_data.position.vel.py = robot_R/2 * (sputnik->ul + sputnik->ur) * sin(sputnik->w);
    this->drrobot_data.position.vel.pa = robot_R/robot_L * (sputnik->ur - sputnik->ul);

    this->drrobot_data.position.stall = 0;
}

void DRROBOT::ResetRawPositions() {
  sputnik->x=0; sputnik->y=0; sputnik->w=0;
}

/* toggle motors on/off, according to val */
void DRROBOT::ToggleMotorPower(unsigned char val) { 
  if (val>0) 
    sputnik->ResumeMotors();
  else
    sputnik->SuspendMotors(); 
}

/* toggle actarray servos on/off, according to val */
void DRROBOT::ToggleActarrayPower(unsigned char val) { 
  if (val>0) 
    sputnik->EnableServos();
  else
    sputnik->DisableServos(); 
}


////////////////////////////////////////////////////////////////////////////////

int DRROBOT::ProcessMessage(MessageQueue * resp_queue,
			    player_msghdr * hdr,
			    void * data) {
  // Look for configuration requests
  if(hdr->type == PLAYER_MSGTYPE_REQ)
    return(this->HandleConfig(resp_queue,hdr,data));
  else if(hdr->type == PLAYER_MSGTYPE_CMD)
    return(this->HandleCommand(hdr,data));
  else
    return(-1);
}


int DRROBOT::HandleConfig(MessageQueue* resp_queue,
			  player_msghdr * hdr,
			  void * data)
{
  int joint = 0;
  double newSpeed = 0.0f;

  //
  // -------------------- POSITION2D -----------------------------------
  //
  // check for position config requests
  if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                           PLAYER_POSITION2D_REQ_SET_ODOM,
                           this->position_id))
  {
    if(hdr->size != sizeof(player_position2d_set_odom_req_t))
    {
      PLAYER_WARN("Arg to odometry set requests wrong size; ignoring");
      return(-1);
    }
    player_position2d_set_odom_req_t* set_odom_req =
      (player_position2d_set_odom_req_t*)data;
    this->Publish(this->position_id, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_SET_ODOM);
    return(0);
  }
  // PLAYER_POSITION2D_REQ_MOTOR_POWER
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                this->position_id)) {
    /* motor state change request
     *   1 = enable motors
     *   0 = disable motors (default)
     */
    if(hdr->size != sizeof(player_position2d_power_config_t)) {
      PLAYER_WARN("Arg to motor state change request wrong size; ignoring");
      return(-1);
    }
    player_position2d_power_config_t* power_config =
      (player_position2d_power_config_t*)data;
    this->ToggleMotorPower(power_config->state);
    this->Publish(this->position_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_POSITION2D_REQ_MOTOR_POWER);
    return(0);
  } 
  // PLAYER_POSITION2D_REQ_RESET_ODOM
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_RESET_ODOM,
                                this->position_id)) {
    /* reset position to 0,0,0: no args */
    if(hdr->size != 0) {
      PLAYER_WARN("Arg to reset position request is wrong size; ignoring");
      return(-1);
    }
    ResetRawPositions();
    
    this->Publish(this->position_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_POSITION2D_REQ_RESET_ODOM);
    return(0);
  }
  // PLAYER_POSITION2D_REQ_GET_GEOM
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_GET_GEOM,
                                this->position_id)) {
    /* Return the robot geometry. */
    if(hdr->size != 0) {
      PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
      return(-1);
    }
    player_position2d_geom_t geom;
    // TODO: why we send pose == 0?
    geom.pose.px = 0.0;
    geom.pose.py = 0.0;
    geom.pose.pa = 0.0;
    // get dimensions from the parameter table
    geom.size.sl = robotParams.RobotLength;
    geom.size.sw = robotParams.RobotWidth;

    this->Publish(this->position_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_POSITION2D_REQ_GET_GEOM,
                  (void*)&geom, sizeof(geom), NULL);
    return(0);
  }
  // PLAYER_POSITION2D_REQ_VELOCITY_MODE
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_VELOCITY_MODE,
                                this->position_id)) {
    /* velocity control mode:
     *   0 = direct wheel velocity control (default)
     *   1 = separate translational and rotational control
     */
    if(hdr->size != sizeof(player_position2d_velocity_mode_config_t)) {
      PLAYER_WARN("Arg to velocity control mode change request is wrong "
                  "size; ignoring");
      return(-1);
    }
    player_position2d_velocity_mode_config_t* velmode_config =
            (player_position2d_velocity_mode_config_t*)data;
    
    this->Publish(this->position_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_POSITION2D_REQ_VELOCITY_MODE);
    return(0);
  }
  //
  // -------------------- SONAR -----------------------------------
  //
  // PLAYER_SONAR_REQ_POWER
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_SONAR_REQ_POWER,
                                this->sonar_id)) {
    this->Publish(this->sonar_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_SONAR_REQ_POWER);
    return(0);
  }
  // PLAYER_SONAR_REQ_GET_GEOM
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_SONAR_REQ_GET_GEOM,
                                this->sonar_id)) {
    /* Return the sonar geometry. */
    if(hdr->size != 0) {
      PLAYER_WARN("Arg get sonar geom is wrong size; ignoring");
      return(-1);
    }
    player_sonar_geom_t geom;
    geom.poses_count = robotParams.SonarNum;
    for(int i = 0; i < robotParams.SonarNum; i++) {
      sonar_pose_t pose = robotParams.sonar_pose[i];
      geom.poses[i].px = pose.x;
      geom.poses[i].py = pose.y;
      geom.poses[i].pa = DTOR(pose.th);
    }

    this->Publish(this->sonar_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
		  PLAYER_SONAR_REQ_GET_GEOM,
                  (void*)&geom, sizeof(geom), NULL);
    return(0);
  }
  //
  // -------------------- IR -----------------------------------
  //
  // PLAYER_IR_POWER
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_IR_POWER,
                                this->ir_id)) {
    this->Publish(this->ir_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_IR_POWER);
    return(0);
  }
  // PLAYER_IR_POSE
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_IR_POSE,
                                this->ir_id)) {
    /* Return the sonar geometry. */
    if(hdr->size != 0) {
      PLAYER_WARN("Arg get ir geom is wrong size; ignoring");
      return(-1);
    }
    player_ir_pose_t geom;
    geom.poses_count = robotParams.IRNum;
    for(int i = 0; i < robotParams.IRNum; i++) {
      ir_pose_t pose = robotParams.ir_pose[i];
      geom.poses[i].px = pose.x;
      geom.poses[i].py = pose.y;
      geom.poses[i].pa = DTOR(pose.th);
    }
    this->Publish(this->ir_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_IR_POSE,
                  (void*)&geom, sizeof(geom), NULL);
    return(0);
  }
  // ----------------------------------------------------------------------
  //                   ACTARRAY
  // ----------------------------------------------------------------------
  // 
  // PLAYER_ACTARRAY_POWER_REQ 
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_POWER_REQ, 
                                this->actarray_id)) {
    /* request for the actarray power state */
    /* motor state change request
     *   1 = enable motors
     *   0 = disable motors (default)
     */
    if(hdr->size != sizeof(player_actarray_power_config_t)) {
      PLAYER_WARN("Arg to actarray state change request wrong size; ignoring");
      return(-1);
    }
    player_actarray_power_config_t* power_config =
      (player_actarray_power_config_t*)data;
    this->ToggleActarrayPower(power_config->value);
    this->Publish(this->actarray_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_ACTARRAY_POWER_REQ);
    return(0);
  }
  // PLAYER_ACTARRAY_BRAKES_REQ  -- nie obsługiwane
  //else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_BRAKES_REQ
  //                              this->actarray_id)) {
  //  /* request for the actarray breaks state */
  //  return(0);
  //}
  // PLAYER_ACTARRAY_GET_GEOM_REQ
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_GET_GEOM_REQ,
                                this->actarray_id)) {
    /* request for the actarray geometry state */
    /* Return the robot geometry. */
    /*    if(hdr->size != 0) {
      PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
      return(-1);
    }
    */
    //printf("PLAYER_ACTARRAY_GET_GEOM_REQ\n");
    this->Publish(this->actarray_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_ACTARRAY_GET_GEOM_REQ, 
		  &robotParams.head, sizeof (player_actarray_geom_t), NULL);
    return(0);
  }
  // PLAYER_ACTARRAY_SPEED_REQ
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_SPEED_REQ,
                                this->actarray_id)) {
    /* request for the actarray speed state */
    if(hdr->size != sizeof(player_actarray_speed_config_t)) {
      PLAYER_WARN("Arg to actarray state change request wrong size; ignoring");
      return(-1);
    }
    player_actarray_speed_config_t* speed_config =
      (player_actarray_speed_config_t*)data;
    robotParams.head.actuators[speed_config->joint].config_speed = speed_config->speed;
    this->Publish(this->actarray_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, 
		  PLAYER_ACTARRAY_SPEED_REQ);
    return(0);
  }
  // ----------------------------------------------------------------------
  // 
  // ======================================================================
  //
  else {
    PLAYER_WARN("unknown config request to drrobot driver");
    return(-1);
  }
}

void DRROBOT::SendPulse (void) {
  sputnik->board[CONTROL]->SendPing();
  sputnik->board[MEDIA]->SendPing();
}

int DRROBOT::HandleCommand(player_msghdr * hdr, void* data) {
  int retVal = -1;
  struct timeval timeVal;

  if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
			   this->position_id)) {
    // get and send the latest motor command
    player_position2d_cmd_vel_t position_cmd;
    position_cmd = *(player_position2d_cmd_vel_t*)data;
    this->HandlePositionCommand(position_cmd);
    retVal = 0;
  }
  // Actarray - head
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_CMD,
                           PLAYER_ACTARRAY_HOME_CMD,
			   this->actarray_id)) {
    //printf("ACTARRAY_HOME_CMD\n");
    // get and send the latest motor command
    player_actarray_home_cmd_t actarray_cmd;
    actarray_cmd = *(player_actarray_home_cmd_t*)data;
    int time = 0, time_max=0;
    if (actarray_cmd.joint == -1){
      for (int i=0;i<robotParams.head.actuators_count;i++){
	time = abs(drrobot_data.head.actuators[i].position/robotParams.head.actuators[i].config_speed*1000);
	if (time_max<time)
	  time_max=time;
      }
      sputnik->ServoCtr(RAD2SPUTNIK(robotParams.head.actuators[0].home),
			RAD2SPUTNIK(robotParams.head.actuators[1].home),
			RAD2SPUTNIK(robotParams.head.actuators[2].home),
			RAD2SPUTNIK(robotParams.head.actuators[4].home),
			RAD2SPUTNIK(robotParams.head.actuators[5].home),time_max);
      for (int i=0;i<robotParams.head.actuators_count;i++)
	drrobot_data.head.actuators[i].position=robotParams.head.actuators[i].home;
    }
    else {
      sputnik->ServoCtrSingle(actarray_cmd.joint,
			      RAD2SPUTNIK(robotParams.head.actuators[actarray_cmd.joint].home),
			      abs(drrobot_data.head.actuators[actarray_cmd.joint].position/
				  robotParams.head.actuators[actarray_cmd.joint].config_speed*1000));
      drrobot_data.head.actuators[actarray_cmd.joint].position =
	robotParams.head.actuators[actarray_cmd.joint].home;
    }
    retVal = 0;
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_CMD,
                           PLAYER_ACTARRAY_POS_CMD,
			   this->actarray_id)) {
    //printf("ACTARRAY_POS_CMD\n");
    // get and send the latest motor command
    player_actarray_position_cmd_t actarray_cmd;
    actarray_cmd = *(player_actarray_position_cmd_t*)data;
    
    float offset = actarray_cmd.position - drrobot_data.head.actuators[actarray_cmd.joint].position;
    int moveTime = abs(offset/robotParams.head.actuators[actarray_cmd.joint].config_speed*1000); // in [ms]

    if (actarray_cmd.position < robotParams.head.actuators[actarray_cmd.joint].min)
      actarray_cmd.position = robotParams.head.actuators[actarray_cmd.joint].min;
    if (actarray_cmd.position > robotParams.head.actuators[actarray_cmd.joint].max)
      actarray_cmd.position = robotParams.head.actuators[actarray_cmd.joint].max;

    sputnik->ServoCtrSingle(actarray_cmd.joint, RAD2SPUTNIK(actarray_cmd.position), moveTime);
    drrobot_data.head.actuators[actarray_cmd.joint].position = actarray_cmd.position;
    retVal = 0;
  }
  //if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_CMD,
  //                         PLAYER_ACTARRAY_SPEED_CMD
  //			   this->actarray_id)) {
  // not available
  //  retVal = 0;
  //}


  // Update the time of last pulse/command on successful handling of commands
  if (retVal == 0 && pulse != -1)
  {
    gettimeofday (&timeVal, NULL);
    lastPulseTime = static_cast<double> (timeVal.tv_sec) + 
      (static_cast<double> (timeVal.tv_usec) / 1e6);
  }
  return retVal;
}

// Handlers for particular devices

void DRROBOT::HandlePositionCommand(player_position2d_cmd_vel_t position_cmd) {
  double speedDemand, turnRateDemand;
  double leftvel, rightvel;
  
  // TODO: poprawic changeTime  
  //unsigned int noControl=0x8000;
  unsigned int changeTime=0; // czas narostu zbocza w [ms]

  speedDemand = position_cmd.vel.px; // predkosc liniowa wzdłóżna [m/s]
  turnRateDemand = position_cmd.vel.pa; // predkosc obrotowa [rad/s]

  leftvel=speedDemand/robot_R-robot_L*turnRateDemand/robot_R/2; // [rad/s]
  rightvel=speedDemand/robot_R+robot_L*turnRateDemand/robot_R/2; // [rad/s]

  // send the speed command
  // TODO: upewnic sie, ze skala jest dobra
  sputnik->VelocityCtr((int)(leftvel/M_PI*600),(int)(rightvel/M_PI*600),100);
}
