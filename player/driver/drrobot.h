/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * $Id: drrobot.h,v 1.30 2006/04/04 21:31:37 gbiggs Exp $
 *
 *   the DRROBOT device.  it's the parent device for all the P2 'sub-devices',
 *   like gripper, position, sonar, etc.  there's a thread here that
 *   actually interacts with DRROBOT via the serial line.  the other
 *   "devices" communicate with this thread by putting into and getting
 *   data out of shared buffers.
 */
#ifndef _DRROBOTDEVICE_H
#define _DRROBOTDEVICE_H

#include <pthread.h>
#include <sys/time.h>

#include <libplayercore/playercore.h>

#include "SputnikSDK.hh"
#include <limits>

// Default max speeds
#define MOTOR_DEF_MAX_SPEED 0.5
#define MOTOR_DEF_MAX_TURNSPEED DTOR(100)

/*
 * Apparently, newer kernel require a large value (200000) here.  It only
 * makes the initialization phase take a bit longer, and doesn't have any
 * impact on the speed at which packets are received from DRROBOT
 */
#define DRROBOT_CYCLETIME_USEC 200000

/* drrobot constants */

#define DRROBOT_NOMINAL_VOLTAGE 12.0

/* conection stuff */
#define DEFAULT_DRROBOT_PORT "/dev/ttyS0"
#define DEFAULT_DRROBOT_TCP_REMOTE_HOST "localhost"
#define DEFAULT_DRROBOT_TCP_REMOTE_PORT_A 10001
#define DEFAULT_DRROBOT_TCP_REMOTE_PORT_B 10002

#define PI 3.14159265

#define LASER

const double robot_R=0.17/2;
const double robot_L=0.268;
short max_short=std::numeric_limits<short>::max();

/*!
 * Structure modeling pose of sonar range sensor
 */
typedef struct
{
  double x;  //! x coordinate
  double y;  //! y coordinate
  double th; //! angle from OX axis coordination
} sonar_pose_t;

/*!
 * Structure modeling pose of IR range sensor
 */
typedef struct
{
  double x;  //! x coordinate
  double y;  //! y coordinate
  double th; //! angle from OX axis coordination
} ir_pose_t;

/*!
 * Structure modeling robot from technical point of view 
 */
typedef struct
{
  double AngleConvFactor; //! \todo DESCRIBE IT
  double DiffConvFactor;  //! \todo DESCRIBE IT
  int IRNum;              //! number of IR sensors
  double RobotLength;     //! robot length in meters [m]
  double RobotWidth;      //! robot width in meters [m]
  int SonarNum;           //! number of sonars
  int SwitchToBaudRate;   //! \todo DESCRIBE IT
  double Vel2Divisor;     //! \todo DESCRIBE IT
  sonar_pose_t sonar_pose[3]; //! poses of sonar sensors
  ir_pose_t ir_pose[8];   //! poses of IR sensors
  player_actarray_geom_t head; //! head actuators parameters
} RobotParams_t;


/*! 
 * Structure containing robot date in PLAYER structures
 */
typedef struct player_drrobot_data
{
  player_position2d_data_t position; //! platform position 
  player_sonar_data_t sonar; //! sonars
  player_ir_data_t ir;       //! IR sensors
  player_power_data_t power; //! power control
  player_actarray_data_t head; //! head actuators
} __attribute__ ((packed)) player_drrobot_data_t;


/* 
 * Structure modeling robot from kinematic point of view
 */
class Sputnik_K_t : public Sputnik_t{
  public:
  double x;    //! x position in [m]
  double y;    //! y position in [m]
  double w;    //! w angle from OX axis in [rad]
  double dt;   //! time quantum, needed for the kinematics
  double ul;   //! angular velocity of the left wheel, needed for the kinematics
  double ur;   //! angular velocity of the left wheel, needed for the kinematics
  short enc_0; //! current left (from robot perspective) encoder state 
  short enc_1; //! current right (from robot perspective) encoder state
  long encL_0; //! total left (from robot perspective) encoder position
  long encL_1; //! total right (from robot perspective) encoder position
  long encL_0_old; //! previous total left (from robot perspective) encoder position
  long encL_1_old; //! previous total right (from robot perspective) encoder positon
  timeval t0; //! previous time moment - needed for the kinematics
  timeval t1; //! current time moment - needed for the kinematics
};


/*!
 * Player driver for the DrRobot Sputnik robots.
 */
class DRROBOT : public Driver{
 private:
   Sputnik_K_t *sputnik; //! Sputnik software interface\

   RobotParams_t robotParams; //! robot technical parameters
   
   player_drrobot_data_t drrobot_data; //! Sputnik robot data
				       //! structure for Player
   
   // devices adresses
   player_devaddr_t position_id; //! position interface addres
   player_devaddr_t sonar_id;    //! sonar interface addres
   player_devaddr_t ir_id;       //! IR interface addres
   player_devaddr_t power_id;    //! power control interface addres
   player_devaddr_t actarray_id;  //! head actuators interface addres

   // PID settings
   int pos_kp, pos_kd, pos_ki, vel_kp, vel_kd, vel_ki;
   
   //! number of subscriptions to the position driver
   int position_subscriptions; 	
   //! number of subscriptions to the sonar driver
   int sonar_subscriptions;
   //! number of subscriptions to the IR driver
   int ir_subscriptions;
   //! number of subscriptions to the head actuators driver
   int actarray_subscriptions;

   /*!
    * Zeros robot platform position
    */
   void ResetRawPositions();

   /*!
    * toggle motors on/off, according to val 
    */
   void ToggleMotorPower(unsigned char val);

   /*!
    * toggle actarray servos on/off, according to val 
    */
   void ToggleActarrayPower(unsigned char val);


   /*! 
    * handle configuration incoming messages
    */
   int HandleConfig(MessageQueue* resp_queue,
		    player_msghdr * hdr,
		    void* data);
   /*! 
    * handle incoming commands
    */
   int HandleCommand(player_msghdr * hdr, void * data);

   /*!
    * Publish robot data to the Player server
    */ 
   void PutData(void);

   /*! 
    * Function realising commands to the position driver
    */
   void HandlePositionCommand(player_position2d_cmd_vel_t position_cmd);
   
   int param_idx;          //! index in the RobotParams table for this robot
   int psos_fd;            //! drrobot device file descriptor
   const char* psos_serial_port; //! name of serial port device
   bool psos_use_tcp;      //! use TCP port instead of serial port
   const char* psos_tcp_host;  //! hostname to use if using TCP
   int psos_tcp_port_A;    //! remote port to use if using TCP
   int psos_tcp_port_B;    //! remote port to use if using TCP
   // const char* board;   // name of the board 
   // int boardID;         // board ID = RID
   
   // Max motor speeds (mm/sec,deg/sec)
   // int motor_max_speed;     //! maximum linear velocity [mm/s]
   // int motor_max_turnspeed; //! maximum turnrate velocity [deg/s]
   
   // Max motor accel/decel (mm/sec/sec, deg/sec/sec)
   //short motor_max_trans_accel, motor_max_trans_decel;
   //short motor_max_rot_accel, motor_max_rot_decel;
   
   float pulse;            //! Pulse time
   double lastPulseTime;   //! Last time of sending a pulse or command to the robot
   void SendPulse (void);  //! Send "ping" to the robot
   void RefreshData(void); //! geather data from devices
   
 public:
   /*!
    * DRROBOT constructor
    */
   DRROBOT(ConfigFile* cf, int section);

   /*!
    * DRROBOT destructor
    */   
   ~DRROBOT (void);
   
   /*! 
    * Subscribe new device to the driver
    */
   virtual int Subscribe(player_devaddr_t id); 

   /*!
    * Unsubscribe device from the driver
    */
   virtual int Unsubscribe(player_devaddr_t id);
   
   /*!
    * Driver main loop function
    */
   virtual void Main();
   
   /*!
    * Initialize robot - start acting
    */
   virtual int Setup();

   /*!
    * Deinitialize robot - stop acting
    */
   virtual int Shutdown();
   
   /*!
    * Handling obtained messanges
    */
   virtual int ProcessMessage(MessageQueue * resp_queue,
			      player_msghdr * hdr,
			      void * data);
};


#endif
