/*! \file SputnikSDK.hh
 *
 *  \brief User application layer for DrRobot comunication protocol
 *
 *  Contains definition of class Sputnik_t and classes representing
 *  DrRobot sputnik robot
 * 
 *  \author Lukasz Malek
 *  \date 2008.09.17
 *  \version 1.00.00
 */

/*! \mainpage DrRobot Aplication Programing Interface
 * 
 *  \section Introduction
 *
 *  This document describes a Aplication Programing Interface (API) for
 *  DrRobot Sputnik robot. This API is based on a row protocol and allow to
 *  comunicate with robot under Linux system in simmilar way that it is
 *  posible in Windows environment. For any details related with the
 *  robot programing one should contact with the manufacturer - DrRobot.
 *
 *  \section Specification
 *
 *
 *
 *  \section Licence
 *
 *  This API is aviable under the GNU GPL. See 
 *  http://www.gnu.org/copyleft/gpl.html
 *  for more details.
 *
 *  \section Restrictions
 *
 *  Authors developed this software trying to make it as good as posibble. 
 *  Hovever they do not take any legal consequences of using this software.
 *  One is using it on its own risk.
 *
 */


#if !defined(SPUTNIKSDK_H)
#define SPUTNIKSDK_H

#include "DrRobotSDK.hh"

#define PWM_CONTROL 0
#define POSITION_CONTROL 1
#define VELOCITY_CONTROL 2
#define POT_SINGLE 0
#define POT_DUAL 1
#define ENCODER 2

enum Board_t_N {CONTROL=0,MEDIA=1};
enum Board_t_TECH {PMS5005=0,PMB5010=1};
enum Board_t_ID {CONTROL_ID=1,MEDIA_ID=8};  

enum ControlDevice_t_N {MOTORS=0,SENSORS=1,CUSTOMDATA=2};
enum ControlDevice_t_ID {MOTORS_ID=123,SENSORS_ID=125,CUSTOMDATA_ID=124};

enum MediaDevice_t_N {AUDIO=0,VIDEO=1,POWERCONTROLER=2};
enum MediaDevice_t_ID {AUDIO_ID=10,VIDEO_ID=9,POWERCONTROLER_ID=124};





/*! \class Sputnik_t
 *  \brief This class models a Sputnik robot device
 *  
 *  This class models a Sputnik robot device. It is base on the DrRobot_t 
 *  general class. 
 */
class Sputnik_t: public DrRobot_t {
public:
  /*! \brief Constructor
   *
   *  Constructor. Object can be created in static or dynamic way.
   */
  Sputnik_t();

  /*! \brief Destructor
   *
   *  Destructor
   */
  ~Sputnik_t();
  
  /*! \brief Connect to the robot
   *
   *  Connect to the robot. Returns result of the connection. 
   *  Port numbers of the conection must be exactly the same on witch the
   *  robot is listening on. Usualy they are portno1=10001 portno2=10002.
   *  
   *  \param servername - name or IP of the server
   *  \param portno1 - port number to the sensor&motor board
   *  \param portno2 - port number to the media board
   *  \return Returns true if conection to all ports succeded. Otherwise
   *          returns false.
   *  \post It connection to any ports fails all robot remains disconected.
   */
  bool Connect(const char* servername,unsigned int portno1,
	       unsigned int portno2);
  /*! \brief Disconnect from the robot
   *
   *  Disconnect from the robot
   */
  void Disconnect();

  /*! \brief Start the robot
   *  
   *  Start the robot
   */ 
  void Start();
  
  /*! \brief Stop the robot
   *  
   *  Stop the robot
   */   
  void Stop();
  
  /*! \brief Check if robot is active
   *  
   *  Check if robot is active
   */
  bool isActive(){return active;};
  
  /*! \brief move eyes
   *
   *  Move eyes
   *
   *  \param positionV - vertical rotation from -100 to 100
   *  \param positionH - horizontal rotation from -100 to 100
   *  \param speed - movement speed from 0 to 100
   */
  void MoveEyesVH(char positionV, char positionH, unsigned char speed);

  /*! \brief Move neck
   *
   *  Move neck
   *
   *  \param positionV - vertical rotation from -100 to 100
   *  \param positionH - horizontal rotation from -100 to 100
   *  \param speed - movement speed from 0 to 100
   */
  void MoveNeckVH(char positionV, char positionH, unsigned char speed);

  /*! \brief Move mouth
   *
   *  Move mouth
   *
   *  \param positionV - horizontal rotation from 0 to 100
   *  \param speed - movement speed from 0 to 100
   */
  void MoveMouth(unsigned char positionV,unsigned char speed);
  
  /*! \brief Get current sonar value
   *
   *  Return the current distance value between the relevant ultrasonic range 
   *  sensor and the obstacle inf front of it. The unit is [cm].
   *
   *  \param channel - 0 for the first sonar, 1 for the second, and so one up 
   *                   to 2 that is related to the 3-rd sonar
   *  \return Distance to the obstacle in [cm]
   */
  unsigned char GetSonar(unsigned char channel);

  /*! \brief Get current IR sensor value
   *
   *  Return the current IR sensor value between the relevant range 
   *  sensor and the obstacle inf front of it. The unit is [cm].
   *
   *  \param channel - 0 for the first IR, 1 for the second, and so one up 
   *                   to 7 that is related to the 8-th IR
   *  \return Distance to the obstacle in [cm]
   */
  unsigned char GetIR(unsigned char channel);

  /*! \brief Get current human motion state
   *
   *  Return the current human motion state. This is a basicly modified 
   *  value from the pirometer.
   *
   *  \param channel - 0 for the first pirometer, 1 for the second
   *  \return Methrod returns 0 for no movement and up to 100 for the 
   *  maximal movement
   */
  unsigned char GetHumanMotion(unsigned char channel);  
  //unsigned char GetHumanAlarm(unsigned char channel);  //useless
  
  /*! \brief Get current air temperature near relevant DC motor
   *
   *  Rerurn the current air temperature value near the relevant DC motor drive
   *  module. This information allow to measure wheather motor is 
   *  overheated or not
   *
   *  \param channel - 0 for the first temperature sensor, 1 for the second 
   *                   temperature sensor
   *  \return The return temperature in Celcius
   */
  unsigned char GetOverheat(unsigned char channel);

  /*! \brief Get current battery voltage
   *
   *  Return the current value of the relevant power supply voltage if battery
   *  voltage monitor is enable.
   *
   *  \param channel - Represents one of the three aviable battey, can have one 
   *                   of the following values:
   *                   <ul>
   *                     <li>0 for the main board battery, 
   *                     <li>1 for the DC motor battery,
   *                     <li>2 for the servo battery.
   *                   </ul>
   *  \return The return data is battery voltage.
   */ 
  float GetBattery(unsigned char channel);

  /*! \brief Get current reference or  power supply voltage
   *
   *  Returns the current current reference or  power supply voltage
   *  
   *  \param channel - Represents one of the three aviable battey, can have one 
   *                   of the following values:
   *                   <ul>
   *                     <li>0 for the reference voltage
   *                     <li>1 power supply voltage on the potentiometer
   *                   </ul>
   *  \return The return data is voltage
   */
  float GetVoltage(unsigned char channel);

  /*! \brief Get current potentiometer position
   *
   *  Return the current velue of the speciefied potentiomoter position sensor
   *
   *  \param channel - 0 for the first potentiometer, 1 for the second
   *  \return The return data an angle
   */
  short int GetPotentiometer(unsigned char channel);
  
  /*! \brief Get the sampling value of motor current sensor
   *
   *  Returns the sampling value of motor current sensor
   *
   *  \param channel - 0 for the first sensor, 1 for the second
   *  \return The return data is a current value in Ampers
   */
  float GetCurrent(unsigned char channel);

  /*! \brief Get from the encoder the direction of the rotation
   *
   *  Returns the dirrection of the rotation
   *
   *  \param channel - 0 for the first encoder, 1 for the second encoder
   *  \return There may appear the following return values:
   *          <ul>
   *            <li>1 - move in positive direction
   *            <li>0 - no move
   *            <li>-1 - move in negative direction
   *          </ul>
   *  \pre Chanel must be 0 or 1. Otherwise method return 0;
   */
  char GetEncoderDir(unsigned char channel);

  /*! \brief Get from the encoder the current pulse counter
   *
   *  Returns the current pulse conter to indicate the position of the rotation
   *
   *  \param channel - 0 for the first encoder, 1 for the second encoder
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 32767.
   *  \pre Chanel must be 0 or 1. Otherwise method return 0;
   */  
  unsigned short int GetEncoderPulse(unsigned char channel);

  /*! \brief Get from the encoder the current speed of rotation
   *
   *  Returns the current speed of rotation
   *
   *  \param channel - 0 for the first encoder, 1 for the second encoder
   *  \return The unit is defined as pulse change within 1 second and it 
   *          is a absolute value
   *  \pre Chanel must be 0 or 1. Otherwise method return 0;
   */ 
  unsigned short int GetEncoderSpeed(unsigned char channel);

  /*! \brief Set the motor polarity
   *
   *  Set the motor polarity to 1 of -1 for each motor channel
   *
   *  \param channel - 0 for the first motor, 1 for the second
   *  \param polarity - Can be 1 or -1.
   *                    <ul>
   *                      <li> When motor is running in positive direction, the
   *                           potentiometer value is also increasing; motor 
   *                           polarity should be set to 1 which is default
   *                      <li> When motor is running in positive direcion, the
   *                           potentiometer value is decreasing, motor plarity
   *                           should be set to -1 or change the sensor mounting
   *                           so that the potentiometer value increases.
   *                    </ul>
   */
  void SetMotorPolarity(unsigned char channel,char polarity);

  /*! \brief Resume all DC motors
   *
   *  Resumes all DC motors.
   */
  void ResumeMotors();
  
  /*! \brief Suspend all DC motors
   *
   *  Suspend all DC motors. 
   *
   *  \post All PWM outputs are low.
   */
  void SuspendMotors();

    /*! \brief Set position control PID 
   *
   *  Set up the PID control paramters of the specified DC motor channel
   *  for position control
   *
   *  \param Kp - proportional gain
   *  \param Kd - derivative gain
   *  \param Ki_x100 - 100 times integral gain
   */
  void SetPositionControlPID(unsigned short int Kp, 
			     unsigned short Kd, 
			     unsigned short int Ki_x100);

  /*! \brief Set velocity control PID 
   *
   *  Set up the PID control paramters of the specified DC motor channel
   *  for velocity control
   *
   *  \param Kp - proportional gain
   *  \param Kd - derivative gain
   *  \param Ki_x100 - 100 times integral gain
   */
  void SetVelocityControlPID(unsigned short int Kp, 
			     unsigned short Kd, 
			     unsigned short int Ki_x100);

  /*! \brief Set the motor sensor type
   *
   *  Set up the sensor type for the specified DC motor control channel
   *  The aviable sensor types are single potentiometer, dual potentiometer,
   *  and quadrature encoder. The single potentiometer sensor is for the
   *  control of robot joint with limited rotation (0 to 332 degree). 
   *  The dual potentiometers and the quadrature sensors ar for continous
   *  rotating joint control
   *
   *  \param SensorType - 0 for single potentiometer, 1 for dual potentiometer,
   *                      and 2 dor quadrature encoder
   */
  void SetSensorUsage(unsigned char SensorType);

  /*! \brief Set the motor control mode
   * 
   *  Set the control mode for the specified DC motor control channel.
   *  The available control modes are open PWM, closed-loop posiion control,
   *  closed-loop velocity control.
   *
   *  \param ControlMode - control type:
   *                   <ul>
   *                   <li>0 - open-loop PWM
   *                   <li>1 - closed-loop position control
   *                   <li>2 - closed-loop velocity control
   *                   </ul>
   */ 
  void SetControlMode(unsigned char ControlMode);

  /*! \brief Sends position control command
   * 
   *  Sends position control command to the specified motor channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second
   *  \param cmdValue - Target position value, data range is:
   *                    <ul>
   *                    <li>0-4095 - single potentiometer
   *                    <li>0-4428 - dual potentiometer
   *                    <li>0-32767 - encoder
   *                    </ul>  
   *  \param timePeriod - executing time in miliseconds
   */
  void PositionCtrSingle(unsigned char channel,
			 unsigned short int cmdValue,
			 unsigned short int timePeriod =0);

  /*! \brief Sends velocity control command
   * 
   *  Sends velocity control command to the specified motor channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second
   *  \param cmdValue - Target velocity value, units are:
   *                    <ul>
   *                    <li>position change/second - single potentiometer
   *                    <li>position change/second - dual potentiometer
   *                    <li>pulse/second - encoder
   *                    </ul>  
   *  \param timePeriod - executing time in miliseconds
   */
  void VelocityCtrSingle(unsigned char channel,
			 short int cmdValue,
			 unsigned short int timePeriod =0);

  /*! \brief Sends PWM control command
   * 
   *  Sends PWM control command to the specified motor channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second
   *  \param cmdValue - Target pulse width value, range is 0 to 32767, and 
   *                    coresponds to the duty cycle og 0 to 100% linearly.
   *                    A pulse width value 16363 means 50% duty cycle puting 
   *                    motor in "Stop" stage. Any value between 16364-32767
   *                    will put motor to run clockwise and any value between
   *                    0-16362 will put the motor to turn counter-clockwise
   *  \param timePeriod - executing time in miliseconds
   */
  void PwmCtrSingle(unsigned char channel,
		    unsigned short int cmdValue,
		    unsigned short int timePeriod =0);
 /*! \brief Sends position control command to all motors
   * 
   *  Sends position control command to all 6 DC motors channel.
   *  Target position value data range is:
   *                   <ul>
   *                   <li>0-4095 - single potentiometer
   *                   <li>0-4428 - dual potentiometer
   *                   <li>0-32767 - encoder
   *                   </ul> 
   *  Value -32768 (0x8000) means no control
   *
   *  \param cmdValue0 - chanel 0 position value
   *  \param cmdValue1 - chanel 1 position value
   *  \param timePeriod - executing time in miliseconds
   */
  void PositionCtr(unsigned short int cmdValue0,
		   unsigned short int cmdValue1,
		   unsigned short int timePeriod=0);

  /*! \brief Sends velocity control command to all motors
   * 
   *  Sends velocity control command to all 6 DC motors channel.
   *  Target velocity value data unit is:
   *                   <ul>
   *                   <li>position change/second - single potentiometer
   *                   <li>position change/second - dual potentiometer
   *                   <li>pulse/second - encoder
   *                   </ul> 
   *  Value -32768 (0x8000) means no control
   *
   *  \param cmdValue0 - chanel 0 velocity value
   *  \param cmdValue1 - chanel 1 velocity value
   *  \param timePeriod - executing time in miliseconds
   */
  void VelocityCtr(short int cmdValue0,
		   short int cmdValue1,
		   unsigned short int timePeriod=0);

  /*! \brief Sends PWM control command to all motors
   * 
   *  Sends PWM control command to all 6 DC motors channel.
   *  Target pulse width value, range is 0 to 32767, and 
   *  coresponds to the duty cycle og 0 to 100% linearly.
   *  A pulse width value 16363 means 50% duty cycle puting 
   *  motor in "Stop" stage. Any value between 16364-32767
   *  will put motor to run clockwise and any value between
   *  0-16362 will put the motor to turn counter-clockwise
   *  Value -32768 (0x8000) means no control.
   *
   *  \param cmdValue0 - chanel 0 PWM value
   *  \param cmdValue1 - chanel 1 PWM value
   *  \param timePeriod - executing time in miliseconds
   */
  void PwmCtr(unsigned short int cmdValue0,
	      unsigned short int cmdValue1,
	      unsigned short int timePeriod=0);

  /*! \brief Enables all servos
   *
   *  Enables all servo motorls chontrol channels
   *
   */ 
  void EnableServos();

  /*! \brief Disable all servos
   *
   *  Disable all servo motors chontrol channels
   *
   */ 
  void DisableServos();

  /*! \brief Sends position control command to the servo
   * 
   *  Sends position control command to the specified servo motor control
   *  channel
   *
   *  \param channel - 0 for the first servo, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th servo
   *  \param cmdValue - Target velocity value, unit is
   *                    (Pulse width in milisecond)*2250
   *                    usualy 1ms stands for 0, 1.5ms for 90 and 2 for 180 
   *                    degree
   *  \param timePeriod - executing time in miliseconds
   */
  void ServoCtrSingle(unsigned char channel, 
		      unsigned short int cmdValue,
		      unsigned short int timePeriod=0);

  /*! \brief Sends position control command to all servos
   * 
   *  Sends position control command to all 6 servo motors channel.
   *  Target velocity value, unit is (Pulse width in milisecond)*2250
   *  usualy 1ms stands for 0, 1.5ms for 90 and 2 for 180 degree
   *  Value -32768 (0x8000) means no control
   *
   *  \param neckV - neck vertical from -100 to 100
   *  \param neckH - neck horizontal from -100 to 100
   *  \param mouth - mouth from 0 to 100
   *  \param eyeV - eye vertical from -100 to 100
   *  \param eyeH - eye horizontal from -100 to 100
   *  \param timePeriod - executing time in miliseconds
   */
  void ServoCtr(char neckV,char neckH,unsigned char mouth,
		char eyeV, char eyeH,
		unsigned short int timePeriod=0);

  /*! \brief Display PGM image on the LCD
   *
   *  Display the image data in PGM format on the graphic LCD
   *
   *  \param bmpFileName - pointer to the table of char contaning
   *  
   *  \pre The image has to be monohromatic with dimention of 128 by 64
   *       pixels.
   */
  void DisplayPGM(char* bmpFileName);
  
  /*! \brief Returns battery voltage
   *
   *  Returns battery voltage
   *
   *  \param channel - 0 for the first battery, 1 for the second
   *
   */
  unsigned short int GetBatVolt(unsigned char channel);

  /*! \brief Returns battery temperature
   *
   *  Returns battery temperature
   *
   *  \param channel - 0 for the first battery, 1 for the second
   *
   */
  unsigned short int GetBatTemp(unsigned char channel);

  /*! \brief Returns DC input voltage
   *
   *  Returns DC input voltage  
   *
   */
  unsigned short int GetDCINVolt();

  /*! \brief Is charging
   *
   *  If robot is being charged.
   *
   */ 
  bool Charging();

  /*! \brief If there is a powerfail 
   *
   *  If there is a powerfial
   *
   */ 
  bool Powerfail();

  /*! \brief DC input comparator output
   *
   * DC input comparator output
   *
   */ 
  bool DCINCompOut();

  /*! \brief If there is a low power 
   *
   *  If there is a low power
   *
   */  
  bool LowerPower();

  /*! \brief If there is a fault
   *
   *  If there is a fault
   *
   */ 
  bool Fault();

  /*! \brief Returns reference voltage
   *
   *  Returns reference voltage
   *
   */
  unsigned short int GetRefVolt();

  /*! \brief If robot is powered form DCIN
   *
   *   If robot is powered form DCIN
   *
   */ 
  bool PoweredDCIN();

  /*! \brief If robot is powered form battery 2
   *
   *   If robot is powered form battery 2
   *
   */ 
  bool PoweredBat2();

  /*! \brief If robot is powered form battery 1
   *
   *   If robot is powered form battery 1
   *
   */ 
  bool PoweredBat1();
  
  /*! \brief If battery 2 is charged
   *
   *  If battery 2 is charged
   *
   */ 
  bool ChargeBat2();

  /*! \brief If battery 1 is charged
   *
   *  If battery 1 is charged
   *
   */ 
  bool ChargeBat1();

  /*! \brief Set camera image refresh period
   *
   *  Set camera image refresh period
   *
   *  \param T image refresh rate in seconds
   *
   */ 
  void SetVideo_T(float T);

  /*! \brief Returns an OpenCV image
   *
   *  Returns an OpenCV image form the camera
   *
   *  \return Return the pointer to the IplImage (OpenCV image)
   */
  IplImage* getIplImage();

  /*! \brief Start recodring
   *
   *  Start recodring an audio file.
   *
   *  \param VoiceSegment - specify the time of voice segment, unit is 256
   *                        millisecond (about 1⁄4 sec). Value could be 
   *                        1- 10. For example, if voiceSegment is 4, then
   *                        1.024 second voice recorded,
   *
   */  
  void StartRecord(unsigned char VoiceSegment=4);

  /*! \brief Stop recodring
   *
   *  Stop recording an audio file
   *
   */  
  void StopRecord();

  /*! \brief Start plaing audio file
   *
   *  Start playing audio previouse loaded by the procedure ReadWAV().
   *
   */  
  void StartAudioPlay();

  /*! \brief Stop plaing audio file
   *
   *  Start playing audio file
   *
   */   
  void StopAudioPlay();

  /*! \brief Save audio to WAV
   *
   *  Save audio incoming from the robot to WAV file
   *
   *  \param FileName - file name
   */
  void SaveWAV(char * FileName);

  /*! \brief Read audio from WAV
   *
   *  Read audio from WAV and save it in local buffer. It is nesssery to be
   *  able to play any audio from the robot by the method StartAudioPlay()
   *
   *  \param FileName - file name

   */
  void ReadWAV(char* FileName);

  /*! \brief Gets input to the robot voice buffer
   *
   *  Gets input to the robot voice buffer
   */
  short* &GetInVoiceBuffer();

  /*! \brief Gets input to the robot voice buffer length
   *
   *  Gets input to the robot voice buffer lenght
   */
  long & GetInVoiceBufferSize(); 

  /*! \brief Gets output form the robot voice buffer
   *
   *  Gets output from the robot  voice buffer
   */
  short* GetOutVoiceBuffer();

  /*! \brief Gets output form the robot voice buffer length
   *
   *  Gets output from the robot  voice buffer length
   */
  long GetOutVoiceBufferSize(); 

  /*! \brief Clears the output form the robot bufer
   *
   *   Clears the output form the robot bufer
   */
  void ClearOutVoiceBuffer();

private:
  bool active;
  friend void * taking_photo(void *ptr); //!< Cyclic taking photos function
  pthread_t videoThread; //!< Thread sending preiodicaly TakePhoto()
  float video_T; //! takePhoto interval in seconds, 0 - disable

  short *InVoiceBuffer;
  long InVoiceBufferSize;

  struct Face_t{
    char EyesV;
    char EyesH;
    char NeckV;
    char NeckH;
    unsigned char Mouth;
  } face;

  struct Dimentions_t{
    float wheel; // wheel radus in cm 8.88
    float distance; //distance between the wheel and robot center 13.5
  } dimentions;
  
  unsigned char SonarEMA[3];
  float SonarEMAAlpha;
  unsigned char IREMA[8];
  float IREMAAlpha;
  unsigned char HMEMA[2];
  float HMEMAAlpha;
  unsigned char *HMresult;
  unsigned char HAEMA[2];
  float HAEMAAlpha;
  unsigned char *HAresult;
  int okno;
  
  unsigned char sensor_type;
  unsigned char control_mode;
};

  
#endif // SPUTNIKSDK_H
