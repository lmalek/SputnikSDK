#ifndef DRROBOTSDK_H
#define DRROBOTSDK_H

/*! \file DrRobotSDK.hh
 *
 *  \brief User application layer for DrRobot comunication protocol
 *
 *  Contains definition of class DrRobot_t and classes representing
 *  boards and deviceas that can be use in DrRobot robots. It is a
 *  lirbrary that contains classes and methods wrapping basic UDP
 *  comunication protocol for PMS5005 and PMB5010.
 * 
 *  \author Lukasz Malek
 *  \date 2008.09.17
 *  \version 1.00.00
 */


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <map>

#define INDEX_STX0	0
#define INDEX_STX1	1
#define INDEX_DES	2
#define INDEX_SN	3
#define INDEX_TYPE	4
#define INDEX_LENGTH	5
#define INDEX_DATA	6

#define COMTYPE_SYSTEM	        0xFF
#define COMTYPE_SENSOR		0x7F
#define COMTYPE_AUDIO		0x0A
#define COMTYPE_VIDEO		0x09
#define COMTYPE_MOTOR		40
#define CMTYPE_ADPCM_RESET	0x33
#define AUDIO_PLAY_BUFFER_READY	0x10

#define COMTYPE_MOT_MOTOR_SENSOR        0x7B
#define COMTYPE_MOT_USER_SENSOR		0x7C
#define COMTYPE_MOT_ULTRASONIC_SENSOR	0x7D

#define DATA_ACK	 0x01
#define DATA_PING	 0x00
#define DATA_URGENT_DATA 0x02
#define DATA_SKIPPACKET	 0x03

enum Board_t_N {CONTROL=0,MEDIA=1};
enum Board_t_TECH {PMS5005=0,PMB5010=1};
enum Board_t_ID {CONTROL_ID=1,MEDIA_ID=8};  

enum ControlDevice_t_N {MOTORS=0,SENSORS=1,CUSTOMDATA=2};
enum ControlDevice_t_ID {MOTORS_ID=123,SENSORS_ID=125,CUSTOMDATA_ID=124};

enum MediaDevice_t_N {AUDIO=0,VIDEO=1,POWERCONTROLER=2};
enum MediaDevice_t_ID {AUDIO_ID=10,VIDEO_ID=9,POWERCONTROLER_ID=124};

#define PWM_CONTROL 0
#define POSITION_CONTROL 1
#define VELOCITY_CONTROL 2
#define POT_SINGLE 0
#define POT_DUAL 1
#define ENCODER 2

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */

class DrRobotBoard_t;

////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEVICE ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*! \class DrRobotDevice_t
 *  \brief Meta device class from which real devices interitate
 *  
 *  This class model a meta device class that describes a basic 
 *  device fnctionality and properties.
 */
class DrRobotDevice_t{
public:
  /*! \brief Parametric constructor
   *
   *  Construct a device by defining its ID and parent board
   *  \param dID - identification number of the device (look at the protocol 
   *               description). We assume that device ID is an ID on which 
   *               a apropreate are send to.
   *  \param board - pointer to the board that this device is conected to.
   */
  DrRobotDevice_t(unsigned char dID, DrRobotBoard_t *board);

  virtual ~DrRobotDevice_t(){};

  /*! \brief Refresh the data related to this device
   *
   *  Refresh the data that are stored in this class and that are related with
   *  that device. As a parameter it should obtain a whole data package related
   *  with the data.
   *  \param data - pointer to the table of chars consisting a whole data 
   *                package directed to this specific device.
   *  \param dataLength - lenght of the data.
   */
  virtual void RefreshData(unsigned char* data,unsigned long dataLength){};

  /*! \brief Sensor data obtaining control function
   *  
   *  Method that contorl sending from the robot information about sensoes state
   *  \param PacketNumber - describes type of geathering data
   *                        <ul>
   *                          <li> PacketNumber =  0 - Stop sendig the 
   *                                                   sensor data packets
   *                          <li> PacketNumber = -1 - Send semsor data 
   *                                                   continously ubtil being
   *                                                   asked to stop
   *                          <li> PacketNumber >  0 - Send exacpty 
   *                                                   PacketNumber sensor data
   *                                                   and then stop sending
   *                        </ul>
   */
  virtual void SystemSensorSending(int PacketNumber){};

  /*! \brief Start sending data continously
   * 
   *  Start sending data continously
   */
  virtual void EnableSensorSending(void){};

  /*! \brief Stop sending data
   * 
   *  Stop sending data
   */
  virtual void DisableSensorSending(void){};

  /*! \brief Send data function 
   *  
   *  Sending data function to the device with specyfic ID
   *  \param ddID - destination device ID
   *  \param dataLength - length of the data to be send
   *  \param data - pointer to the table of char containing the data to be send
   */
  void SendData(unsigned char ddID,unsigned short dataLength, 
		unsigned char* data);
private:
  /*! \brief Device ID
   *
   *  Field containing device indentification number  
   */
  unsigned char dID;

  /*! \brief Pointer to parent board
   *
   *  Field containing pointer to the board that this device is conected to
   */  
  DrRobotBoard_t *board;
};

////////////////////////////////////////////////////////////////////////////////
//////////////////////// SENSORS ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  
/*! \class DrRobotSensors_t
 *  \brief This class model a board with sensors
 *  
 *  This class contains a methods that allow to refresh data geather form 
 *  the robot sensors, methods to control data geathering and methods usefull
 *  to obtain specific sensor values.
 */
class DrRobotSensors_t: public DrRobotDevice_t {
public:
  /*! \brief Parametric constructor
   *
   *  Construct a device by defining its ID and parent board
   *  \param dID - identification number of the device (look at the protocol 
   *               description). We assume that device ID is an ID on which
   *               a apropreate are send to.
   *  \param board - pointer to the board that this device is conected to.
   *  \sa DrRobotDevice_t::DrRobotDevice_t()
   */
  DrRobotSensors_t(unsigned char dID, DrRobotBoard_t *board);

  virtual ~DrRobotSensors_t(){};
  
  /*! \brief Refresh the data related to this device
   *
   *  Refresh the data that are stored in this class and that are related with
   *  that device. As a parameter it should obtain a whole data package related
   *  with the data.
   *  \param dataBuffer - pointer to the table of chars consisting a whole data
   *                      package directed to this specific device.
   *  \param dataLength - lenght of the data.
   *  \sa DrRobotDevice_t::RefreshData()
   */
  void RefreshData(unsigned char *dataBuffer,unsigned long dataLength);

  /*! \brief Sensor data obtaining control function
   *  
   *  Method that contorl sending from the robot information about sensoes state
   *  \param PacketNumber - describes type of geathering data
   *                        <ul>
   *                          <li> PacketNumber =  0 - Stop sendig the 
   *                                                   sensor data packets
   *                          <li> PacketNumber = -1 - Send semsor data 
   *                                                   continously ubtil being
   *                                                   asked to stop
   *                          <li> PacketNumber >  0 - Send exacpty 
   *                                                   PacketNumber sensor data
   *                                                   and then stop sending
   *                        </ul>
   */
  void SystemSensorSending(int PacketNumber);

  /*! \brief Start sending data continously
   * 
   *  Start sending data continously
   */
  void EnableSensorSending();

  /*! \brief Stop sending data
   * 
   *  Stop sending data
   */
  void DisableSensorSending();
  
  /*! \brief Get current sonar vlue
   *
   *  Return the current distance value between the relevant ultrasonic range 
   *  sensor DUR5200 and the obstacle inf front of it. The unit is [cm].
   *
   *  \param channel - 0 for the first sonar, 1 for the second, and so one up 
   *                   to 5 that is related to the 6-th sonar
   *  \return Distance to the obstacle in [cm]
   */
  unsigned char GetSonar(unsigned char channel);

  /*! \brief Get current human alarm value
   *
   *  Return the current human alarm from DHM5150.
   *
   *  \param channel - 0 for the first alarm, 1 for the second alarm sensor
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the amplified (x5 times) output voltage of the sensor
   *          device. The data range is  between 0 and 4095. When there is no 
   *          human present the module voltage is about 1.5 V and the return 
   *          value is about 2047
   */
  unsigned short int GetHumanAlarm(unsigned char channel);

  /*! \brief Get current human motion sensor value
   *
   *  Return the current human motion value from DHM5150.
   *
   *  \param channel - 0 for the first human motion sensor, 1 for the second
   *                   human motion sensor
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095.
   */
  unsigned short int GetHumanMotion(unsigned char channel);
  
  /*! \brief Get current tilting X value
   *
   *  Return the current tilt angle value in the X direction from DTA5102
   *
   *  \return If return value is \f$f(\alpha)\f$ then tilting angle \f$\alpha\f$
   *          can be calculated from the following formula 
   *          \f[
   *             \alpha = \arcsin\left(\frac{f(\alpha)-f(0^o)}{
   *                             |f(90^o)-f(0^o)|}\right)
   *          \f]
   *          Where \f$ f(0^o) \f$ is a value sensor value measured for angle
   *          equalzero and \f$ f(90^o) \f$ is the sensor value
   *          measured for the angle equal \f$ 90^o \f$. Typical values are
   *          \f$ f(0^o) \approx 2048  \f$ and  
   *          \f$ f(90^o) \approx 1250 \f$
   */
  unsigned short int GetTiltingX();
  /*! \brief Get current tilting Y value
   *
   *  Return the current tilt angle value in the Y direction from DTA5102
   *
   *  \return If return value is \f$f(\alpha)\f$ then tilting angle \f$\alpha\f$
   *          can be calculated from the following formula 
   *          \f[
   *             \alpha = \arcsin\left(\frac{f(\alpha)-f(0^o)}{
   *                             |f(90^o)-f(0^o)|}\right)
   *          \f]
   *          Where \f$ f(0^o) \f$ is a value sensor value measured for angle
   *          equalzero and \f$ f(90^o) \f$ is the sensor value
   *          measured for the angle equal \f$ 90^o \f$. Typical values are
   *          \f$ f(0^o) \approx 2048  \f$ and  
   *          \f$ f(90^o) \approx 1250 \f$
   */
  unsigned short int GetTiltingY();

  /*! \brief Get current air temperature near relevant DC motor
   *
   *  Rerurn the current air temperature value near the relevant DC motor drive
   *  module MDM5253. This information allow to measure wheather motor is 
   *  overheated or not
   *
   *  \param channel - 0 for the first temperature sensor, 1 for the second 
   *                   temperature sensor
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095. If the return value is \f$ f(T) \f$ then 
   *          temperature \f$ T \f$ represented i [C] can be calculated from 
   *          the follwing formula
   *          \f[
   *             T = 100 - \frac{f(T) - 980}{11.6}
   *          \f]
   */ 
  unsigned short int GetOverheatAD(unsigned char channel);
 
  /*! \brief Get current temperature 
   *
   *  Rerurn the current temperature value from DTA5280
   *
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095. If the return value is \f$ f(T) \f$ then 
   *          temperature \f$ T \f$ represented i [C] can be calculated from 
   *          the follwing formula:
   *          \f[
   *             T = 100 - \frac{f(T) - 1256}{34.8}
   *          \f]
   */  
  unsigned short int GetTemperature();
  
  /*! \brief Get current IR control sensor value
   *
   *  Return the current infra red control sensor value. The remote control
   *  commands are captured by PMS5005 throught the IR control module
   *  MIR5500
   *
   *  \param channel - 0 for the first code, 1 for the second, and so one up 
   *                   to 3 thtat is equivalent for the 4-th code. Range [0 - 3]
   *  \return Return the four parts of a two 16 bits code of remote control.
   *          Recovery information from each chanell (4 byte code) is following:
   *            Key Code    : [3-rd byte][2-nd byte][1-st byte]
   *            Repeat Code : [4-th byte]
   *          where the repeat code would be 255 if the button is pressed
   *          continuously.
   */ 
  unsigned char GetIRCode(unsigned char channel);

  /*! \brief Get current IR range sensor value
   *  
   *  Return the current infra red sensor value.
   *
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095.
   */
  unsigned short int GetIRRange();

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
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095. If the return vaue is <b>f(U)</b> then the
   *          voltage <b>U</b> [V] can be calculated in the following way:
   *          <ul>
   *            <li> Main board battery U = 9 * f(U)/4095
   *            <li> DC motor battery U = 24 * f(U)/4095
   *            <li> Servo battery U = 9 * f(U)/4095 
   *          </ul>
   */ 
  unsigned short int GetBatteryAD(unsigned char channel);
  
  /*! \brief Get current reference voltage
   *
   *  Returns the current value of the reference voltage of the A/D converter
   *  of the controler DSP
   *  
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095. If the return vaue is <b>f(U)</b> then the
   *          voltage <b>U</b> [V] can be calculated in the following way
   *             U = 6 * f(U)/4095
   */
  unsigned short int GetRefVoltage();

  /*! \brief Get current power supply voltage on the potentiometer
   *
   *  Returns the current value of the power supply votage of the potentiometer
   *  position sensor.
   *  
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095. If the return vaue is <b>f(U)</b> then the
   *          voltage <b>U</b> [V] can be calculated in the following way
   *             U = 6 * f(U)/4095
   */
  unsigned short int GetPotVoltage();
private:
  /*! \brief Field containing data from sensors 
   */
  struct SensorDataField {
    unsigned char Sonar[6];
    unsigned short int HumanAlarm1;
    unsigned short int HumanMotion1;
    unsigned short int HumanAlarm2;
    unsigned short int HumanMotion2;
    unsigned short int TiltingX;
    unsigned short int TiltingY;
    unsigned short int Overheat[2];
    unsigned short int Temperature;
    unsigned short int IRRange;
    unsigned char IRCode[4];
    unsigned short int BatteryAD[3];
    unsigned short int RefVoltage;
    unsigned short int PotVoltage;
  } SensorData;
}; // class DrRobotSensors_t

////////////////////////////////////////////////////////////////////////////////
//////////////////////// MOTORS ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


/*! \class DrRobotMotors_t
 *  \brief This class model a board controling motors and their sensors
 *  
 *  This class contains a methods that allow to refresh data geather form 
 *  the robot motors. This class alow to control motors and servos.
 */
class DrRobotMotors_t: public DrRobotDevice_t  {
 public:
 
  /*! \brief Parametric constructor
   *
   *  Construct a device by defining its ID and parent board
   *  \param dID - identification number of the device (look at the protocol 
   *               description). We assume that device ID is an ID on which
   *               a apropreate are send to.
   *  \param board - pointer to the board that this device is conected to.
   *  \sa DrRobotDevice_t::DrRobotDevice_t()
   */
  DrRobotMotors_t(unsigned char dID, DrRobotBoard_t *board);

  virtual ~DrRobotMotors_t(){};

  /*! \brief Refresh the data related to this device
   *
   *  Refresh the data that are stored in this class and that are related with
   *  that device. As a parameter it should obtain a whole data package related
   *  with the data.
   *  \param dataBuffer - pointer to the table of chars consisting a whole data 
   *                      package directed to this specific device.
   *  \param dataLength - lenght of the data.
   */
  void RefreshData(unsigned char *dataBuffer,unsigned long dataLength);

  /*! \brief Motor data obtaining control function
   *  
   *  Method that contorl sending from the robot information about motor state
   *  \param PacketNumber - describes type of geathering data
   *                        <ul>
   *                          <li> PacketNumber =  0 - Stop sendig the 
   *                                                   motor data packets
   *                          <li> PacketNumber = -1 - Send motor data 
   *                                                   continously ubtil being
   *                                                   asked to stop
   *                          <li> PacketNumber >  0 - Send exacpty 
   *                                                   PacketNumber motor data
   *                                                   and then stop sending
   *                        </ul>
   */
  void SystemSensorSending(int PacketNumber);

  /*! \brief Start sending data continously
   * 
   *  Start sending data continously
   */
  void EnableSensorSending();

  /*! \brief Stop sending data
   * 
   *  Stop sending data
   */
  void DisableSensorSending(); 
 
  /*! \brief Get current potentiometer position
   *
   *  Return the current velue of the speciefied potentiomoter position sensor
   *
   *  \param channel - 0 for the first potentiometer, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th potentiometer
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095. Angular position can be calculated as 
   *          follows, with th \f$ 180^o \f$ position defined at sensor 
   *          physical middle position. Single sensor od dual sensor can be
   *          used for toration measurement.
   *          <ul>
   *            <li> Single sensor is mainly used for the control of the robot
   *                 joint with limited rotation range. The effective 
   *                 mechanical rotation is \f$ 14^o \f$ to \f$ 346^o \f$
   *                 corresponding to the effective electrical rotation range
   *                 \f$ 0^o \f$ to \f$ 332^o \f$. Angle position \f$ \alpha \f$
   *                 can be calculated from the formula
   *                 \f[
   *                    \alpha = 333\frac{f(\alpha)-2048}{4095}+180
   *                 \f]
   *            <li> Dual-sensor configuration is mainly used for continuous
   *                 rotating joints (such as wheels). The effectivce rotation
   *                 range is \f$ 0^o \f$ to \f$ 360^o \f$. Dual sensor 
   *                 configuration is only aviable on chanel 0 and 1. By 
   *                 chonnecting two potentiometers to potentiometer channel 0
   *                 and channel 5 and specify the sensor type with command
   *                 SetSensorUsage() to "Dual potentiometer sensor", the 
   *                 channel 0 reading will combine these two sensors reading 
   *                 into \f$ 0^o \f$ to \f$ 360^o \f$ range. For channel 1,
   *                 you should connect two potentiometers to chanel 1 and 
   *                 channel 4. Angle position \f$ \alpha \f$
   *                 can be calculated from the formula
   *                 \f[
   *                    \alpha = 180\frac{f(\alpha)-2214}{2214}+180
   *                 \f]
   *          </ul>
   */
  unsigned short int GetPot(unsigned char channel);
  
  /*! \brief Get the sampling value of motor current sensor
   *
   *  Returns the sampling value of motor current sensor
   *
   *  \param channel - 0 for the first sensor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th sensor
   *  \return The return data is a raw value of the analog to digital converter
   *          indicating the output from the device The data range is 
   *          between 0 and 4095. Motor currnet \f$ I \f$ [A] can be
   *          calculated from the following formula
   *          \f[
   *              I=\frac{f(I)}{728}
   *          \f]
   */
  unsigned short int GetCurrent(unsigned char channel);

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
  char GetDir(unsigned char channel);

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
  unsigned short int GetPulse(unsigned char channel);

  /*! \brief Get from the encoder the current speed of rotation
   *
   *  Returns the current speed of rotation
   *
   *  \param channel - 0 for the first encoder, 1 for the second encoder
   *  \return The unit is defined as pulse change within 1 second and it 
   *          is a absolute value
   *  \pre Chanel must be 0 or 1. Otherwise method return 0;
   */ 
  unsigned short int GetSpeed(unsigned char channel);

  /*! \brief Set the motor polarity
   *
   *  Set the motor polarity to 1 of -1 for each motor channel
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
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
  void SetPolarity(unsigned char channel,char polarity);

  /*! \brief Resume DC motor
   *
   *  Resumes the specific DC motor control channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   */
  void Resume(unsigned char channel);

  /*! \brief Resume all DC motors
   *
   *  Resumes all DC motors.
   */
  void ResumeAll(void); 

  /*! \brief Suspend DC motor
   *
   *  Suspend the specific DC motor control channel. 
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \post PWM output is low.
   */
  void Suspend(unsigned char channel); 

  /*! \brief Suspend all DC motors
   *
   *  Suspend all DC motors. 
   *
   *  \post All PWM outputs are low.
   */
  void SuspendAll(void);  

  /*! \brief Set position control PID 
   *
   *  Set up the PID control paramters of the specified DC motor channel
   *  for position control
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param Kp - proportional gain
   *  \param Kd - derivative gain
   *  \param Ki_x100 - 100 times integral gain
   */
  void SetPositionControlPID(unsigned char channel,
			     unsigned short int Kp, 
			     unsigned short Kd, 
			     unsigned short int Ki_x100);

  /*! \brief Set velocity control PID 
   *
   *  Set up the PID control paramters of the specified DC motor channel
   *  for velocity control
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param Kp - proportional gain
   *  \param Kd - derivative gain
   *  \param Ki_x100 - 100 times integral gain
   */
  void SetVelocityControlPID(unsigned char channel,
			     unsigned short int Kp, 
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
   *  \param channel - this paramtser states for motor channel, but it has 
   *                   a different meaning depanding on the SensorType:
   *                   <ul>
   *                   <li>for single potentiometer - 0 for the first motor,
   *                       1 for the second, and so on up to 5 that is related
   *                       to the 6-th motor
   *                   <li>for dual potentiometer - 0 for the first motor, 
   *                       1 for the second, and 2 for the third
   *                   <li>for quadrature encoder -  0 for the first motor, 
   *                       1 for the second
   *                   </ul>
   *  \param SensorType - 0 for single potentiometer, 1 for dual potentiometer,
   *                      and 2 dor quadrature encoder
   */
  void SetSensorUsage(unsigned char channel,
		      unsigned char SensorType);  

  /*! \brief Set the motor control mode
   * 
   *  Set the control mode for the specified DC motor control channel.
   *  The available control modes are open PWM, closed-loop posiion control,
   *  closed-loop velocity control.
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param ControlMode - control type:
   *                   <ul>
   *                   <li>0 - open-loop PWM
   *                   <li>1 - closed-loop position control
   *                   <li>2 - closed-loop velocity control
   *                   </ul>
   */
  void SetControlMode(unsigned char channel,
		      unsigned char ControlMode);  

  /*! \brief Sends position control command
   * 
   *  Sends position control command to the specified motor channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param cmdValue - Target position value, data range is:
   *                    <ul>
   *                    <li>0-4095 - single potentiometer
   *                    <li>0-4428 - dual potentiometer
   *                    <li>0-32767 - encoder
   *                    </ul>  
   *  \param timePeriod - executing time in miliseconds
   */
  void PositionTimeCtr(unsigned char channel,
		       unsigned short int cmdValue,
		       unsigned short int timePeriod);

 /*! \brief Sends position control command
   * 
   *  Sends position control command to the specified motor channel. 
   *  The motion controller will drive the motor to the target position
   *  at the maximum speed
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param cmdValue - Target position value, data range is:
   *                   <ul>
   *                   <li>0-4095 - single potentiometer
   *                   <li>0-4428 - dual potentiometer
   *                   <li>0-32767 - encoder
   *                   </ul>  
   */
  void PositionNonTimeCtr(unsigned char channel,
			  unsigned short int cmdValue);

  /*! \brief Sends velocity control command
   * 
   *  Sends velocity control command to the specified motor channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param cmdValue - Target velocity value, units are:
   *                    <ul>
   *                    <li>position change/second - single potentiometer
   *                    <li>position change/second - dual potentiometer
   *                    <li>pulse/second - encoder
   *                    </ul>  
   *  \param timePeriod - executing time in miliseconds
   */
  void VelocityTimeCtr(unsigned char channel,
		       unsigned short int cmdValue,
		       unsigned short int timePeriod);  

  /*! \brief Sends velocity control command
   * 
   *  Sends velocity control command to the specified motor channel.
   *  The motion controller will drive the motor to the target position
   *  at the maximum posible acceleration
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param cmdValue - Target velocity value, units are:
   *                    <ul>
   *                    <li>position change/second - single potentiometer
   *                    <li>position change/second - dual potentiometer
   *                    <li>pulse/second - encoder
   *                    </ul>  
   */
  void VelocityNonTimeCtr(unsigned char channel,
			  unsigned short int cmdValue); 
 
  /*! \brief Sends PWM control command
   * 
   *  Sends PWM control command to the specified motor channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param cmdValue - Target pulse width value, range is 0 to 32767, and 
   *                    coresponds to the duty cycle og 0 to 100% linearly.
   *                    A pulse width value 16363 means 50% duty cycle puting 
   *                    motor in "Stop" stage. Any value between 16364-32767
   *                    will put motor to run clockwise and any value between
   *                    0-16362 will put the motor to turn counter-clockwise
   *  \param timePeriod - executing time in miliseconds
   */
  void PwmTimeCtr(unsigned char channel,
		  unsigned short int cmdValue,
		  unsigned short int timePeriod);

  /*! \brief Sends PWM control command
   * 
   *  Sends PWM control command to the specified motor channel.
   *
   *  \param channel - 0 for the first motor, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th motor
   *  \param cmdValue - Target pulse width value, range is 0 to 32767, and 
   *                    coresponds to the duty cycle og 0 to 100% linearly.
   *                    A pulse width value 16363 means 50% duty cycle puting 
   *                    motor in "Stop" stage. Any value between 16364-32767
   *                    will put motor to run clockwise and any value between
   *                    0-16362 will put the motor to turn counter-clockwise
   */
  void PwmNonTimeCtr(unsigned char channel,
		     unsigned short int cmdValue); 

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
   *  \param cmd1 - chanel 0 position value
   *  \param cmd2 - chanel 1 position value
   *  \param cmd3 - chanel 2 position value
   *  \param cmd4 - chanel 3 position value
   *  \param cmd5 - chanel 4 position value
   *  \param cmd6 - chanel 5 position value
   *  \param timePeriod - executing time in miliseconds
   */
  void PositionTimeCtrAll(short int cmd1, short int cmd2,
			  short int cmd3, short int cmd4,
			  short int cmd5, short int cmd6,
			  unsigned short int timePeriod);

  /*! \brief Sends position control command to all motors
   * 
   *  Sends position control command to all 6 DC motors channel.
   *  The motion controller will drive the motor to the target position
   *  at the maximum speed
   *  Target position value data range is:
   *                   <ul>
   *                   <li>0-4095 - single potentiometer
   *                   <li>0-4428 - dual potentiometer
   *                   <li>0-32767 - encoder
   *                   </ul> 
   *  Value -32768 (0x8000) means no control
   *
   *  \param cmd1 - chanel 0 position value
   *  \param cmd2 - chanel 1 position value
   *  \param cmd3 - chanel 2 position value
   *  \param cmd4 - chanel 3 position value
   *  \param cmd5 - chanel 4 position value
   *  \param cmd6 - chanel 5 position value
   */
  void PositionNonTimeCtrAll(short int cmd1, short int cmd2,
			     short int cmd3, short int cmd4,
			     short int cmd5, short int cmd6);  

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
   *  \param cmd1 - chanel 0 velocity value
   *  \param cmd2 - chanel 1 velocity value
   *  \param cmd3 - chanel 2 velocity value
   *  \param cmd4 - chanel 3 velocity value
   *  \param cmd5 - chanel 4 velocity value
   *  \param cmd6 - chanel 5 velocity value
   *  \param timePeriod - executing time in miliseconds
   */
  void VelocityTimeCtrAll(unsigned short int cmd1, unsigned short int cmd2,
			  unsigned short int cmd3, unsigned short int cmd4,
			  unsigned short int cmd5, unsigned short int cmd6,
			  unsigned short int timePeriod);

  /*! \brief Sends velocity control command to all motors
   * 
   *  Sends velocity control command to all 6 DC motors channel.
   *  The motion controller will drive the motor to the target position
   *  at the maximum posible acceleration.
   *  Target velocity value data unit is:
   *                   <ul>
   *                   <li>position change/second - single potentiometer
   *                   <li>position change/second - dual potentiometer
   *                   <li>pulse/second - encoder
   *                   </ul> 
   *  Value -32768 (0x8000) means no control
   *
   *  \param cmd1 - chanel 0 velocity value
   *  \param cmd2 - chanel 1 velocity value
   *  \param cmd3 - chanel 2 velocity value
   *  \param cmd4 - chanel 3 velocity value
   *  \param cmd5 - chanel 4 velocity value
   *  \param cmd6 - chanel 5 velocity value
   */
  void VelocityNonTimeCtrAll(unsigned short int cmd1, unsigned short int cmd2,
			     unsigned short int cmd3, unsigned short int cmd4,
			     unsigned short int cmd5, unsigned short int cmd6);
  
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
   *  \param cmd1 - chanel 0 PWM value
   *  \param cmd2 - chanel 1 PWM value
   *  \param cmd3 - chanel 2 PWM value
   *  \param cmd4 - chanel 3 PWM value
   *  \param cmd5 - chanel 4 PWM value
   *  \param cmd6 - chanel 5 PWM value
   *  \param timePeriod - executing time in miliseconds
   */
  void PwmTimeCtrAll(short int cmd1, short int cmd2,
		     short int cmd3, short int cmd4,
		     short int cmd5, short int cmd6,
		     unsigned short int timePeriod);  

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
   *  \param cmd1 - chanel 0 PWM value
   *  \param cmd2 - chanel 1 PWM value
   *  \param cmd3 - chanel 2 PWM value
   *  \param cmd4 - chanel 3 PWM value
   *  \param cmd5 - chanel 4 PWM value
   *  \param cmd6 - chanel 5 PWM value
   */
  void PwmNonTimeCtrAll(short int cmd1, short int cmd2,
			short int cmd3, short int cmd4,
			short int cmd5, short int cmd6);

  /*! \brief Enables specified servo
   *
   *  Enables the specified servo motorl chontrol channel
   *
   *  \param channel - 0 for the first servo, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th servo
   */ 
  void ServoEnable(unsigned char channel);

  /*! \brief Enables all servos
   *
   *  Enables all servo motorls chontrol channels
   *
   */ 
  void ServoEnableAll();

  /*! \brief Disable specified servo
   *
   *  Disable the specified servo motors chontrol channel
   *
   *  \param channel - 0 for the first servo, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th servo
   */ 
  void ServoDisable(unsigned char channel);

  /*! \brief Disable all servos
   *
   *  Disable all servo motors chontrol channels
   *
   */ 
  void ServoDisableAll();

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
  void ServoTimeCtr(unsigned char channel, 
	       unsigned short int cmdValue,
	       unsigned short int timePeriod);

  /*! \brief Sends position control command to the servo
   * 
   *  Sends position control command to the specified servo motor control
   *  channel. The motion controller will drive the servo motor to the target 
   *  position at the maximum speed.
   *
   *  \param channel - 0 for the first servo, 1 for the second, and so
   *                   on up to 5 that is related to the 6-th servo
   *  \param cmdValue - Target velocity value, unit is
   *                    (Pulse width in milisecond)*2250
   *                    usualy 1ms stands for 0, 1.5ms for 90 and 2 for 180 
   *                    degree
   */
  void ServoNonTimeCtr(unsigned char channel, 
		  unsigned short int cmdValue);

  /*! \brief Sends position control command to all servos
   * 
   *  Sends position control command to all 6 servo motors channel.
   *  Target velocity value, unit is (Pulse width in milisecond)*2250
   *  usualy 1ms stands for 0, 1.5ms for 90 and 2 for 180 degree
   *  Value -32768 (0x8000) means no control
   *
   *  \param cmd1 - chanel 0 position value
   *  \param cmd2 - chanel 1 position value
   *  \param cmd3 - chanel 2 position value
   *  \param cmd4 - chanel 3 position value
   *  \param cmd5 - chanel 4 position value
   *  \param cmd6 - chanel 5 position value
   *  \param timePeriod - executing time in miliseconds
   */
  void ServoTimeCtrAll(short int cmd1,
		  short int cmd2, short int cmd3,
		  short int cmd4, short int cmd5,
		  short int cmd6,
		  unsigned short int timePeriod);

  /*! \brief Sends position control command to all servos
   * 
   *  Sends position control command to all 6 servo motors channel.
   *  Target velocity value, unit is (Pulse width in milisecond)*2250
   *  usualy 1ms stands for 0, 1.5ms for 90 and 2 for 180 degree
   *  Value -32768 (0x8000) means no control. The motion controller will 
   *  drive the servo motor to the target position at the maximum speed.
   *
   *  \param cmd1 - chanel 0 position value
   *  \param cmd2 - chanel 1 position value
   *  \param cmd3 - chanel 2 position value
   *  \param cmd4 - chanel 3 position value
   *  \param cmd5 - chanel 4 position value
   *  \param cmd6 - chanel 5 position value
   */
  void ServoNonTimeCtrAll(short int cmd1,
		     short int cmd2, short int cmd3,
		     short int cmd4, short int cmd5,
		     short int cmd6);
 private:
  /*! \brief Field containing data from motor sensors 
   */
  struct MotorDataField {
    unsigned short int Pot[6];
    unsigned short int Current[6];
    unsigned short int Pulse1;
    unsigned short int Speed1;
    unsigned short int Pulse2;
    unsigned short int Speed2;
    unsigned char Dir;
  } MotorData;
}; //class DrRobotMotors_t

/*! \class DrRobotCustomData_t
 *  \brief This class model a board allwing acces to custom data sensors
 *  
 *  This class contains a methods that allow to refresh data geather form 
 *  the robot custom sensors. This class alow also to control LCD and robot
 *  colision avoidance system.
 */
class DrRobotCustomData_t: public DrRobotDevice_t  {
 public:
  /*! \brief Parametric constructor
   *
   *  Construct a device by defining its ID and parent board
   *  \param dID - identification number of the device (look at the protocol 
   *               description). We assume that device ID is an ID on which
   *               a apropreate are send to.
   *  \param board - pointer to the board that this device is conected to.
   *  \sa DrRobotDevice_t::DrRobotDevice_t()
   */
  DrRobotCustomData_t(unsigned char dID, DrRobotBoard_t *board);

  virtual ~DrRobotCustomData_t(){};

  /*! \brief Refresh the data related to this device
   *
   *  Refresh the data that are stored in this class and that are related with
   *  that device. As a parameter it should obtain a whole data package related
   *  with the data.
   *  \param dataBuffer - pointer to the table of chars consisting a whole data 
   *                      package directed to this specific device.
   *  \param dataLength - lenght of the data.
   */
  void RefreshData(unsigned char *dataBuffer,unsigned long dataLength);

  /*! \brief Sensor data obtaining control function
   *  
   *  Method that contorl sending from the robot information about sensoes state
   *  \param PacketNumber - describes type of geathering data
   *                        <ul>
   *                          <li> PacketNumber =  0 - Stop sendig the 
   *                                                   sensor data packets
   *                          <li> PacketNumber = -1 - Send semsor data 
   *                                                   continously ubtil being
   *                                                   asked to stop
   *                          <li> PacketNumber >  0 - Send exacpty 
   *                                                   PacketNumber sensor data
   *                                                   and then stop sending
   *                        </ul>
   */
  void SystemSensorSending(int PacketNumber);
  
  /*! \brief Start sending data continously
   * 
   *  Start sending data continously
   */
  void EnableSensorSending(void);
 
  /*! \brief Stop sending data
   * 
   *  Stop sending data
   */ 
  void DisableSensorSending(void);
  
  /*! \brief Returns the sampling value of custom A/D inputs
   *
   *  Returns the sampling value of custom analog to digital input
   *  signals. By default AD1-AD3 are used as the imputs of power supply
   *  voltage monitors for DSP circuts, DC motors and servo motors.
   *
   *  \param channel - 0 for the first custom A/D, 1 for the second, and so
   *                   on up to 7 that is related to the 8-th custom A/D channel
   *  \return Return data is the raw value of the analog to digital converter
   *          indicating the input voltage levels. The data range is between 
   *          0 and 4095. The voltage U can be calculated from the following equation:
   *          \f[
   *              U = f(U)*3.0/4095
   *          \f]
   *  
   */
  unsigned short int GetAD(unsigned char channel);

  /*! \brief Returns a value from custom digital inputs
   *
   *  Returns a value with lower 8-bits coresponding to the 8-channel custom
   *  digital inputs
   * 
   *  \return Returns a value with lower 8-bits coresponding to the 8-channel
   *          custom digital inputs. MSB of lower byte represents chanell 8
   *          and LSB of lower byte represents channel 1
   */
  unsigned char GetDIN(void);

  /*! \brief Sets the custom digital outputs
   *
   *  Sets the 8-channel custom digital outputs.
   * 
   *  \param value - MSB of lower byte represents chanell 8
   *          and LSB of lower byte represents channel 1
   */
  void SetDIN(unsigned char value);
  
  /*! \brief Display PGM image on the LCD
   *
   *  Display the image data in PGM format on the graphic LCD
   *
   *  \param bmpFileName - pointer to the table of char contaning
   *  
   *  \pre The image has to be monohromatic with dimention of 128 by 64
   *       pixels.
   */
  unsigned char DisplayPGM(char* bmpFileName);
  void EnableBumperProtection(void);
  void DisableBumperProtection(void);
 private:
  /*! \brief Field containing data from custom sensors
   */
  struct CustomDataField {
    unsigned short int Channel[8];
    unsigned char Digital;
  } CustomData;
  
  /*! \brief Read PGM file
   */
  unsigned char ReadPGM(FILE *p, int image_pgm[64][128], int *dimX, 
			int *dimY, int *grayness);
}; //class DrRobotCustomData_t

/*! \class DrRobotPowerControler_t
 *  \brief This class model a board allwing acces to custom data sensors
 *  
 *  This class contains a methods that allow to refresh data geather form 
 *  the robot custom sensors. This class alow also to control LCD and robot
 *  colision avoidance system.
 */
class DrRobotPowerControler_t: public DrRobotDevice_t  {
 public:
  /*! \brief Parametric constructor
   *
   *  Construct a device by defining its ID and parent board
   *  \param dID - identification number of the device (look at the protocol 
   *               description). We assume that device ID is an ID on which
   *               a apropreate are send to.
   *  \param board - pointer to the board that this device is conected to.
   *  \sa DrRobotDevice_t::DrRobotDevice_t()
   */
  DrRobotPowerControler_t(unsigned char dID, DrRobotBoard_t *board);

  virtual ~DrRobotPowerControler_t(){};

  /*! \brief Refresh the data related to this device
   *
   *  Refresh the data that are stored in this class and that are related with
   *  that device. As a parameter it should obtain a whole data package related
   *  with the data.
   *  \param dataBuffer - pointer to the table of chars consisting a whole data 
   *                      package directed to this specific device.
   *  \param dataLength - lenght of the data.
   */
  void RefreshData(unsigned char *dataBuffer,unsigned long dataLength);

  /*! \brief Sensor data obtaining control function
   *  
   *  Method that contorl sending from the robot information about sensoes state
   *  \param PacketNumber - describes type of geathering data
   *                        <ul>
   *                          <li> PacketNumber =  0 - Stop sendig the 
   *                                                   sensor data packets
   *                          <li> PacketNumber = -1 - Send semsor data 
   *                                                   continously ubtil being
   *                                                   asked to stop
   *                          <li> PacketNumber >  0 - Send exacpty 
   *                                                   PacketNumber sensor data
   *                                                   and then stop sending
   *                        </ul>
   */
  void SystemSensorSending(int PacketNumber);
  
  /*! \brief Start sending data continously
   * 
   *  Start sending data continously
   */
  void EnableSensorSending(void);
 
  /*! \brief Stop sending data
   * 
   *  Stop sending data
   */ 
  void DisableSensorSending(void);
  
  /*! \brief Returns the batery voltage AD value
   *
   *  Returns the sampling value of custom analog to digital input
   *  signals coresponding to the batery voltage
   *
   *  \param channel - 0 for the first custom A/D, 1 for the second
   *                
   *  \return Return data is the raw value of the analog to digital converter
   *          indicating the input voltage levels. The data range is between 
   *          0 and 4095.
   *  
   */
  unsigned short int GetBatVolt(unsigned char channel);

 /*! \brief Returns the batery temperature AD value
   *
   *  Returns the sampling value of custom analog to digital input
   *  signals coresponding to the batery temperature
   *
   *  \param channel - 0 for the first custom A/D, 1 for the second
   *                
   *  \return Return data is the raw value of the analog to digital converter
   *          indicating the input voltage levels. The data range is between 
   *          0 and 4095.
   *  
   */
  unsigned short int GetBatTemp(unsigned char channel);

 /*! \brief Returns the DCIN voltage AD value
   *
   *  Returns the sampling value of custom analog to digital input
   *  signals coresponding to the DCIN voltage 
   *
   *  \return Return data is the raw value of the analog to digital converter
   *          indicating the input voltage levels. The data range is between 
   *          0 and 4095.
   *  
   */
  unsigned short int GetDCINVolt();

 /*! \brief Returns the charge status flag
   *
   *  Returns the power status flag coresponding to the first battery
   *
   *  \return Returns masked bit values 0x04 - charging, 0x08 - Power fail, 
   *          0x10 - DCIN comparator output, 0x20 - low power, 0x40 - Fault,
   *  
   */
  unsigned char GetStatus();

 /*! \brief Returns the reference voltage  AD value
   *
   *  Returns the sampling value of custom analog to digital input
   *  signals coresponding to the reference voltage
   *
   *  \return Return data is the raw value of the analog to digital converter
   *          indicating the input voltage levels. The data range is between 
   *          0 and 4095.
   *  
   */
  unsigned short int GetRefVolt();

 /*! \brief Returns the power status flag
   *
   *  Returns the power status flag coresponding to the first battery
   *
   *  \return Returns masked bit values 0x20 - Powered by DCIN, 
   *          0x40 - powered by battery 2,
   *          0x80 - powered by battery 1
   *  
   */
  unsigned char GetPowerPath();

 /*! \brief Returns the charge status flag
   *
   *  Returns the power status flag coresponding to the first battery
   *
   *  \return 0x40 - charge battery 2,
   *          0x80 - charge battery 1
   *  
   */
  unsigned char GetChargePath();

 private:
  /*! \brief Field containing data from custom sensors
   */
  struct PowerControlerField {
    unsigned short Bat1Volt;
    unsigned short Bat1Temp;
    unsigned short Bat2Volt;
    unsigned short Bat2Temp;
    unsigned short DCINVolt;
    unsigned char reserved1[6];
    unsigned char Status;
    unsigned char reserved2[2];
    unsigned short RefVolt;
    unsigned char reserved3[6];
    unsigned char PowerPath;
    unsigned char reserved4;
    unsigned char ChargePath;
    unsigned char reserved5;
  } PowerControler;
  
}; //class DrRobotPowerControler_t


/*! \class DrRobotCustomData_t
 *  \brief This class models an audio board
 *  
 *  This class contains a methods that allow to refresh data geather form 
 *  the robot audio devices. 
 */
class DrRobotAudio_t: public DrRobotDevice_t  {
 public:
 /*! \brief Parametric constructor
   *
   *  Construct a device by defining its ID and parent board
   *  \param dID - identification number of the device (look at the protocol 
   *               description). We assume that device ID is an ID on which
   *               a apropreate are send to.
   *  \param board - pointer to the board that this device is conected to.
   *  \sa DrRobotDevice_t::DrRobotDevice_t()
   */
  DrRobotAudio_t(unsigned char dID, DrRobotBoard_t *board);
  
  virtual ~DrRobotAudio_t(){};

  /*! \brief Refresh the data related to this device
   *
   *  Refresh the data that are stored in this class and that are related with
   *  that device. As a parameter it should obtain a whole data package related
   *  with the data.
   *  \param dataBuffer - pointer to the table of chars consisting a whole data 
   *                      package directed to this specific device.
   *  \param dataLength - lenght of the data.
   */
  void RefreshData(unsigned char *dataBuffer,unsigned long dataLength);

  /*! \brief Analize the audio data
   *
   *  Analize the bufor lpInData of the length nLen containing the audio data.
   *
   *  \param lpInData - pointer to the table of chars consisting a whole data 
   *                    package directed to this specific device.
   *  \param nLen - lenght of the data.
   */  
  void DealWithAudio(unsigned char * lpInData, int nLen);

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
  void StartRecord(unsigned char VoiceSegment);

  /*! \brief Stop recodring
   *
   *  Stop recording an audio file
   *
   */  
  void StopRecord();

  /*! \brief Start plaing audio file
   *
   *  Start playing audio file
   *
   */    
  void StartAudioPlay();

  /*! \brief Stop plaing audio file
   *
   *  Start playing audio file
   *
   */   
  void StopAudioPlay();

  /*! \brief Send audio file to the robot
   *  
   *  Send audio file to the robot. After sending Start Audio Play command,
   *  host can send Audio to PMB5010.
   *
   *  \param data - pointer to the table with the audio to be played
   *  \param lenght - data length
   */
  void SendAudio(unsigned char *data,unsigned char length);

  /*! \brief Initialize ADPCM 
   *
   *  Initialize ADPCM audio coder/decoder
   */
  void adpcm_init();  
  
  /*! \brief Decode with ADPCM 
   *
   *  Decode with ADPCM audio data.
   *
   *  \param indata - input data
   *  \param outdata - output data
   *  \param len - data lenght
   */ 
  void adpcm_decoder(unsigned char *indata, 
		     short *outdata, int len);

  /*! \brief Code with ADPCM 
   *
   *  Code with ADPCM audio data.
   *
   *  \param indata - input data 
   *  \param outdata - output data
   *  \param len - data length
   */
  void adpcm_coder(short *indata, unsigned char *outdata,
		   int len);  
  
  /*! \brief Save audio to WAV
   *
   *  Save audio to WAV
   *
   *  \param FileName - file name
   *  \param audioBuff - audio data
   *  \param audioBufIndex -audio data lenght
   */
  void SaveWAV(char * FileName, short * audioBuf, long audioBufIndex); 
  
  /*! \brief Read audio from WAV
   *
   *  Read audio from WAV
   *
   *  \param lpFilePath - file name
   *  \param VoiceBuffer - pointer to the audio data
   *  \param VoiceBufferSize - number of raw audio data samples
   *  \param VoiceLength - value sescribing audio data length
   */
  void ReadWAV(char* lpFilePath,short** VoiceBuffer,
	       long* VoiceBufferSize,int *VoiceLength);  
  
  /*! \brief Send voice buffer to the robot after prepering it
   *
   *  Send voice buffer to the robot after prepering it
   * 
   *  \param VoiceBuffer - pointer to the audio data 
   *  \param VoiceBufferSize - audio data length
   */
  void SendVoiceBuffer(short* VoiceBuffer,long VoiceBufferSize);

  /*! \brief Returns the audio buffer
   *
   * Returns the audio buffer
   */
  short * GetAudioBuffer();
  /*! \brief Returns the audio buffer length
   *
   *  Returns the audio buffer length
   */
  long GetAudioBufferLength();

  /*! \brief Clear the audio buffer
   *
   *  Clear the audio buffer
   */
  void ClearAudioBuffer();

  void SlowDownAudio();  
private:
  struct adpcm_state {
    short valprev; /* Previous output value */
    char index; /* Index into stepsize table */
  } state;    
  short audioBuf[1024000];
  long audioBufIndex;
  short lastAudio[1024000];
  long lastAudioLength;
  short *lpOutData;
  int audioSegment;
  unsigned short int audioDelay;  
}; // class DrRobotAudio_t

#define MaxImageSize		18000

/*! \class DrRobotCustomData_t
 *  \brief This class models an audio board
 *  
 *  This class contains a methods that allow to refresh data geather form 
 *  the robot audio devices. 
 */
class DrRobotVideo_t: public DrRobotDevice_t  {
 public:
   /*! \brief Parametric constructor
   *
   *  Construct a device by defining its ID and parent board
   *  \param dID - identification number of the device (look at the protocol 
   *               description). We assume that device ID is an ID on which
   *               a apropreate are send to.
   *  \param board - pointer to the board that this device is conected to.
   *  \sa DrRobotDevice_t::DrRobotDevice_t()
   */
  DrRobotVideo_t(unsigned char dID, DrRobotBoard_t *board);


  virtual ~DrRobotVideo_t(){};
    

  /*! \brief Refresh the data related to this device
   *
   *  Refresh the data that are stored in this class and that are related with
   *  that device. As a parameter it should obtain a whole data package related
   *  with the data.
   *  \param dataBuffer - pointer to the table of chars consisting a whole data 
   *                      package directed to this specific device.
   *  \param dataLength - lenght of the data.
   */
  void RefreshData(unsigned char *dataBuffer,unsigned long dataLength);

  /*! \brief Analize the video data
   *
   *  Analize the videoo data.
   *
   */  
  void DealWithVideo();

  /*! \brief Take a photo
   *
   *  Send a command to a robot to take a photo
   */
  void TakePhoto(void); 
  
  /*! \brief Returns an OpenCV image
   *
   *  Returns an OpenCV image form the camera
   *
   *  \return Return the pointer to the IplImage (OpenCV image)
   */
  //IplImage* getIplImage();
  unsigned char rgbArr[100000];
  unsigned int width;
  unsigned int height;
private:
  unsigned char imageBuffer[MaxImageSize+1]; 

  //IplImage* img;
  long imageBufferPtr;
  int takingPhotoFlag;
  int Last_Line_Num;
  int Line_Num_Wrong;
}; // class DrRobotVideo_t


/*! \brief Function to analize the data incoming from robot
 *
 *  Function to analize the data incoming from robot. It should work
 *  as athread generated by the Listener_t
 *
 *  \param ptr - pointer to the board related with this function
 *
 *  \sa Listener_t
 */
void *data_analizer(void *ptr);

/*! \brief Function to listen for the data incoming from robot
 *
 *  Function to listen for the data incoming from robot. It should work
 *  as athread generated by the Listener_t
 *
 *  \param ptr - pointer to the board related with this function
 *
 *  \sa Listener_t
 */
void *listener_function(void *ptr);

/*! \class Listener_t
 *  \brief Class that model a listener  
 *
 *  Class that model a listener related with the board device. The listener
 *  is responsible to geather data incoming form the specific board and to
 *  analize it and pass it to the proper device.
 *
 */
class Listener_t{
public:
  /*! \brief Parametric constructor
   *
   *  Construct a listener that is related with specific board and that has
   *   a limited buffer size
   *
   *  \param board - pointer to the board that this listener is related to.
   *  \param totalsize - maximal size fo the listener input buffer
   */
  Listener_t(DrRobotBoard_t *board,unsigned long int totalsize);

  /*! \brief Destructor
   *
   *  Destructor
   */
  virtual ~Listener_t();

  /*! \brief Start listener
   *
   *  Start listener
   */
  void start();

  /*! \brief Stop listener
   *
   *  Stop listener
   */
  void stop();

  /*! \brief Check wheather listener is active
   *
   *  Check wheather listener is active
   */
  bool isActive(){return active;};
private:
  unsigned char* data; //!< Buffer for the incoming data
  unsigned long int dl; //!< Actual data length
  unsigned long int totalsize; //!< Maxium data capacity
  DrRobotBoard_t *board; //!< Pointer to the board related with the listener
  bool active; //!< Activation flag
  pthread_t listenerThread; //!< Listening function thread
  pthread_t analizerThread; //!< Data analizing function thread
  pthread_mutex_t mutex; //!< Lock securing acces to the data buffer
  friend void *data_analizer(void *ptr); //!< Data analizing function
  friend void *listener_function(void *ptr); //!< Listening function
  bool DealWithPacket(unsigned char* data, int nLen); //!< Procesing data 
};


/*! \brief Heart beat board function
 *
 *  Function to generate cyclic ping to the robot board to keep it active
 *
 *  \sa DrRobotBoard_t
 */
void *heartbeat_function(void *ptr);


unsigned char DeviceID2NR(unsigned char);

/*! \class DrRobotBoard_t
 *  \brief Class models a robot board with devices
 *
 *  Class models a robot board with devices.
 *
 */
class DrRobotBoard_t {
public:
  /*! \brief  Parametric constructor
   *  
   *  Parametric constructor
   */ 
  DrRobotBoard_t(unsigned char robotID,float T);
  
  /*! \brief Destructor
   *
   *  Destructor
   */
  virtual ~DrRobotBoard_t();
  
  /*! \brief robot ID
   *
   *  Robot ID - board indication number
   */
  unsigned char rID;

  /*! \brief Container of the devices
   *
   *  Container of the pointers to the devices 
   */
  DrRobotDevice_t** device;

  unsigned char deviceSize;
  
  /*! \brief Listener
   *
   *  Object listening for the data incoming to this board
   *  \sa Listener_t
   */
  Listener_t listener;

  /*! \brief Send data to the robot
   *
   *  Send data to the robot. Low lever funcion. User should not use it unless 
   *  he/she knows what is dooing
   *
   *  \param dID - device ID, code of the device that the data is direced to.
   *               See the protocol manual.
   *  \param dataLength - data lenght
   *  \param data - pointer to the data
   *
   */
  void SendData(unsigned char dID,unsigned short dataLength, 
		unsigned char* data);

  /*! \brief Send ping to the robot
   *
   *  Send ping to the robot. Low lever funcion. User should not use it unless 
   *  he/she knows what is dooing
   *
   */
  void SendPing(void);

  /*! \brief Send acknolegement to the robot
   *
   *  Send acknolagement to the robot. Low lever funcion. User should not use it unless 
   *  he/she knows what is dooing
   *
   */
  void SendAck(void);

  /*! \brief Start the board
   *
   *  Start the board
   */
  void start();

  /*! \brief Stop the board
   *
   *  Stop the board
   */
  void stop();

  /*! \brief Check wheather board is active
   *
   *  Check wheather board is active
   */
  bool isActive(){return active;};

  /*! \brief Connect to the robot
   *
   *  Connect to the robot
   *  
   *  \param servername - name or IP of the server
   *  \param portno - port number
   */
  bool Connect(const char* servername,unsigned int portno);

  /*! \brief Disconnect from the robot
   *
   *  Disconnect from the robot
   */
  bool Disconnect();

  /*! \brief Check wheather board is conected
   *
   *  Check wheather board is conected
   */
  bool isConnected();

  /*! \brief Calculate CRC
   *
   *  Calculate checksum
   *
   *  \param lpBuffer - pointer to the buffer with data
   *  \param nSize - data length
   *
   */
  unsigned char CalculateCRC(unsigned char *lpBuffer, unsigned char nSize);
  
  /*! \brief Returns socket related with the board
   *
   *  Returns socket related with the board
   */
  int getSockfd(){return sockfd;};
private:
  int sockfd; //!< socket related with the specific board
  pthread_t heartBeat; //!< Function to generate cyclic ping thread
  unsigned char seq; //!< sequence number of the sending data
  bool active; //!< flag of being active
  float heartbeat_T; //!< cycling ping time period constant
  friend void *heartbeat_function(void *ptr); //!< Cyclic ping function
};
  
/*! \class DrRobot_t
 *  \brief Models a robot
 * 
 *  Models a robot
 */

class DrRobot_t {
public:
    DrRobot_t(){};
    virtual ~DrRobot_t(){};
    
  /*! \brief A container geathering boards
   *
   *  A container geathering boards
   */
  DrRobotBoard_t** board;

  unsigned char boardSize;
};

#pragma pack(pop)   /* pop current alignment from stack */
  
#endif // DRROBOTSDK_H
