
#include <Servo.h>
float prevSensors[2]; 

#define KILLSWITCH 46
#define LABVIEW Serial
#define SENSORARDUINO Serial1
#define BESTDISTANCE 8
#define LED 33
#define L_skipRight 1
#define L_skipLeft 2
#define PINO_L 8
#define PINO_H 9
//stepper 1 pins to driver
#define PINO_StepperPuls1 2
#define PINO_StepperDir1 3
#define PINO_StepperEna1 4
// arduino comunications gose to arduinoDistance
#define PINO_Ackmessege 5 //output
//output arduino controler gose to arduinoDistance
//stepper 2 pins to driver
#define PINO_Puls2 11
#define PINO_Dir2 12
#define PINO_pinEna2 13
#define PINO_WallOpenChecker 14
#define PINO_MOTOR 22
#define PINO_PROPELOR 23
#define SOUND A0
#define UVPIN A1

#define UVLIMIT 300
#define TURNDELAY_SERVO 50

#define SENSOR1 49
#define SENSOR2 50
#define NOTURN '-'
#define POSATIVE90DEGREETURN 'r'
#define NEGATIVE90DEGREETURN 'l'

int skipLeftTicks = 0;
int skipRightTicks = 0;
int correctionDeg = 0;


bool starting = true;
//others Globals
double circumference = 34.34;
int gearStepper = 7;
int gearDrive = 14;
#define TURNINGRADIUS 11.4
#define twoPIE  6.283185
int numberOfPulses = 0;
double gearRatio = gearDrive/gearStepper;
double cm_to_ticks = (circumference/(200.0*gearRatio));
float delaysfactor[] = {1,0.999847695,0.999390827,0.998629535,0.99756405,0.996194698,0.994521895,0.992546152,0.990268069,0.987688341,0.984807753,0.981627183,0.978147601,0.974370065,0.970295726,0.965925826,0.961261696,0.956304756,0.951056516,0.945518576,0.939692621,0.933580426,0.927183855,0.920504853,0.913545458,0.906307787,0.898794046,0.891006524,0.882947593,0.874619707,0.866025404,0.857167301,0.848048096,0.838670568,0.829037573,0.819152044,0.809016994,0.79863551,0.788010754,0.777145961,0.766044443,0.75470958,0.743144825,0.731353702,0.7193398,0.707106781,0.69465837,0.68199836,0.669130606,0.656059029,0.64278761,0.629320391,0.615661475,0.601815023,0.587785252,0.573576436,0.559192903,0.544639035,0.529919264,0.515038075,0.5,0.48480962,0.469471563,0.4539905,0.438371147,0.422618262,0.406736643,0.390731128,0.374606593,0.35836795,0.342020143,0.325568154,0.309016994,0.292371705,0.275637356,0.258819045,0.241921896,0.224951054,0.207911691,0.190808995,0.173648178,0.156434465,0.139173101,0.121869343,0.104528463,0.087155743,0.069756474,0.052335956,0.034899497,0.017452406,6.12574E-17,-0.017452406,-0.034899497,-0.052335956,-0.069756474,-0.087155743,-0.104528463,-0.121869343,-0.139173101,-0.156434465,-0.173648178,-0.190808995,-0.207911691,-0.224951054,-0.241921896,-0.258819045,-0.275637356,-0.292371705,-0.309016994,-0.325568154,-0.342020143,-0.35836795,-0.374606593,-0.390731128,-0.406736643,-0.422618262,-0.438371147,-0.4539905,-0.469471563,-0.48480962,-0.5,-0.515038075,-0.529919264,-0.544639035,-0.559192903,-0.573576436,-0.587785252,-0.601815023,-0.615661475,-0.629320391,-0.64278761,-0.656059029,-0.669130606,-0.68199836,-0.69465837,-0.707106781,-0.7193398,-0.731353702,-0.743144825,-0.75470958,-0.766044443,-0.777145961,-0.788010754,-0.79863551,-0.809016994,-0.819152044,-0.829037573,-0.838670568,-0.848048096,-0.857167301,-0.866025404,-0.874619707,-0.882947593,-0.891006524,-0.898794046,-0.906307787,-0.913545458,-0.920504853,-0.927183855,-0.933580426,-0.939692621,-0.945518576,-0.951056516,-0.956304756,-0.961261696,-0.965925826,-0.970295726,-0.974370065,-0.978147601,-0.981627183,-0.984807753,-0.987688341,-0.990268069,-0.992546152,-0.994521895,-0.996194698,-0.99756405,-0.998629535,-0.999390827,-0.999847695,-1};
int delaysMaxLength = sizeof(delaysfactor)/sizeof(float);
int delays[200];
int low = 1000;
int high = 3000;
float tendegreesInTicks;
int tickSpeed;
int fixerDistance = 0;
int fixerSensor = 0;

Servo PINO_UVservo;
#define PINO_UV 44
#define PINO_FAN 50

void motorOn()
{
  digitalWrite(PINO_StepperEna1, HIGH);
  digitalWrite(PINO_pinEna2, HIGH);
}

void motorOff()
{
  delay(200);
  //digitalWrite(PINO_StepperEna1, LOW);
  //digitalWrite(PINO_pinEna2, LOW);
}

void initDelays()
{
  int m = (high-low)/2;
  int b = low+m;
  for(int i=0; i<delaysMaxLength; i++)
  {
    delays[i] = (int)((delaysfactor[i]*m)+b);
  }
}

void motorsForward()
{
  digitalWrite(PINO_StepperDir1, HIGH);
  digitalWrite(PINO_Dir2, LOW);
}

void motorsBackwards()
{
  digitalWrite(PINO_StepperDir1, LOW);
  digitalWrite(PINO_Dir2, HIGH);  
}
int convertDistanceToTicks(double distance)
{
  int newDistance = (int)((distance/cm_to_ticks)+0.5);
  return newDistance;
}


void tick(bool showedPINO_StepperPuls1, bool showedPINO_Puls2)
{
  if(showedPINO_Puls2)
    digitalWrite(PINO_Puls2, HIGH);
  if(showedPINO_StepperPuls1)
    digitalWrite(PINO_StepperPuls1, HIGH);
    
  delayMicroseconds(tickSpeed);
  
  digitalWrite(PINO_Puls2, LOW);
  digitalWrite(PINO_StepperPuls1, LOW);
  delayMicroseconds(tickSpeed);
}
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

byte LABVIEWSerialReadOneByte()
{
  while(!LABVIEW.available())
  {
  }
  return LABVIEW.read();  
}


byte SENSORSerialReadOneByte()
{
  while(!SENSORARDUINO.available())
  {
  }
  return SENSORARDUINO.read();  
}

int SerialGetAngle()
{
  while(!LABVIEW.available())
  {
  }
  switch(LABVIEW.read())
  {
    case NOTURN:
      return 0;
    case POSATIVE90DEGREETURN:
      return 93;
    case NEGATIVE90DEGREETURN:
      return -90; 
  }
}
/*
 * logical communication
 */
bool isStopSignal()
{
  if(SENSORARDUINO.available())
    return SENSORARDUINO.read() == 'o';
  return false;
}

void starTurnControl(int sensor)
{
  SENSORARDUINO.flush();
  SENSORARDUINO.print(sensor==SENSOR1?"t1":"t2");
}
//////////////////////
// motion cycle control

void ctrlMtn_sendStartLegCmd(int sensor)
{
  SENSORARDUINO.flush();
  SENSORARDUINO.print(sensor==SENSOR1?"m1":"m2");  
}


int ctrlMtn_getSkipCycleCmd()
{
  if(SENSORARDUINO.available())
  {
    int skipType = SENSORARDUINO.read();
    switch(skipType)
    {
       case '1':
        return L_skipRight;
       case '2':
        return L_skipLeft;    
    }
  }
  return -1;
}

/*
 * Notify the sensor 
 * Completed motions correction cycle
 */
void ctrlMtn_sendSkipCycleEnd()
{
  SENSORARDUINO.print("a");
}

/*
 * All the motion command is completed
 */
void ctrlMtn_sendEndLegCmd()
{
  SENSORARDUINO.print("d");
  int sensordistance = SENSORSerialReadOneByte();
  if(sensordistance>0)
  {
    switch(fixerSensor)
    {
      case SENSOR1:
        fixerDistance = sensordistance - BESTDISTANCE;
      break;
      
      case SENSOR2:
        fixerDistance = BESTDISTANCE - sensordistance;
      break;
    }
    fixerDistance = (int)((float)fixerDistance * 0.6);
  }
  else 
  {
    fixerDistance = 0;
  }
  
}

////////////////////////////////////////////////////
void waitForSound()
{
  while(analogRead(SOUND)>100)
  {
  }
}
////////////////////////////////////////////////////

void setup() {
  LABVIEW.begin(9600);
  SENSORARDUINO.begin(9600);
  
  delay(10);

  pinMode(LED, OUTPUT);
  pinMode(KILLSWITCH, INPUT);

  PINO_UVservo.attach(PINO_UV);
  PINO_UVservo.write(180);
  pinMode(PINO_WallOpenChecker, OUTPUT);

  pinMode(PINO_StepperPuls1, OUTPUT);  
  pinMode(PINO_StepperDir1, OUTPUT);
  pinMode(PINO_StepperEna1, OUTPUT);
  
  pinMode(PINO_Puls2, OUTPUT);
  pinMode(PINO_Dir2, OUTPUT);
  pinMode(PINO_pinEna2, OUTPUT);

  pinMode(PINO_PROPELOR, OUTPUT);

  digitalWrite(PINO_PROPELOR, LOW);
  digitalWrite(PINO_StepperDir1, HIGH);
  digitalWrite(PINO_Dir2, LOW);
  digitalWrite(PINO_StepperEna1, HIGH);
  digitalWrite(PINO_pinEna2, HIGH);

  pinMode(PINO_MOTOR, OUTPUT);
  digitalWrite(PINO_MOTOR, LOW);
  delay(150);

  digitalWrite(LED,HIGH);
  delay(1000);
    digitalWrite(LED,LOW);
  delay(1000);

  initDelays();
  tendegreesInTicks = convertDistanceToTicks(((double)45/360.0)*twoPIE*TURNINGRADIUS); 
  correctionDeg = convertDistanceToTicks(((double)1.5/360.0)*twoPIE*TURNINGRADIUS);
  motorOff();
}
////////////////////fixer!/////////////////////////

void cntrl_fixerAngle(int angle)
{
  switch(angle)
  {
    case -90:
      fixerDistance *= -1;
      break;
  }
}

int checkForFixer(int distance)
{
  return distance+fixerDistance;
}

//////////////////////////////////////////////////////////////
void exec_driveWithSensor(double distance, int sensor)
{
  tickSpeed = 4;
  motorOn();
  motorsForward();
  bool invert = false;
  if(distance<0)
  { 
    motorsBackwards();
    distance = abs(distance);
  }
  int finalPuls = convertDistanceToTicks(distance);  

  int delaysLength = delaysMaxLength;
  
  if(finalPuls < delaysMaxLength)
  {
    delaysLength = finalPuls/2;
  }

  ctrlMtn_sendStartLegCmd(sensor);

  if(invert)
    SENSORARDUINO.print("i");
  else
    SENSORARDUINO.print("n");

  bool Fix = true;
  
  for(int numberOfPulses = 0; numberOfPulses < finalPuls; numberOfPulses++)
  { 
    if(numberOfPulses<delaysLength)
    {
      tickSpeed = delays[numberOfPulses];
      Fix = false;
    }
    else if(numberOfPulses > finalPuls-delaysLength)
    {
      tickSpeed = delays[finalPuls-numberOfPulses];
      Fix = false;
    }
    else
    {
      if(!Fix)
        ctrlMtn_sendSkipCycleEnd(); 

      tickSpeed = delays[delaysLength-1];
      Fix = true;
    }

    bool isincorrection = (skipRightTicks + skipLeftTicks)>0;
    int skipDirection = ctrlMtn_getSkipCycleCmd();

    if(Fix &&!isincorrection)
    {
      switch(skipDirection)
      {
        case L_skipRight:
          skipRightTicks=correctionDeg;
          finalPuls += (correctionDeg/3);
          break;
        case L_skipLeft:
          skipLeftTicks=correctionDeg;
          finalPuls += (correctionDeg/3);
          break;
      }
    }

    tick(skipLeftTicks == 0, skipRightTicks == 0);

    if(skipLeftTicks > 0)
      skipLeftTicks=skipLeftTicks-1;
      
    if(skipRightTicks > 0)
      skipRightTicks=skipRightTicks-1;

    bool isstillincorrection = (skipRightTicks + skipLeftTicks)>0;
    if(isincorrection && !isstillincorrection)
    {
      ctrlMtn_sendSkipCycleEnd(); 
    }
  } 

  ctrlMtn_sendEndLegCmd();
  motorOff();
}

void command_SearchRoomForCandle()
{
  int angle = exec_candleDidection();
  if(angle != -1)
    exec_extinguishFire(angle);
}

void exec_extinguishFire(int angle)
{
  digitalWrite(PINO_PROPELOR, HIGH);
  for(int i=0; i<5; i++)
  {
    if(!exec_movePropelorByDegreesAndResults(angle, angle+10))
      return;
    if(!exec_movePropelorByDegreesAndResults(angle+10, angle-10))
      return;
  }
  digitalWrite(PINO_PROPELOR, LOW);
}

bool isCandleOn()
{
  return analogRead(UVPIN)>UVLIMIT;
}

bool exec_movePropelorByDegreesAndResults(int startingAngle, int endingAngle)
{
  for(int i = startingAngle; i<endingAngle; i++)
  {
    if(isCandleOn())
      delay(TURNDELAY_SERVO);
    else
      return true;
  }
  return false;
}

int exec_candleDidection()
{
  for(int i=0; i<180; i+=5)
  {
    if(analogRead(UVPIN)>UVLIMIT)
      return i;
    delay(TURNDELAY_SERVO);
  }
  return -1;
}

void motorFireOn()
{
  digitalWrite(PINO_MOTOR,HIGH);
  delay(2000);
  digitalWrite(PINO_MOTOR,LOW);
}

void moveDistance(double distance)
{
  tickSpeed = 4;
  motorOn();

  if(distance<0)
  {
    motorsBackwards();
    distance = abs(distance);
  }  
  int finalPuls = convertDistanceToTicks(distance);  

  int delaysLength = delaysMaxLength;
  
  if(finalPuls < delaysMaxLength)
  {
    delaysLength = finalPuls/2;
  }
  
  for(int numberOfPulses = 0; numberOfPulses < finalPuls; numberOfPulses++)
  { 
    if(numberOfPulses<delaysLength)
    {
      tickSpeed = delays[numberOfPulses];
    }
    else if(numberOfPulses > finalPuls-delaysLength)
    {
      tickSpeed = delays[finalPuls-numberOfPulses];
    }
    else
    {
      tickSpeed = delays[delaysLength-1];
    }
    
    tick(true, true);
  }
  motorsForward();
  motorOff();
}


void command_drive(int Direction)
{
  int distance = (int)LABVIEWSerialReadOneByte();
  distance = checkForFixer(distance*Direction);
  moveDistance((double)distance);
  fixerDistance = 0;
}


void command_driveWithSensor(int Direction)
{
  fixerSensor = LABVIEWSerialReadOneByte();
  int distance = LABVIEWSerialReadOneByte();

  distance = checkForFixer(distance*Direction);
  exec_driveWithSensor((double)distance, fixerSensor);
}

void turn(int angle)
{
  tickSpeed = 1000;
  motorOn();
  double distance = ((double)angle/360.0)*twoPIE*TURNINGRADIUS;
  if(distance == 0)
    return;
  setDirection(distance<0);

  distance = abs(distance);
  
  int finalPuls = convertDistanceToTicks(distance);  
   
  for(int numberOfPulses = 0; numberOfPulses < finalPuls; numberOfPulses++)
  {
    tick(true, true);
  }
  
  motorOff();
  digitalWrite(PINO_StepperDir1, HIGH);
  digitalWrite(PINO_Dir2, LOW);
}

int turnTicks(int pinStartingPuls, int finalTicks)
{
  for(; pinStartingPuls < finalTicks; pinStartingPuls++)
  {
    tick(true, true);
  }  
  return finalTicks;
}

void backTenDegrees(int returnTicks, bool turnccw)
{
  setDirection(!turnccw);
  
  for(int rpuls =0; rpuls < returnTicks; rpuls++)
  {
    tick(true, true);
  }
}
void turnAndSensorStop(int currentTick, int finalPuls)
{
  for(; currentTick < finalPuls; currentTick++)
  {
    if(isStopSignal())
      return;
    tick(true, true);
  }
}
void exec_turnWithSensor(int angle, int sensor)
{
  angle = angle<0?angle-20:angle+20;
  tickSpeed = 1000;
  motorOn();
  double distance = ((double)angle/360.0)*twoPIE*TURNINGRADIUS;
  int returnTicks = 0;
  switch(sensor)
  {
    case SENSOR1:
      returnTicks = convertDistanceToTicks((double)((16/360.0)*twoPIE*TURNINGRADIUS));
      break;
    case SENSOR2:
      returnTicks = convertDistanceToTicks((double)((18/360.0)*twoPIE*TURNINGRADIUS));
      break;
  }
  
  bool turnccw = distance<0;
  
  if(distance == 0)
    return;
  setDirection(turnccw);

  distance = abs(distance);
  
  int finalPuls = convertDistanceToTicks(distance);  

  // trun "blind" until the final approach where we use the sensor
  int currentTick = turnTicks(0, finalPuls-tendegreesInTicks-1);

  tickSpeed = 9000; // slow down speed to help the sensor
  
  starTurnControl(sensor); 
  turnAndSensorStop(currentTick, finalPuls+tendegreesInTicks);
  
  backTenDegrees(returnTicks, turnccw); // the sensor will always have a fixed overshot, so go back fixed amount

  motorOff();
  motorsForward();
}

void setDirection(bool turnLeft)
{
  if(turnLeft)
  {
    digitalWrite(PINO_StepperDir1, LOW);
    digitalWrite(PINO_Dir2, LOW);
  }
  else
  {
    digitalWrite(PINO_StepperDir1, HIGH);
    digitalWrite(PINO_Dir2, HIGH);
  }
}

void command_turn()
{
  int angle = SerialGetAngle();
  cntrl_fixerAngle(angle);
  turn(angle);
}

void command_turnWithSensor()
{
  byte sensor = LABVIEWSerialReadOneByte();
  int angle = SerialGetAngle();
  cntrl_fixerAngle(angle);
  exec_turnWithSensor(angle,sensor);
}

void command_isWallOpen()
{
  byte sensor = LABVIEWSerialReadOneByte();
  digitalWrite(PINO_H, LOW);
  digitalWrite(PINO_L, LOW);

  while(true)
  {
    SENSORARDUINO.print(sensor==SENSOR1?"p1":"p2");
    switch(SENSORSerialReadOneByte())
    {
        case 'h':
            LABVIEW.print("h");
            digitalWrite(PINO_H, HIGH);
          return;
  
        case 'l':
            LABVIEW.print("l");  
            digitalWrite(PINO_L, HIGH);
        return;
    }
  }
}

void loop() {
  
  if(starting)
  {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    starting = false;
    waitForSound();
    digitalWrite(LED, HIGH);
  }
  
  if(digitalRead(KILLSWITCH) == HIGH)
    return; 

  if(LABVIEW.available())
  {
    byte input = LABVIEW.read();

    switch(input)
    {
      case 't':
        command_turn();
        break;
      case 's':
        command_turnWithSensor();
        break;
      case 'm':
        command_drive(1);
        LABVIEW.print("d");
        break;
      case 'k':
        command_driveWithSensor(1);
        LABVIEW.print("d");
        break;
      case 'b':
        command_drive(-1);
        LABVIEW.print("d");
        break;
      case 'u':
        command_driveWithSensor(-1);
        LABVIEW.print("d");
        break;
      case 'w':
        command_isWallOpen();
        break;
      case 'r':
        command_SearchRoomForCandle();
        LABVIEW.print("d");
        break;
    }  
  }  
}
