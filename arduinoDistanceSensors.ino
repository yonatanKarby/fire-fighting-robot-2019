
#include <VL53L0X.h>
#include <Wire.h>

VL53L0X sensor1;
VL53L0X sensor2;

#define SEN1 7
#define SEN2 8

#define PINO_H 13
#define PINO_L 12

#define MAXSENSORREAD 700
#define WALLMAXDISTANCE 300

//byte sensorNum = 1;
bool isInvert = false;
int sensorCurrentReadIndex = 0; // holds the place where the next read is going to be placed into the reads array
float sensorFilterReads[11];
bool isInSkipMode =false;

float sensorLastDistance;
float sensorCurrentDistance;
float sensorLastSpeed;
float sensorCurrentSpeed;
int usingSensor = -1;

int numberOfAvgs = sizeof(sensorFilterReads)/sizeof(float);
  
int canTurnLeft = 0;
int canTurnRight = 0;

bool readingTurn = false;
bool readingMove = false;

#define SKIP_RIGHT 1
#define SKIP_LEFT 2
#define CMD_SENSOR1 49
#define CMD_SENSOR2 50
#define STARTcommand_turning 116
#define STARTcommand_moving 109


#define PINO_StopTurn 6
#define PINO_leftStepper 4
#define PINO_rightStepper 3
#define PINI_sensorLeft 5

#define BESTDISTACE 7

float currentDistanceForFixer = 0;
float accuracy = 0.4;
int num =0 ;

void setup() {
  
  Serial.begin(9600);
  
  Wire.begin();

  pinMode(6, OUTPUT);

  pinMode(PINO_H, OUTPUT);
  pinMode(PINO_L, OUTPUT);

  pinMode(PINO_rightStepper, OUTPUT);
  pinMode(PINO_leftStepper, OUTPUT);
  digitalWrite(PINO_rightStepper, LOW);
  digitalWrite(PINO_leftStepper, LOW);
  
  pinMode(PINO_StopTurn, OUTPUT);

  digitalWrite(PINO_StopTurn, LOW);
  
  pinMode(SEN1,OUTPUT);
  pinMode(SEN2,OUTPUT); 
  
  digitalWrite(SEN1 , LOW);
  digitalWrite(SEN2 , LOW);
    
  digitalWrite(SEN1 , HIGH);
  sensor1.init();
  sensor1.setAddress(0x1);
  delay(50);
  sensor1.setTimeout(100);
  sensor1.setMeasurementTimingBudget(5);
    
  digitalWrite(SEN2 , HIGH);
  sensor2.init();
  sensor2.setAddress(0x2);
  delay(50);
  sensor2.setTimeout(100);
  sensor2.setMeasurementTimingBudget(5);

}
void loop(){

  if(Serial.available())
  {
    int Read = Serial.read();

    switch(Read)
    {
      case 't':
        command_turning();
        break;
      case 'm':
        command_moving();
        break;
      case 'p':
        command_openingChecker();
        break;
    }
  }
}


/*
 * Initializes the reading filter array
 */
void sensor_restartReadings()
{
  sensorCurrentReadIndex=0;
}


/*
 * Read from the sensor
 * If the sensor filtered data is not ready the function will
 * return -1 
 */
///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

bool ctrlCmd_isInvert()
{
  while(!Serial.available())
  {
  }  
  return Serial.read() == 'i';
}

void ctrlCmd_updateSensor()
{
  while(!Serial.available())
  {
  }
  usingSensor = Serial.read();
}


////////////////////////////////////////////////////////////////////////
int readSensor(bool ignoreMaxRead)
{
  float newReads;
  switch(usingSensor)
  {
    case CMD_SENSOR1:
      newReads = sensor1.readRangeSingleMillimeters();
      break;
    case CMD_SENSOR2:
      newReads = sensor2.readRangeSingleMillimeters();
      break;  
  }
  if(newReads > MAXSENSORREAD && !ignoreMaxRead)
  {
     sensor_restartReadings();
     return -1;
  }
  
  currentDistanceForFixer = currentDistanceForFixer <=0 ? sensorCurrentDistance : (currentDistanceForFixer*0.2)+(sensorCurrentDistance*0.8);
  
  sensorLastDistance = sensorCurrentDistance;
  sensorCurrentReadIndex++;
  sensorFilterReads[sensorCurrentReadIndex%numberOfAvgs] = newReads;
  
  if(sensorCurrentReadIndex<numberOfAvgs+1) return -1; // first we need to fill in the reads filter buffer
  
  float avg = getAvg();
  if(ignoreMaxRead)
  {
    float mostdistant = getDiviation(avg);
    sensorCurrentDistance = getNewAvg(mostdistant, avg);
  }
  else
    sensorCurrentDistance = avg;
  
  float newspeed = sensorCurrentDistance - sensorLastDistance;
  sensorLastSpeed = sensorCurrentSpeed;
  sensorCurrentSpeed = newspeed;
  return sensorCurrentDistance;
}

float getDiviation(float avg)
{
  float divi=avg;
  for(int j=0; j<numberOfAvgs;j++)
  {
    if(abs(sensorFilterReads[j]-avg) > abs(divi-avg))
      divi = sensorFilterReads[j];
  }
  return divi;  
  
}

float getNewAvg(float divi,float avg){
  
  float sum=0;
  for(int j=0; j<numberOfAvgs;j++)
    sum += sensorFilterReads[j] == divi ? avg: sensorFilterReads[j];
  return (float)sum/numberOfAvgs;  
}

void skip(int skiptype)
{
  String msg;
  switch(skiptype)
  {
    case SKIP_RIGHT:
      msg = isInvert ? "2": "1";
      digitalWrite(PINO_rightStepper, HIGH);
      digitalWrite(PINO_leftStepper, LOW);
      break;
    case SKIP_LEFT:
      msg = isInvert? "1" : "2";
      digitalWrite(PINO_rightStepper, LOW);
      digitalWrite(PINO_leftStepper, HIGH);
      break;
  }

  Serial.print(msg);  
}

void command_openingChecker()
{
  ctrlCmd_updateSensor();
  digitalWrite(PINO_H, LOW);
  digitalWrite(PINO_L, LOW);
  if(isWallOpen())
  {
    digitalWrite(PINO_H, HIGH);
    Serial.print("h");
  }
  else
  {
    digitalWrite(PINO_L, HIGH);
    Serial.print("l");
  }
}

bool isWallOpen()
{
  sensor_restartReadings();
  for(int i=0; i<3*numberOfAvgs;i++) // run until valid response or just quit after 3 times the filter window size
  {
    readSensor(true);
  }
  return sensorCurrentDistance <= 0 || sensorCurrentDistance >= WALLMAXDISTANCE;    
}


void command_moving()
{
  ctrlCmd_updateSensor();
  isInvert = ctrlCmd_isInvert();
  if(usingSensor == CMD_SENSOR2)
    isInvert = !isInvert;
  sensor_restartReadings();
  currentDistanceForFixer = 0;
  while(true)
  {
    if(Serial.available())
    {
      int newRead = Serial.read();
      switch(newRead)
      {
        case 'a':
          isInSkipMode = false;
          digitalWrite(PINO_rightStepper, LOW);
          digitalWrite(PINO_leftStepper, LOW);
          
          break;
        case 'd':
          Serial.write((byte)(((int)(currentDistanceForFixer/10))&0xff));
          return;       
      }
    }
    
    if(readSensor(false)<0) 
      continue;
 
    if(sensorCurrentSpeed > accuracy && !isInSkipMode)
    {
      skip(SKIP_RIGHT);
      isInSkipMode=true;
    }
      
    if(sensorCurrentSpeed < -accuracy && !isInSkipMode)
    {
      skip(SKIP_LEFT);
      isInSkipMode=true;
    }
  } 
}

void command_turning()
{
  ctrlCmd_updateSensor();
  sensor_restartReadings();
  byte turnOverWallCounter = 0;

  while(true)
  {
    if(readSensor(false)<0)
      continue;
      
    if(sensorCurrentDistance-sensorLastDistance>0.4)
    {
      turnOverWallCounter++;
    }
    else
    {
      turnOverWallCounter = 0;
    }
    if(turnOverWallCounter >= 6)
    {      
      Serial.print("o");
      return;
    }
  }
}




float getAvg(){
  
  float sum=0;
  for(int j=0; j<numberOfAvgs;j++)
    sum += sensorFilterReads[j];
  return (float)sum/numberOfAvgs;  
}
