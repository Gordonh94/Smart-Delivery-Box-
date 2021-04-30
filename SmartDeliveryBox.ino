/*********************************************************************************************************************
 * 
 * File Name: SmartDeliveryBox
 * 
 * Description: This is the code used in my final year project. It allows an arduino to commincate with 
 *              a HC-06 module and a fingerprint sensor in order to rotate a motor to an open or close position. 
 *              This code also takes an analog reading from an infrared senor and depending on this reading
 *              will sound the buzzer appropriately 
 *             
 * Programmer: Gordon Hegarty
 * 
 * Date: 26/4/2021
 * 
 *********************************************************************************************************************/

#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>

#define motorPin1 8
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11
#define MotorInterfaceType 8            /*Type 8 is for a 4 wire stepper motor which uses 4 pins of the Arduino. This type of motor was used in this project*/
#define open 1
#define close 0
#define BTDefault 0
#define BTClose 1
#define BTOpen 2

SoftwareSerial hc06(2, 3);              /* Arduino pin 2 (TXD of HC06), pin 3 (RXD of HC06)*/
SoftwareSerial fingerSensor(4, 5);      /*Arduino pin 4 to white wire(Receive of fingerprint), pin 5 to green wire(Transmit of fingerprint)*/
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&fingerSensor);  /*Sets up sensore with a stream for serial communication*/
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4); /*Type of motor being used. Sequence pins are energised*/

char BluetoothData = 0;                 /*Variable to store received Bluetooth Data*/
const int IRSensorPin = A0;             /*Infrared light sensor is conected to pin A0 on Arduino*/
const int buzzPin = 7;                  /*Buzzer is connected to pin 7 on Arduino*/
const int greenBtnPin = 6;              /*Green button to activate fingerprint sensor is connected to pin 6 of the Arduino*/
const int blueBtnPin = 12;              /*Blue button to rotate motor to close position is connected to pin 12 on arduino*/ 

uint8_t fingerprint;                    /*Variable used to store fingerprints*/
static char boxState = close;           /*Variable to keep track of if the box has been opened or closed*/
int BTState = BTDefault;                /*Variable to store state of Bluetooth signal received (If the open or close button was pressed on the app)*/ 

void setup() {

  Serial.begin(9600);                   /*Begin Serial communication with Arduino and laptop*/ 
  stepper.setMaxSpeed(1000);            /*Set speed of motor*/
  pinMode(buzzPin, OUTPUT);             /*Declare buzzer as output*/
  pinMode(IRSensorPin, INPUT);          /*Declare IR light sensor as input*/
  pinMode(greenBtnPin, INPUT);          /*Declare green button as input*/
  pinMode(blueBtnPin, INPUT);           /*Declare blue button as input*/

}

void loop() {
  int greenBtnState;
  int blueBtnState;

  greenBtnState = digitalRead(greenBtnPin);   /*Check the state of the green button and store in variable greenBtnState*/

  if (greenBtnState == HIGH){                 /*If green button is pressed*/
    checkFinger();                            /*Perform function to check the fingerprint*/
  }
  
  else {                                      /*If green button is not pressed*/
    hc06.begin(9600);                         /*Begin communication between Arduino and HC06*/
    if (hc06.available()){                    /*If HC06 is available*/ 
      checkBTData();                          /*Perform function to get Bluetooth data*/
    }
    
    blueBtnState = digitalRead(blueBtnPin);   /*Store the state of close button in variable closeBtn*/
    if (blueBtnState == HIGH){                /*If a the close button on the app was pressed*/
      if (boxState != close){                 /*If the box is already closed, do NOT rotate the motor*/
          closeMotor();                       /*Perform function to rotate motor to close position*/
      }
    }
  } 
}
/*This function begins serial communication between the arduino and the fingerprint sensor. 
 * The fingerprint sensor takes an image of the finger placed on the glass
 * This image is then converted to a template
 * This template is compared to all the previously stored templates in the fingerprint sensor.
 * In order to get a match the newly created template must match a previously enrolled fingerprint.
 * If the finger matches and the box is closed, perform the function to rotate the motor to the open position
 */
void checkFinger (void){
  
  finger.begin(57600);                                     /*Begin communication with fingerprint sensor*/
  finger.getParameters();                                  /*Get fingerprint sensor parameters, Baud rate, Capacity, Device Address*/
    
  fingerprint = finger.getImage();                         /*Take image of fingerprint*/
  fingerprint = finger.image2Tz();                         /*Turn image into a new template*/
  fingerprint = finger.fingerSearch();                     /*Search for new template in stored template*/
  
  if(fingerprint == FINGERPRINT_OK && boxState == close){  /*If fingerprint matches and the box is closed*/
    openMotor();                                           /*Open the box*/
  }
}

/*This function begins the serial communication between the HC06 and the arduino 
 * It then reads the data received from the HC06 and stores it in the variable BluetoothData
 * If BluetoothData is a '0' that means the person using the app pressed the CLOSE button on the app and 
 * wants to the motor to rotate to the CLOSE position
 * if BluetoothData is a '1' that means the person using the app pressed the OPEN button on the app and
 * wants the motor to rotate to the OPEN position
 * The openMotor function will only run if an OPEN signal was received and the box is not already OPEN 
 * The closemotor function will only run if a CLOSE signal was received and the box is not already CLOSED
 */
void checkBTData (void){
  
  hc06.begin(9600);                 /*Begin communication between Arduino and HC06*/
  BluetoothData = hc06.read();      /*Read the data from HC06 and store in variable BluetoothData*/

  if (BluetoothData == '0'){        /*If a 0 is received*/
    BTState = BTClose;              /*That means the close button was pressed on the app*/
  }
  else if (BluetoothData == '1'){   /*If a 1 is received*/
    BTState = BTOpen;               /*That means the Open button was pressed on the app*/
  }

  if (BTState == BTOpen){           /*If the Open button on the app was pressed*/
    if (boxState != open){          /*If the box is already open, do NOT rotate the motor*/
      openMotor();                  /*Perform function to rotate motor to open position*/
    }
  }
  else if (BTState == BTClose){     /*If a the close button on the app was pressed*/
    if (boxState != close){         /*If the box is already closed, do NOT rotate the motor*/
      closeMotor();                 /*Perform function to rotate motor to close position*/
    }
  }
  
}

/* This function is used to rotate the motor to the CLOSE position
 *  The motor can perform 4096 steps in 1 full revolution therefor 1024 is 1 quarter of a revolution or 90Degrees
 *  This functions sets the motors current position to 1024 and runs the motor in the minus direction until this value reaches 0
 *  The function delays to allow the motor to rotate and potentially block the infrared beam
 *  If the infrared beam is broken that means the latch has blocked it and the box is secure
 *  if the infrared beam is not broken that means the latch has not blocked it and the box is not secure
 *  The Infrared receiver has a value between 950 to 1024 when the beam is not blocked and a value of 0 to 20 when it is blocked
 *  After the delay, a reading from the Infrared receiver is taken and if its less than 900 the boxNotSecureBeep function will run 
 *  (900 was a threshold I picked that works during testing)
 *  Otherwise the boxSecureBeep function will run
 *  This function resets the BTState function to the default and sets the boxState to close after the motor has rotated
 *  
 */
void closeMotor (void){              

  int IRValue;

  stepper.setCurrentPosition(1024);        /*Set current position to 90degrees from close position*/
  while (stepper.currentPosition() != 0) { /*Rotate motor until the current position in close position*/
    stepper.setSpeed(-1000);               /*Positive number sets direction. 1000 is speed*/
    stepper.runSpeed();                    /*Start motor moving*/
   }

   BTState = BTDefault;
   boxState = close;
   delay(1000);

   IRValue = analogRead(IRSensorPin);     /*Read the value received by the Infrared light sensor and stor in lightValue variable*/

   if (IRValue > 900){                    /*If the light beam is not broken*/
    boxNotSecureBeep();                   /*Perform function to alert user the box is not secure*/
   }
   else {
    boxSecureBeep();                      /*Perform function to alert user the box is secure*/
   } 
}

/*This function is used to rotate the motor to the OPEN position
 * The motor can perform 4096 steps in 1 full revolution therefor 1024 is 1 quarter of a revolution or 90Degrees
 * This function sets the current position of the motor as 0 and runs the motor in the positive direction until it reached 1024 
 * Once the motor reaches the open position the BTState is reset to default and the boxState is set to OPEN
 */
void openMotor (void){                       
  
  stepper.setCurrentPosition(0);              /*Motor can do 4096 steps, set motor current position to 90degrees from open position*/
  while (stepper.currentPosition() != 1024) { /*Rotate motor until the current position is open position*/
    stepper.setSpeed(1000);                   /*Negitive speed sets the direction. 1000 is the speed*/
    stepper.runSpeed();                       /*Start motor moving */            
  }
  BTState = BTDefault;
  boxState = open;                            /*Set the prevstate to open*/
}
/*This function is used to alert the user of the box that the door has not been properly secured
 * This functions makes the buzzer perform 1 long beep 
 */
void boxNotSecureBeep(void){      /*Function to do 1 long beep (Box not locked correctly)*/
  digitalWrite(buzzPin, HIGH);    /*Turn buzzer on*/
  delay(1000);                    /*Delay while buzzer is on*/
  digitalWrite(buzzPin, LOW);     /*Turn buzzer off*/
}

/*This function is used to alert the user of the box that the door has been properly secured
 * This functions makes the buzzer perform 2 short beeps 
 */
void boxSecureBeep (void){        /*Function to do 2 short beeps (Box is locked correctly)*/
  digitalWrite(buzzPin, HIGH);    /*Turn buzzer on*/
  delay(50);                      /*Delay while buzzer on*/
  digitalWrite(buzzPin, LOW);     /*Turn buzzer off*/
  delay(50);                      /*Delay While buzzer off*/
  digitalWrite(buzzPin, HIGH);    /*Turn buzzer on*/
  delay(50);                      /*Delay while buzzer on*/
  digitalWrite(buzzPin, LOW);     /*Turn buzzer off*/
}
