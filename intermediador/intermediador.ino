// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for Serial2 (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      100         // [-] Maximum speed for testing
#define SPEED_STEP          0          // [-] Speed step
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#include <Arduino.h>

#define RXD2 16
#define TXD2 17



// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
bool hoverNewMessage = false;

uint8_t idxpc = 0;                        // Index for new data pointer
uint16_t bufStartFramepc;                 // Buffer Start Frame
byte *p_pc;                                // Pointer declaration for the new received data
byte incomingBytepc;
byte incomingBytePrevpc;
bool pcNewMessage = false;


typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;
SerialCommand NewCommand;
SerialCommand NewCommandPc;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  wheelR_cnt;
   int16_t  wheelL_cnt;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  //Serial.println("Hoverboard Serial v1.0");

  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
}

// ########################## SEND ##########################
// void Send(int16_t uSteer, int16_t uSpeed)
void Send()
{
  // Create command
  if(pcNewMessage)
  {
  //   Command.start    = (uint16_t)START_FRAME;
  //   Command.steer    = (int16_t)uSteer;
  //   Command.speed    = (int16_t)uSpeed;
  //   Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

    // Write to Serial
    Serial2.write((uint8_t *) &NewCommand, sizeof(Command));
    pcNewMessage = false;
  }
}


void SendPc()
{
  // Write to Serial
  if(hoverNewMessage)
  {
    Serial.write((uint8_t *) &Feedback, sizeof(Feedback)); 
    hoverNewMessage = false;
  }
}


// ########################## RECEIVE ##########################
void ReceivePc()
{
    // Check for new data availability in the Serial buffer
    if (Serial.available()) {
        incomingBytepc 	  = Serial.read();                                   // Read the incoming byte
        bufStartFramepc	= ((uint16_t)(incomingBytepc) << 8) | incomingBytePrevpc;       // Construct the start frame
    }
    else {
        return;
    }
    // Copy received data
    if (bufStartFramepc == START_FRAME) {	                    // Initialize if new data is detected
        p_pc       = (byte *)&NewCommandPc;
        *p_pc++    = incomingBytePrevpc;
        *p_pc++    = incomingBytepc;
        idxpc     = 2;	
    } else if (idxpc >= 2 && idxpc < sizeof(SerialCommand)) {  // Save the new received data
        *p_pc++    = incomingBytepc; 
        idxpc++;
    }	
    
    // Check if we reached the end of the package
    if (idxpc == sizeof(SerialCommand)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewCommandPc.start ^ NewCommandPc.steer ^ NewCommandPc.speed );

        // Check validity of the new data
        if (NewCommandPc.start == START_FRAME && checksum == NewCommandPc.checksum) {
            // Copy the new data
            memcpy(&Command, &NewCommand, sizeof(SerialCommand));


            // //Print data to built-in Serial
            // Serial.print("1: ");   Serial.print(Feedback.cmd1);
            // Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            // Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            // Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            // Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            // Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            // Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          // Serial.println("Non-valid data skipped");
        }
        pcNewMessage = true;
        idxpc = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrevpc = incomingBytepc;
}



void Receive()
{
    // Check for new data availability in the Serial buffer
    if (Serial2.available()) {
        incomingByte 	  = Serial2.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;
    
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  // #ifdef DEBUG_RX
  //       Serial.print(incomingByte);
  //       return;
  // #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	 
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas 
                              ^ NewFeedback.wheelR_cnt ^ NewFeedback.wheelL_cnt ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && NewFeedback.checksum == checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
            hoverNewMessage = true;

            // Print data to built-in Serial
            // Serial.print("1: ");   Serial.print(Feedback.cmd1);
            // Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            // Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            // Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            // Serial.print(" r: ");  Serial.print(Feedback.wheelR_cnt);
            // Serial.print(" l: ");  Serial.print(Feedback.wheelL_cnt);
            // Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            // Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            // Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);

        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

void loop(void)
{ 

  // // Check for new received data
 
  //PublisheData
  ReceivePc();
  Send();
  Receive();
  SendPc();
//
  //Receive();
  

//

}

// ########################## END ##########################