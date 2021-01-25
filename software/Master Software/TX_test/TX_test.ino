#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Define pins
#define  LED_BLUE     0
#define  LED_RED      1
#define  CE           3 // 3 Arduino pro mini
#define  CSN          8  // 8 Arduino pro mini
#define BUTTON_X      A0
#define BUTTON_Y      A1
#define  DEBUG_PIN    0
#define MOTOR_OPCODE  0x02
#define POTENTIOMETER A4

#define PACKET_SIZE     15
#define BUFFER_SIZE     30
#define NUMBER_OF_NODES 6

// #define JOYSTICK CONTROL
// #define COMPUTER CONTROL
#define which_node 0

RF24 radio(CE,CSN); // Default SPI speed is 4Mbit, but should work up to 10MBbit

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[] = {0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL};

bool SERIAL_DEBUG  = true;
uint8_t receiveData[PACKET_SIZE] = {};
uint8_t sendData[PACKET_SIZE] = {};

int state = 0;
int speed_forward  = 0;  

uint8_t robotUnderControl = 0;

uint8_t serialBuffer[BUFFER_SIZE] = { }; //Initialize to zeros
uint8_t computerReceiveBuffer[PACKET_SIZE][NUMBER_OF_NODES] = { };

void setup(void)
{
  pinMode(LED_BLUE, OUTPUT);

  Serial.begin(0);

  radio.begin();

  radio.setRetries(10,0); //retries, delay
//  radio.setPayloadSize(PACKET_SIZE);
  radio.setDataRate(RF24_2MBPS);
//  radio.setChannel(60);

  radio.openReadingPipe(0,pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.openReadingPipe(2,pipes[2]);
  radio.openReadingPipe(3,pipes[3]);
  radio.openReadingPipe(4,pipes[4]);
  radio.openReadingPipe(5,pipes[5]);

  radio.startListening();

//  digitalWrite(LED_RED, HIGH);
}

void loop(void)
{
  // Under joystick control. 
  getJoyStickData();

  //--------------CHECK FOR SERIAL PORT DATA---------------
  uint8_t pipeNum = 0; //variable to hold which reading pipe sent data
  byte gotByte = 0; //used to store payload from transmit module
  int numBytes = 0;

  //------------CHECK THE RADIO----------------------------
  while(radio.available(&pipeNum)){ //Check if received data
     radio.read( &receiveData, PACKET_SIZE ); //read one byte of data and store it in gotByte variable
     //printReceiveData(pipeNum);
//     printReceiveDataFormated(pipeNum);
     delay(5);

     if (sendToSlave(pipeNum,  PACKET_SIZE)) {
        //Success sending
//        Serial.println("Success sending");
     }
     else {
        //failed sending
     }
     digitalWrite(DEBUG_PIN,LOW);
  } // end while
}   // end loop

bool sendToSlave(byte whichPipe, uint8_t packet_size) {
    bool worked;
    radio.stopListening();
    radio.openWritingPipe(pipes[whichPipe]); 
    if(!radio.write(&sendData,sizeof(sendData))){
//      Serial.println("worked false");
      worked = false; 
    }
    else{
      worked = true;
//      Serial.println("worked");
    }
    radio.startListening(); 
    return worked; 
} //end sendToSlave

void printReceiveData(byte whichPipe) { 
   Serial.print("S");
   Serial.print(",");
   Serial.print(whichPipe); 
   Serial.print(",");
  for (int i=0; i<PACKET_SIZE; i++) {
         Serial.print(receiveData[i]); 
         if(i<PACKET_SIZE-1){
          Serial.print(",");
         }
  }
  Serial.println(" ");
}

void printReceiveDataFormated(byte whichPipe) { 
   Serial.print("S");
   Serial.print(",");
   Serial.print(whichPipe); 
   Serial.print(",");
   Serial.print(receiveData[0]); 
   Serial.print(","); 
   Serial.print(receiveData[1]); 
   Serial.print(","); 
   Serial.print(receiveData[2]); 
   Serial.print(",");
   Serial.print(receiveData[3]); 
   Serial.print(",");
   Serial.print(receiveData[4]); 
   Serial.print(",");
   Serial.println(receiveData[5]);
//   Serial.print(",");

   unsigned long d;
   d =  (receiveData[6] << 24) | (receiveData[5] << 16) | (receiveData[4] << 8) | (receiveData[3]);
   //   float member = *(float *)&d;
   float member = float(d);
   Serial.print( member );
   Serial.print(","); 

   d =  (receiveData[10] << 24) | (receiveData[9] << 16) | (receiveData[8] << 8) | (receiveData[7]);
   member = *(float *)&d;
   Serial.print( member ); 

   Serial.print(","); 

   d =  (receiveData[14] << 24) | (receiveData[13] << 16) | (receiveData[12] << 8) | (receiveData[11]);
   member = *(float *)&d;
   Serial.println( member );
}

void getJoyStickData() {
  sendData[0] = MOTOR_OPCODE;
  sendData[1] = 120; 
  sendData[2] = 120;
  sendData[3] = 0;
  sendData[4] = 1;
  state = 1;

//  int forward_back = analogRead(BUTTON_X); 
//  int left_right   = analogRead(BUTTON_Y); 
//  float speed_scalar = (float)analogRead(POTENTIOMETER)/594.0;
//  speed_forward  = 0;
//  int intesityTurn = 0;
//  if (forward_back>=556) {
//    speed_forward = speed_scalar*((forward_back-556)/2);
//  }
//  else if (forward_back<556) {
//     speed_forward = speed_scalar*((556-forward_back)/2)-25;
//     if(speed_forward <=0)  speed_forward = 0;
//  }
//
//  //debugButtons();
//
//  if (left_right >700) { 
//    //turn LEFT
//      sendData[0] = MOTOR_OPCODE;
//      sendData[1] = 120; 
//      sendData[2] = 120;
//      sendData[3] = 1;
//      sendData[4] = 1;
//      state = 1;
//  }
//
//  else if (left_right < 300) { 
//    //turn RIGHT
//      sendData[0] = MOTOR_OPCODE;
//      sendData[1] = 130; 
//      sendData[2] = 130;
//      sendData[3] = 0;
//      sendData[4] = 0;
//      state =2;
//  }
//
//  else if (forward_back > 600) { 
//    //go FORWARD 
//      sendData[0] = MOTOR_OPCODE;
//      sendData[1] = speed_forward; 
//      sendData[2] = speed_forward; 
//      sendData[3] = 1;
//      sendData[4] = 0;
//      state = 3; 
//  }
//
//  else if (forward_back < 400) {
//    //go BACKWARDS
//      sendData[0] = MOTOR_OPCODE;
//      sendData[1] = speed_forward;  //turning left  60/60 original
//      sendData[2] = speed_forward;
//      sendData[3] = 0;
//      sendData[4] = 1;
//      state = 4;
//  }
//
//  else  { 
//    sendData[0] = MOTOR_OPCODE;
//    sendData[1] = 0; 
//    sendData[2] = 0;
//    state = 0;
//  }
}

void debugButtons() { 
  Serial.print(analogRead(BUTTON_X));
  Serial.print(",");
  Serial.print(analogRead(BUTTON_Y));
  Serial.print(","); 
  Serial.print(state);
  Serial.print(",");
  Serial.println(speed_forward); 
}
