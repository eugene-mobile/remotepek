#include "Arduino.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "Adafruit_ssd1306syp.h"

#define SDA_PIN 8
#define SCL_PIN 7
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 (CE & CS)
RF24 radio(9,10);

// sets the role of this unit in hardware.  Connect to GND to be the 'led' board receiver
// Leave open to be the 'remote' transmitter
const int role_pin = A4;
const int functional_pin = 2;
const int PACKET_SIZE = sizeof(long);

const byte MAX_PACKET_LOST_BEFORE_DISCONNECT = 5;

// Single radio pipe address for the 2 nodes to communicate.
byte addresses[][6] = {"Forwd","Backw"};

typedef enum { role_transmitter = 1, role_receiver } role_e;

// The role of the current running sketch
role_e role;
const long MAX_MILLIS_BEFORE_ERROR_SIGNAL = 10000;
long lastReceivedPacketTime = 0 - MAX_MILLIS_BEFORE_ERROR_SIGNAL;
bool lastReceivedData = LOW;    //Start with relay off

byte failureCount = 0;

void handleReceiverRole(void);

void setup(void) {
  display.initialize();
  display.clear();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // set up the role pin
  pinMode(role_pin, INPUT);
  digitalWrite(role_pin,HIGH);
  delay(20); // Just to get a solid reading on the role pin

  // read the address pin, establish our role
  if ( digitalRead(role_pin) )
    role = role_transmitter;
  else
    role = role_receiver;

  Serial.begin(115200);
  printf_begin();

  radio.begin();
  radio.setDataRate(RF24_2MBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  if ( role == role_transmitter) {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
    printf("\n\rPEK Transmitter\n\r");
    pinMode(functional_pin,INPUT);
    digitalWrite(functional_pin,HIGH);
  } else {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
    printf("\n\rPEK Receiver\n\r");
    radio.startListening();
    pinMode(functional_pin,OUTPUT);
    digitalWrite(functional_pin, lastReceivedData);
  }
  display.clear();
  radio.printDetails();
  lastReceivedPacketTime = micros();
}

void loop(void) {
  display.clear();
  delay(50);
  display.setCursor(0,0);
  if ( role == role_transmitter ) {
    display.print("PEK Transmitter");

    radio.stopListening();
    unsigned long time = micros();
    if (time>0 && time<256) { //As 0 to 255 are control packets
      time = 256;
    }
    printf("Sending current time %lu \n\r", time);
    radio.write(&time, PACKET_SIZE);
    delay(10);

    unsigned long ackTime;
    radio.read( &ackTime, PACKET_SIZE);                  // Read it, and display the response time
    printf("Sync time received: %lu \n\r", ackTime);
    if (ackTime == time) {
      printf("Time sync packet OK\n\r");
      failureCount = 0;
    } else {
      failureCount++;
    }
    printf("Failure count= %d \n", failureCount);

    display.setCursor(0,15);
    display.print("Link:");
    display.setCursor(35,15);
    if (failureCount>=MAX_PACKET_LOST_BEFORE_DISCONNECT) {
      failureCount = MAX_PACKET_LOST_BEFORE_DISCONNECT;
      printf("No connection to the reciever\n\r");
      display.print("NO CONNECTION");
      display.update();
      delay(10);
      return;
    }
    display.print("OK");
    unsigned long controlData = digitalRead(functional_pin);
    radio.write(&controlData, PACKET_SIZE);

    unsigned long ackControl;
    radio.read( &ackControl, PACKET_SIZE);
    if (ackControl == controlData) {
      printf("Control packet %lu sent succesfully\n\r", controlData);
    } else {
      printf("Control packet %lu send failed\n\r", controlData);
    }
  }

  //
  // Receiver role
  //
  if ( role == role_receiver ) {
    handleReceiverRole();
  }

}

void handleReceiverRole() {
  display.print("PEK Receiver");
  display.setCursor(0,30);
  display.print("Relay: ");

  long receivedData;
  bool newPinMode = LOW;
  uint8_t pipeNum;
  while( radio.available(&pipeNum)) {              // Read all available payloads
    radio.read( &receivedData, PACKET_SIZE );
    radio.writeAckPayload(pipeNum, &receivedData, PACKET_SIZE);
    printf("Received and ACK'ed %lu \n\r", receivedData);
    lastReceivedPacketTime = millis();

    if (receivedData >= 0 && receivedData < 256) {
      if (receivedData==1) {
        newPinMode = HIGH;
      } else if (receivedData==0) {
        newPinMode = LOW;
      }
    }
  }
  display.setCursor(40,30);

  if (newPinMode==HIGH) {
    display.print("ON");
  } else {
    display.print("OFF");
  }
  if (lastReceivedData!=newPinMode) {
    digitalWrite(functional_pin, newPinMode);
  }
  lastReceivedData = newPinMode;
  unsigned long timeDelta = millis()-lastReceivedPacketTime;
  printf("Millis since last signal: %lu \n\r", timeDelta);
  display.setCursor(0,15);
  display.print("Link: ");
  display.setCursor(35,15);
  if ( timeDelta < MAX_MILLIS_BEFORE_ERROR_SIGNAL) {
    display.print("OK");
  } else {
    display.print("NO CONNECTION");
  }
  display.update();
}
