#include "Arduino.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "Adafruit_ssd1306syp.h"

#define VERSION "0.1"
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
bool lastReceivedData = HIGH;    //Start with relay off. The relay is activated by LOW signal

byte failureCount = MAX_PACKET_LOST_BEFORE_DISCONNECT+1;

void handleReceiverRole(void);
void handleTransmitterRole(void);

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
  radio.setAutoAck(0);                    // Ensure autoACK
  //radio.enableAckPayload();               // Allow optional ack payloads
  radio.enableDynamicPayloads();
  radio.setRetries(0,15);                 // Smallest time between retries, max no. of retries
  radio.setPayloadSize(PACKET_SIZE);
  radio.setChannel(0x77);                 //Select a channel
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.powerUp();
  if ( role == role_transmitter) {
    printf("\n\rPEK Transmitter ");

    pinMode(functional_pin,INPUT);
    digitalWrite(functional_pin,HIGH);

    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
    radio.stopListening();
  } else {
    printf("\n\rPEK Receiver ");

    pinMode(functional_pin,OUTPUT);
    digitalWrite(functional_pin, lastReceivedData);

    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
    radio.startListening();
  }
  printf(VERSION);
  printf("\n\r");
  radio.printDetails();
  display.clear();

  lastReceivedPacketTime = 0;
}

void loop(void) {
  display.clear();
  delay(50);
  display.setCursor(0,0);

  if ( role == role_transmitter ) {
    handleTransmitterRole();
  }

  if ( role == role_receiver ) {
    handleReceiverRole();
  }
  display.update();
}

/**
* Send data and wait for ACK packet until timeout
*/
bool sendAndWaitForAck(unsigned long data, bool debug = false) {
  bool sendSuccess = false;
  unsigned long startTime = millis();
  if (debug) printf("%lu: Trying to send packet %lu \n\r", micros(), data);
  do {
    if (debug) putchar('.');
    sendSuccess = radio.write(&data, PACKET_SIZE);
    unsigned long waitTime = millis()-startTime;
    if ( waitTime > 5000) {
      if (debug) printf("\n\r%lu: Packet send timeout\n\r", micros());
      return false;
    }
    if (!sendSuccess) {
      delay(500);
    }
  } while (!sendSuccess);
  if (debug) printf("\n\r%lu: Packet send success\n\r", micros());
  unsigned long ackPacket = 0;
  if (!radio.isAckPayloadAvailable()) {
    delay(100);
  }
  while (radio.isAckPayloadAvailable()) {
      radio.read( &ackPacket, PACKET_SIZE);
      if (debug) printf("%lu: ACK packet received: %lu \n\r", micros(), ackPacket);
      if (ackPacket == data) {
        if (debug) printf("%lu: ACK packet OK\n\r", micros());
        return true;
      } else {
        if (debug) printf("%lu: ACK packet wrong\n\r", micros());
        delay(50);
      }
  }
  return true;
}

//
// Transmitter role
//
void handleTransmitterRole() {
  display.print("PEK Transmitter ");
  display.print(VERSION);
  unsigned long time = micros();
  if (time>0 && time<256) { //As 0 to 255 are control packets
    time = 256;
  }

  //printf("Sending sync time %lu \n\r", time);
  boolean sendSuccess = sendAndWaitForAck(time, false);
  if (!sendSuccess) {
    printf("%lu: Sync packet transmission failed\n\r", micros());
    failureCount++;
  } else {
    if (failureCount>MAX_PACKET_LOST_BEFORE_DISCONNECT) {
      printf("%lu: Connection restored\n\r", micros());
    }
    failureCount = 0;
  }

  display.setCursor(0,15);
  display.print("Link:");
  display.setCursor(35,15);
  if (failureCount>=MAX_PACKET_LOST_BEFORE_DISCONNECT) {
    failureCount = MAX_PACKET_LOST_BEFORE_DISCONNECT;
    printf("%lu: No connection to the reciever\n\r", micros());
    display.print("NO CONNECTION");
    display.update();
    return;
  }
  display.print("OK");
  unsigned long controlData = digitalRead(functional_pin);
  if (lastReceivedData!=controlData) {
    printf("%lu: Sending control packet %lu\n\r", micros(), controlData);
  }
  lastReceivedData = controlData;
  display.setCursor(0,30);
  display.print("Contact: ");
  display.setCursor(50,30);
  if (controlData==0) {
    display.print("OFF");
  } else {
    display.print("ON");
  }
  sendSuccess = sendAndWaitForAck(controlData, true);
  if (sendSuccess) {
    //printf("Control send succesfully\n\r");
  } else {
    printf("%lu: Control send failed\n\r", micros());
    failureCount++;
  }

}

//
// Receiver role
//
void handleReceiverRole() {
  display.print("PEK Receiver ");
  display.print(VERSION);
  display.setCursor(0,30);
  display.print("Relay: ");

  unsigned long receivedData = 0;
  bool newPinMode = lastReceivedData;
  byte pipeNum;
  while( radio.available(&pipeNum)) {              // Read all available payloads
    radio.read( &receivedData, PACKET_SIZE );
    radio.flush_tx();
    delay(50);
    radio.writeAckPayload(pipeNum, &receivedData, PACKET_SIZE);
    printf("%lu: Received and ACK'ed %lu \n\r", micros(), receivedData);
    lastReceivedPacketTime = millis();

    if (receivedData >= 0 && receivedData < 256) {
      if (receivedData==1) {
        newPinMode = LOW;
      } else if (receivedData==0) {
        newPinMode = HIGH;
      }
    }

    //printf("Millis since last signal: %lu \n\r", timeDelta);
  }
  unsigned long timeDelta = millis()-lastReceivedPacketTime;

  display.setCursor(0,15);
  display.print("Link: ");
  display.setCursor(35,15);
  if ( timeDelta < MAX_MILLIS_BEFORE_ERROR_SIGNAL) {
    display.print("OK");
  } else {
    display.print("NO CONNECTION");
    newPinMode = HIGH;
  }
  if (lastReceivedData!=newPinMode) {
    printf("%lu: New pin mode %i\n", micros(), newPinMode);
  }
  display.setCursor(40,30);

  if (newPinMode==HIGH) {
    display.print("OFF");
  } else {
    display.print("ON");
  }
  if (lastReceivedData!=newPinMode) {
    digitalWrite(functional_pin, newPinMode);
  }
  lastReceivedData = newPinMode;
  display.setCursor(0, 45);
  display.print(receivedData);
}
