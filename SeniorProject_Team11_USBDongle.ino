/*
 * Name:            Senior Project Team 11
 * File:            SeniorProject_Team11_USBDongle.ino
 * Description:     This code is to be used to control the USB module used in the EMG Controlled Robotic Limb Project.
 *                  It performs UART with a serial monitor, as well as SPI with an nrf24l01+ module.
 *                  This also utilizes the arduino serial plotter to plot real time EMG data
 *
 */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
//Pin 13 -> SCK
//Pin 12 -> MISO
//Pin 11 -> MOSI
//Pin  8 -> CSN
//Pin  7 -> CE
//Unused -> IRQ

//List of error codes, description is at bottom of code
const uint16_t Error1 = 0b0000000000000001;
const uint16_t Error2 = 0b0000000000000010;
const uint16_t Error3 = 0b0000000000000100;
const uint16_t Error4 = 0b0000000000001000;
const uint16_t Error5 = 0b0000000000010000;
const uint16_t Error6 = 0b0000000000100000;

// an identifying device destination
// Let these addresses be used for the pair
uint8_t address[][6] = {"10dar", "30dar"};    //our address is 30dar, addresses are backwards compared to the MSP432

// uniquely identify which address this radio will use to transmit
bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

struct PayloadStruct {
  char message[13];          // only using 13 characters for TX & ACK payloads
  uint8_t counter;
};
PayloadStruct payload;

unsigned long previousMillis = 0;        // will store last time message was sent
const long interval = 7000;             // 7 second timeout for communication to RF module

void errorcheck(int ErrorCode);

void setup() {

  
  Serial.begin(115200);         //baud rate needs to be 115200 in serial monitor/plotter
  
    // initialize the transceiver on the SPI bus
  while (!radio.begin()) {
    Serial.println("USB RF module not responding"); // hold in infinite loop
  }
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_HIGH);     // RF24_PA_MAX is default.

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  radio.startListening();                                     // put radio in RX mode
}

void loop() 
{
    
    uint8_t pipe;
    int Sensor1;
    int Sensor2;
    int Sensor3;
    int Sensor4;
    int BatteryCharge;
    int ErrorCode;

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) 
    {
     // save the last time
     previousMillis = currentMillis;
     ErrorCode |= Error2;             //flag an error
     errorcheck(ErrorCode);           //display error
    }
    else
    {
      ErrorCode &= ~Error2;
    }
    
    if(Serial.available())                //use serial port to check for E for error display or D for data display
    {
        char c = toupper(Serial.read());
        
        if (c == 'E') 
        {
          role = true;
    
        } 
        else if (c == 'D') 
        {
          role = false;
        }
    }
    if (radio.available(&pipe)) 
    {                    // is there a payload? get the pipe number that recieved it
      previousMillis = currentMillis;                 //reset timeout counter
      uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
      PayloadStruct received;
      radio.read(&received, sizeof(received));       // get incoming payload

      //Convert message to ints for data display
      if(received.message[1] == '.')
      {
        Sensor1 = (float)(received.message[0] - '0');
      }
      else
      {
        Sensor1 = (float)((received.message[0] - '0')*10 + (received.message[1] - '0'));
      }

      if(received.message[3] == '.')
      {
        Sensor2 = (float)(received.message[2] - '0');
      }
      else
      {
        Sensor2 = (float)((received.message[2] - '0')*10 + (received.message[3] - '0'));
      }

      if(received.message[5] == '.')
      {
        Sensor3 = (float)(received.message[4] - '0');
      }
      else
      {
        Sensor3 = (float)((received.message[4] - '0')*10 + (received.message[5] - '0'));
      }

      if(received.message[7] == '.')
      {
        Sensor4 = (float)(received.message[6] - '0');
      }
      else
      {
        Sensor4 = (float)((received.message[6] - '0')*10 + (received.message[7] - '0'));
      }

      if(received.message[9] == '.')
      {
        BatteryCharge = (float)(received.message[8] - '0');
      }
      else
      {
        BatteryCharge = (float)((received.message[8] - '0')*10 + (received.message[9] - '0'));
      }

      //To read errors we need to combine both characters and shift the first one over since each error is a bit
      ErrorCode = (uint16_t)((received.message[10]<<8) | (received.message[11]));
      
      //Error check for if sensors arent reading
      if(!Sensor1 && !Sensor2 && !Sensor3 && !Sensor4)
      {
        ErrorCode |= Error5;
      }
      else
      {
        ErrorCode &= ~Error5;
      }
      
      //switches between plotting the data and printing errors. User must clear serial port between switches
      if(!role)
      {
        Serial.print("Sensor1:"); Serial.print(Sensor1); Serial.print(", ");      //format for printing to the serial plotter
        Serial.print("Sensor2:"); Serial.print(Sensor2); Serial.print(", ");
        Serial.print("Sensor3:"); Serial.print(Sensor3); Serial.print(", ");
        Serial.print("Sensor4:"); Serial.print(Sensor4); Serial.print(", ");
        Serial.println();
      }
      else if(role)
      {
        errorcheck(ErrorCode);                        //error display and battery display
        Serial.print("Battery: ");
        Serial.println(BatteryCharge);
      }
    }

} // loop

/*----------------------------------------------------------------
 * void errorcheck(int ErrorCode)
 *
 * Description: Displays errors of project to the seial monitor
 * Inputs: Error code data
 * Outputs: None
----------------------------------------------------------------*/
void errorcheck(int ErrorCode)
{
      //Error checking using bitwise operations
      Serial.println("Errors:");
      
      if(ErrorCode & Error1)
      {
        Serial.println("  Harness RF module not responding");
      }

      if(ErrorCode & Error2)
      {
        Serial.println("  Arm RF module not responding");
      }
      
      if(ErrorCode & Error3)
      {
        Serial.println("  USB RF module not responding");
      }
      
      if(ErrorCode & Error4)
      {
        Serial.println("  Not receiving information from battery module");
      }
      
      if(ErrorCode & Error5)
      {
        Serial.println("  EMG Sensors not reading");
      }

      if(!ErrorCode)
      {
        Serial.println("  None");
      }
}
