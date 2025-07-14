/*
  Controlling VALETON GP-100 with Arduino and MIDI

  andrevjunqueira@yahoo.com
  
  With code Made by Gustavo Silveira.

  If you are using for anything that's not for personal use don't forget to give credit.



VALETON GP-100 LIST OF MIDI COMANDS
CC 7 Patch Volume
CC 11 Volume Pedal
CC 13 Expression Pedal
CC 24 Patch Up
CC 25 Patch Down
CC 48 PRE ON/OFF
CC 49 DST ON/OFF
CC 50 AMP ON/OFF
CC 51 NR ON/OFF
CC 52 CAB ON/OFF
CC 53 EQ ON/OFF
CC 54 MOD ON/OFF
CC 55 DLY ON/OFF
CC 56 RVB ON/OFF
CC 57 Tuner ON/OFF
CC 58 DRUM Menu
CC 59 DRUM ON/OFF
CC 60 SET Drum Style
CC 61 SET Drum Volume
CC 62 Loop Menu
CC 63 Loop Record
CC 64 Loop Play/Stop
CC 65 Loop Erase
CC 66 SET Loop Rec Volume
CC 67 SET Loop Play Volume
CC 68 Foot Switch Mode (Global)
CC 69 Volume Pedal Post/Pre (Global)
CC 70 SET BPM1 (not Taptempo)
CC 71 SET BPM2 (not Taptempo)
CC 72 SET Input Level
CC 73 SET DRUM BPM1 (not Taptempo)
CC 74 SET DRUM BPM2 (not Taptempo)

To use Program Change you must first send CC0 to enter Patch Change mode

BANK MSB: 

01-A~32-D: CC0=1, PC=0-127 

33-A~64-D: CC0=0, PC=0-127

*/


#include <usbh_midi.h>
#include <usbhub.h>

USB Usb;
USBHub Hub(&Usb);
USBH_MIDI  Midi(&Usb);

//USBMIDI_CREATE_DEFAULT_INSTANCE();

//#define DEBUG_SERIAL 0


// ResponsiveAnalogRead - smooth analog values
#include <ResponsiveAnalogRead.h>  
//https://github.com/dxinteractive/ResponsiveAnalogRead


/////////////////////////////////////////////
// BUTTONS

const int N_BUTTONS = 8;                                //  total numbers of buttons
//with the usb shield only pins 2 through 8 and 12 can be used - pins 9 and 10 do not work and the 0 and 1 are TX and RX serial and tb cannot be used
int BUTTON_ARDUINO_PIN[N_BUTTONS] = { 2, 3, 4, 5, 6, 7, 8, 12 };  // pins of each button connected straight to the Arduino

int buttonCState[N_BUTTONS] = {};  // stores the button current value
int buttonPState[N_BUTTONS] = {};  // stores the button previous value

//#define pin13 1 // uncomment if you are using pin 13 (pin with led), or comment the line if not using
byte pin13index = 12;  // put the index of the pin 13 of the buttonPin[] array if you are using, if not, comment

// debounce
unsigned long lastDebounceTime[N_BUTTONS] = { 0 };  // the last time the output pin was toggled
unsigned long debounceDelay = 50;                   // the debounce time; increase if the output flickers

//FOR VALETON GP100 below list of each MIDI CC of each button
//Chance the corresponding MIDI CC below to control other functions besides this
#define EXPPEDAL 11 //Volume Pedal
#define BOTAO01 49 //DST ON/OFF
#define BOTAO02 55 //DLY ON/OFF
#define BOTAO03 56 //RVB ON/OFF
#define BOTAO04 25 //Patch Down
#define BOTAO05 48 //PRE ON/OFF
#define BOTAO06 54 //MOD ON/OFF 
#define BOTAO07 59 //DRUM ON/OFF
#define BOTAO08 24 //Patch Up

//store the CC codes corresponding to each button
int BUTTON_MIDICC[N_BUTTONS] = { BOTAO01, BOTAO02, BOTAO03, BOTAO04, BOTAO05, BOTAO06, BOTAO07, BOTAO08 };

//Saves the current actuation values of each button as 
//the GP100 waits 0 for triggering and 127 for switching off
//Boots with the value of Off 127
int BUTTON_STATE[N_BUTTONS] = {127, 127, 127, 127, 127, 127, 127, 127};


/////////////////////////////////////////////
// POTENTIOMETERS


const int N_POTS = 1;                            // total numbers of pots (slide & rotary)
const int POT_ARDUINO_PIN[N_POTS] = { A5 };  // ARDUINO ANALOG IN - pins of each pot connected straight to the Arduino

int potCState[N_POTS] = { 0 };  // Current state of the pot
int potPState[N_POTS] = { 0 };  // Previous state of the pot
int potVar = 0;                 // Difference between the current and previous state of the pot

int midiCState[N_POTS] = { 0 };  // Current state of the midi value
int midiPState[N_POTS] = { 0 };  // Previous state of the midi value

const int TIMEOUT = 300;              // Amount of time the potentiometer will be read after it exceeds the varThreshold
const int varThreshold = 20;          // Threshold for the potentiometer signal variation
boolean potMoving = true;             // If the potentiometer is moving
unsigned long PTime[N_POTS] = { 0 };  // Previously stored time
unsigned long timer[N_POTS] = { 0 };  // Stores the time that has elapsed since the timer was reset

int reading = 0;
// Responsive Analog Read
float snapMultiplier = 0.01;                      // (0.0 - 1.0) - Increase for faster, but less smooth reading
ResponsiveAnalogRead responsivePot[N_POTS] = {};  // creates an array for the responsive pots. It gets filled in the Setup.

int potMin = 205;
int potMax = 1023;

// Maximum pot value and volume for the GP100 - which can only have a maximum value of 100 MIDI
const int GP100_MAX_VOLUME_EXP_PEDAL_MIDI_CC_VALUE = 100;


/////////////////////////////////////////////
// MIDI
byte midiCh = 1;  // MIDI channel to be used - start with 1 for MIDI.h lib or 0 for MIDIUSB lib
byte note = 36;   // Lowest note to be used
byte cc = 01;      // Lowest MIDI CC to be used

void MIDI_poll();

void onInit()
{
  char buf[20];
  uint16_t vid = Midi.idVendor();
  uint16_t pid = Midi.idProduct();
  #ifdef DEBUG_SERIAL
  sprintf(buf, "VID:%04X, PID:%04X", vid, pid);

  Serial.println(buf); 
  #endif
}

/////////////////////////////////////////////
// SETUP
void setup() {


  Serial.begin(115200);

  if (Usb.Init() == -1) {
    while (1); //halt
  }//if (Usb.Init() == -1...
  delay( 200 );

  // Register onInit() function
  Midi.attachOnInit(onInit);


  // Buttons
  //{ 2, 3, 4, 5, 6, 7, 8, 10 }; 
  // Initialize buttons with pull up resistors
  for (int i = 0; i < N_BUTTONS; i++) {
    pinMode(BUTTON_ARDUINO_PIN[i], INPUT_PULLUP);

  }

  pinMode(13, OUTPUT);


  for (int i = 0; i < N_POTS; i++) {
    responsivePot[i] = ResponsiveAnalogRead(0, true, snapMultiplier);
    responsivePot[i].setAnalogResolution(1023);  // sets the resolution
  }


}



/////////////////////////////////////////////
// LOOP
void loop() {

  Usb.Task();
  if ( Midi ) {
    MIDI_poll();
  }

  //Read Arduino butons
  buttons();

  //Read Arduino pots
  potentiometers();


}



/////////////////////////////////////////////
// BUTTONS


void buttons() {

  for (int i = 0; i < N_BUTTONS; i++) {

    buttonCState[i] = digitalRead(BUTTON_ARDUINO_PIN[i]);  // read pins from arduino


    
       //Serial.print("------------ reading BUTTON:   ");
       //Serial.println(i+1);



    if ((millis() - lastDebounceTime[i]) > debounceDelay) {

      if (buttonPState[i] != buttonCState[i]) {
        lastDebounceTime[i] = millis();

        if (buttonCState[i] == LOW) {

          // Sends the MIDI CC

            //MIDi CC


            //reverses the current state of the button to send it
            if (BUTTON_STATE[i] == 0) {
                BUTTON_STATE[i] = 127;
            } else {
                BUTTON_STATE[i] = 0;
            }



            
            controlChange(midiCh, BUTTON_MIDICC[i], BUTTON_STATE[i]);  //  (channel, CC number,  CC value)

            #ifdef DEBUG_SERIAL
            Serial.print("############# BOTAO APERTADO:  ");
            Serial.println(i+1);
            #endif
    


        } else {




        }
        buttonPState[i] = buttonCState[i];
      }
    }
  }
}



/////////////////////////////////////////////
// POTENTIOMETERS


void potentiometers() {


  for (int i = 0; i < N_POTS; i++) {  // Loops through all the potentiometers

    reading = analogRead(POT_ARDUINO_PIN[i]);
    responsivePot[i].update(reading);
    potCState[i] = responsivePot[i].getValue();

    midiCState[i] = map(potCState[i], potMin, potMax, 0, GP100_MAX_VOLUME_EXP_PEDAL_MIDI_CC_VALUE);  // Maps the reading of the potCState to a value usable in midi


    if (midiCState[i] < 0) {
      midiCState[i] = 0;
    }
    if (midiCState[i] > GP100_MAX_VOLUME_EXP_PEDAL_MIDI_CC_VALUE) {
      midiCState[i] = 0;
    }

    potVar = abs(potCState[i] - potPState[i]);  // Calculates the absolute value between the difference between the current and previous state of the pot
    //Serial.println(potVar);

    if (potVar > varThreshold) {  // Opens the gate if the potentiometer variation is greater than the threshold
      PTime[i] = millis();        // Stores the previous time
    }

    timer[i] = millis() - PTime[i];  // Resets the timer 11000 - 11000 = 0ms

    if (timer[i] < TIMEOUT) {  // If the timer is less than the maximum allowed time it means that the potentiometer is still moving
      potMoving = true;
    } else {
      potMoving = false;
    }

    if (potMoving == true) {  // If the potentiometer is still moving, send the change control
      if (midiPState[i] != midiCState[i]) {

        // Sends the MIDI CC accordingly to the chosen board



        //use if using with ATmega32U4 (micro, pro micro, leonardo...)
       controlChange(midiCh, EXPPEDAL , midiCState[i]);  //  (channel, CC number,  CC value)
        //MidiUSB.flush();


        potPState[i] = potCState[i];  // Stores the current reading of the potentiometer to compare with the next
        midiPState[i] = midiCState[i];
      }
    }
  }
}





// Arduino (pro)micro midi functions MIDIUSB Library
void noteOn(byte channel, byte pitch, byte velocity)  {
  //midiEventPacket_t noteOn = { 0x09, 0x90 | channel, pitch, velocity };
  //MIDI.sendNoteOn(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  //midiEventPacket_t noteOff = { 0x08, 0x80 | channel, pitch, velocity };
  //MIDI.sendNoteOff(noteOff);
}




void controlChange(byte channel, byte control, byte value)  {


  Usb.Task();
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {

        //Gp-100 - Use the pre trigger button to also turn on the expression pedal
        if (control == 48) {

          byte Message[3];                 // Construct the midi message (2 bytes)
          Message[0]=176;                 // 0x0B Control change
          Message[1]=control;                // Number of Control Change
          Message[2]=value;                  // Velocity amount
          Midi.SendData(Message, 0);          // Send the message
        
        
          delay(50);

                #ifdef DEBUG_SERIAL
                Serial.print("########################################### Sending  MIDI CC - control: ");
                Serial.print(Message[1]);
                Serial.print(" - value: ");
                Serial.print(Message[2]);
                Serial.print(" - channel: ");
                Serial.println(channel);
                Serial.println("  ######################################################################");
                #endif


          Message[0]=176;                 // 0x0B Control change

          Message[1]=13;                // MIDI CC 13 FOR EXPRESSIO PEDAL ACTIVATION

          if (value==0) {
              Message[2]=64; 
 
          } else {
            Message[2]=0; 

          }

          Midi.SendData(Message, 0);          // Send the message

                #ifdef DEBUG_SERIAL
                Serial.print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Sending MIDI CC - control: ");
                Serial.print(Message[1]);
                Serial.print(" - value: ");
                Serial.print(Message[2]);
                Serial.print(" - channel: ");
                Serial.println(channel);
                Serial.println("  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
                #endif
        
        
          delay(50);


        } else {


        

          byte Message[3];                 // Construct the midi message (2 bytes)
          Message[0]=176;                 // 0x0B Control change
          Message[1]=control;                // Number of Control Change
          Message[2]=value;                  // Velocity amount
          Midi.SendData(Message, 0);          // Send the message
        
        
          delay(10);

          #ifdef DEBUG_SERIAL
          Serial.print("-------------Sending MIDI CC - control: ");
          Serial.print(control);
          Serial.print(" - value: ");
          Serial.print(value);
          Serial.print(" - channel: ");
          Serial.print(channel);
          Serial.println(" ---------------------------");
          #endif

        }


  }

}

// Poll USB MIDI Controler and send to serial MIDI
void MIDI_poll()
{

  #ifdef DEBUG_SERIAL

  char buf[16];
  uint8_t bufMidi[MIDI_EVENT_PACKET_SIZE];
  uint16_t  rcvd;

  if (Midi.RecvData( &rcvd,  bufMidi) == 0 ) {
    uint32_t time = (uint32_t)millis();
    
    sprintf(buf, "%04X%04X:%3d:", (uint16_t)(time >> 16), (uint16_t)(time & 0xFFFF), rcvd); // Split variable to prevent warnings on the ESP8266 platform
    Serial.print(buf);


    for (int i = 0; i < MIDI_EVENT_PACKET_SIZE; i++) {
      sprintf(buf, " %02X", bufMidi[i]);
      Serial.print(buf);

    }
    Serial.println("");
  }

  #endif
}
