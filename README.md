**Controlling VALETON GP-100 with Arduino and MIDI**

---

Controlling VALETON GP-100 with Arduino leonardo and MIDI usinng a USB HOST SHIELD to conect the GP-100 to Arduino with GP-100 USB-B port.  
  
The original project uses 8 momentary switches but 10 can be used with code modification.  
  
The project also uses 1 analog input to read a potentiometer to be used as Expession Pedal.  
  
The Pinout is as folow:  
  
Pin 2 to 8 and pin 12 as DIGITAL INPUT for the momentary switches.  
  
Analog pin 5 for the potentiometer.  
  
This project can also be used with Valeton GP-200 or GP-5 as the MIDI messages are almost the same, requiring a small modification to the code regarding the MIDI CC numbers.

<img width="1000" height="1000" alt="valeton_gp100_midi_arduino_schematic_v01" src="https://github.com/user-attachments/assets/90a912d9-0c29-47a9-84de-8ccf6c587e42" />

