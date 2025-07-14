Controlling VALETON GP-100 with Arduino and MIDI

Controlling VALETON GP-100 with Arduino leonardo and MIDI usinng a USB HOST SHIELD to conect the GP-100 to Arduino with GP-100 USB-B port.

The original project uses 8 momentary switches but 10 can be used with code modification.

The project also uses 1 analog input to read a potentiometer to be used as Expession Pedal.

The Pinout is as folow:

Pin 2 to 8 and pin 12 as DIGITAL INPUT for the momentary switches.

Analog pin 5 for the potentiometer.

All digital pins uses a 10k pull-up resistor before connecting to the corresponding input.

This project can also be used with Valeton GP-200 or GP-5 as the MIDI messages are almost the same, requring a smal modification to the code regarding the MIDI CC numbers.
