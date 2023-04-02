#include <Arduino.h>

// Define enum for message types
enum MidiMessageTypes {
  Invalid = 0x00,                 // zero as invalid message
  NoteOff = 0x80,                 // Data byte 1: note number, Data byte 2: velocity
  NoteOn = 0x90,                  // Data byte 1: note number, Data byte 2: velocity
  PolyphonicAftertouch = 0xA0,    // Data byte 1: note number, Data byte 2: pressure
  ControlChange = 0xB0,           // Data byte 1: controller number, Data byte 2: controller value
  ProgramChange = 0xC0,           // Data byte 1: program number
  ChannelAftertouch = 0xD0,       // Data byte 1: pressure
  PitchBend = 0xE0,               // Data byte 1: LSB, Data byte 2: MSB (14-bit value)
  SystemExclusive = 0xF0,         // Data bytes: variable length (sysex message)
  TimeCodeQuarterFrame = 0xF1,    // Data byte 1: time code value
  SongPositionPointer = 0xF2,     // Data byte 1: LSB, Data byte 2: MSB (14-bit value)
  SongSelect = 0xF3,              // Data byte 1: song number
  TuneRequest = 0xF6,             // No data bytes
  EndOfExclusive = 0xF7,          // No data bytes (end of sysex message)
  TimingClock = 0xF8,             // No data bytes
  Start = 0xFA,                   // No data bytes
  Continue = 0xFB,                // No data bytes
  Stop = 0xFC,                    // No data bytes
  ActiveSensing = 0xFE,           // No data bytes
  Reset = 0xFF                    // No data bytes
};

class MidiReader {
  private:
    // Callback function to notify the caller about received midi messages
    void (*onNewMidi)(MidiMessageTypes, int, int, int);

    // Buffer for incoming midi messages
    int midiBuffer[3];
    uint8_t bufferIndex = 0;

  public:
    bool running = true; //is false after midi stop message, and back true after start/continue
    MidiReader(){
      Serial.begin(31250);
    }

    MidiReader(void (*newMidiCallback)(MidiMessageTypes, int, int, int)) {
      onNewMidi = newMidiCallback;
    }

    // Method to be called in the main loop
    void update() { 
      while (Serial.available() > 0) {
        uint8_t incomingByte = Serial.read();
        // Serial.print(incomingByte, HEX);
        // Serial.print("\n");
        if(incomingByte & 0x80){
          //is command byte, MSB == 1
          midiBuffer[0] = incomingByte;
          bufferIndex = 1;
        } else {
          // databyte, MSB == 0
          if(bufferIndex > 0){
            midiBuffer[bufferIndex] = incomingByte;
            bufferIndex++;
          } else {
            //error, databyte recieved without command byte
          }
        }
        int val = midiBuffer[0];
        MidiMessageTypes type = 0;
        uint8_t channel = 0;
        //check if message complete
        if(midiBuffer[0] >= 0xF0){
          //is System Message 0xF0..0xFF
          type = (MidiMessageTypes)midiBuffer[0];
        } else{
          //is channel message
          type = (MidiMessageTypes)(midiBuffer[0] & 0xF0);
          channel = midiBuffer[0] & 0x0F;
        }
        switch(type){
          case Invalid:
            //Invalid Message
          break;

          case NoteOn:
          case NoteOff:
          case PolyphonicAftertouch:
          case ControlChange:
          case PitchBend:
          case SongPositionPointer:
          case SystemExclusive:
            //messages that require 2 data bytes
            if(bufferIndex == 3){
              //enough data for NoteOn Message available
              onNewMidi(type, channel, midiBuffer[1], midiBuffer[2]); 
              bufferIndex = 0;             
            }
          break;

          case ProgramChange:
          case ChannelAftertouch:
          case SongSelect:
          case TimeCodeQuarterFrame:
            //messages that require 1 data byte
            if(bufferIndex == 2){
              //enough data for NoteOn Message available
              onNewMidi(type, channel, midiBuffer[1], 0);  
              bufferIndex = 0;            
            }
          break;

          case Reset:
          case ActiveSensing:
          case Stop:
          case Start:
          case Continue:
          case TimingClock:
          case EndOfExclusive:
          case TuneRequest:
            if(bufferIndex == 2){
              //messages that require no data bytes
              onNewMidi(type, 0, 0, 0);
              bufferIndex = 0;
              if(type == Start || type == Continue){
                running = true;
              } else if (type == Stop){
                running = false;
              }              
            }
          break;

          default:
            // //Unknown message recieved
            // Serial.print("unknown Message: ");
            // Serial.print(type, HEX);
            // Serial.print("\n");
          break;
          
        }
      }
    }
};

class PWMGenerator {
  public:
    PWMGenerator(uint8_t outputPin) : _outputPin(outputPin) {
      pinMode(_outputPin, OUTPUT);
    }

    void setFrequency(unsigned long frequency) {
      _frequency = frequency;
      _updateTimerRegisters();
    }

    void setDutyCycle(uint8_t dutyCycle) {
      _dutyCycle = dutyCycle;
      _updateTimerRegisters();
    }

    void enable(bool enbl) {
      _enabled = enbl;
      _updateTimerRegisters();
    }

    void disable() {
      _enabled = false;
      _updateTimerRegisters();
    }

  private:
    uint8_t _outputPin;
    unsigned long _frequency = 1000;
    uint8_t _dutyCycle = 50;
    bool _enabled = false;

    void _updateTimerRegisters() {
      if (_enabled) {
        uint32_t prescaler;
        uint8_t prescalerBits;
        uint32_t topValue;
        uint32_t compareValue;

        if (_frequency <= 122) {
          prescaler = 1024;
          prescalerBits = _BV(CS12) | _BV(CS10);
        } else if (_frequency <= 244) {
          prescaler = 256;
          prescalerBits = _BV(CS12);
        } else if (_frequency <= 1953) {
          prescaler = 64;
          prescalerBits = _BV(CS11) | _BV(CS10);
        } else if (_frequency <= 62500) {
          prescaler = 8;
          prescalerBits = _BV(CS11);
        } else {
          prescaler = 1;
          prescalerBits = _BV(CS10);
        }

        topValue = (F_CPU / (2 * prescaler * _frequency)) - 1;
        compareValue = topValue * _dutyCycle / 100;

        TCCR1A = _BV(COM1A1) | _BV(WGM11);
        TCCR1B = _BV(WGM13) | _BV(WGM12) | prescalerBits;
        ICR1 = topValue;
        OCR1A = compareValue;
      } else {
        TCCR1A = 0;
        TCCR1B = 0;
      }
    }
};

enum Note {
  C,
  Cs,
  D,
  Ds,
  E,
  F,
  Fs,
  G,
  Gs,
  A,
  As,
  B
};

float noteFrequency(Note note, uint8_t octave) {
  // A4 is defined as 440 Hz
  const float referenceFrequency = 440.0;
  // A4 is the 9th note in our enumeration and belongs to the 4th octave
  const int referenceNoteIndex = 9;
  const int referenceOctave = 4;

  int noteIndex = static_cast<int>(note);
  int totalHalfSteps = (octave - referenceOctave) * 12 + (noteIndex - referenceNoteIndex);
  float frequency = referenceFrequency * pow(2, static_cast<float>(totalHalfSteps) / 12.0);

  return frequency;
};

// Create a PWMGenerator object using pin 9 (OC1A) for PWM output
PWMGenerator pwmGen(9);

MidiReader midi;

void onMidi(MidiMessageTypes type, int channel, int data1, int data2){
  // Serial.print("Type: ");
  // Serial.print(type, HEX);
  // Serial.print(" - Chanel: ");
  // Serial.print(channel, HEX);
  // Serial.print(" - data1: ");
  // Serial.print(data1, HEX);
  // Serial.print(" - data2: ");
  // Serial.print(data2, HEX);
  // Serial.print("\n");
  switch(type){
    case NoteOn:
      int nt = noteFrequency(((data1 % 12) + 12) % 12, data1 / 0x0C);
      pwmGen.setFrequency(nt);
      pwmGen.enable(data2 != 0x00);
    break;
    case NoteOff:
      pwmGen.setFrequency(noteFrequency(data1 / 0x0C, ((data1 % 12) + 12) % 12));
      pwmGen.enable(false);
    break;
  }
}

int lastMlls = 0;
int counter = 0;
int dutyc = 0;

void setup() {
  //setup midi reader
  Serial.begin(31250);
  MidiReader md(onMidi);
  midi = md;
  // Set the frequency and duty cycle, then enable the PWM output
  pwmGen.setFrequency(1000); // 1 kHz
  pwmGen.setDutyCycle(50); // 50% duty cycle
  pwmGen.enable(false);
}

void loop() {
  int deltat = millis() - lastMlls;
  lastMlls = millis();
  counter += deltat;
  if(counter >= 10){
    counter = 0;
    dutyc++;
    if(dutyc >= 100){
      dutyc = 0;
    }
    pwmGen.setDutyCycle(dutyc);
  }

  midi.update();

  //  for(int i = 0; i < 12; i++){
  //   pwmGen.setFrequency(noteFrequency(i, 4));
  //   delay(500);
  // }
}