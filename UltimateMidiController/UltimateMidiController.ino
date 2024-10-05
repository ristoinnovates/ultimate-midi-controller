
#include <Arduino.h>
#include "MIDIUSB.h"
#include <Encoder.h>
#include "Keyboard.h"
// Display Libraries
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>

// For the Adafruit 0.96" TFT with ST7735
#define TFT_CS     18  // Chip select line for TFT (can be any digital pin)
#define TFT_RST    14  // Reset line for TFT (or connect to Arduino RESET pin)
#define TFT_DC     10  // Data/command line for TFT (can be any digital pin)
#define TFT_MOSI   16  // Data out (MOSI) pin for SPI
#define TFT_SCLK   15  // Clock out (SCLK) pin for SPI

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


// PINOUT
// MUX
// Outputs
int muxChannel1 = 8;
int muxChannel2 = 9;
int muxChannel3 = 4;
int muxChannel4 = 5;

// Inputs
int muxPotsInput = A7; // 8 channel analog
int muxSwitchesInput1 = 7; // 16 channel digital
int muxSwitchesInput2 = 19; // 16 channel digital
int muxPushButtonsInput = 20; // 8 channel digital


// # MULTIPLEXER 1 BUTTONS - 8 CH

const int NUMBER_MUX_1_BUTTONS = 8;
bool muxButtons1CurrentState[NUMBER_MUX_1_BUTTONS] = {0};
bool muxButtons1PreviousState[NUMBER_MUX_1_BUTTONS] = {0};

unsigned long lastDebounceTimeMUX1[NUMBER_MUX_1_BUTTONS] = {0};
unsigned long debounceDelayMUX1 = 5;

// # -------------------

// # MULTIPLEXER 2 BUTTONS - 16 CH

const int NUMBER_MUX_2_BUTTONS = 16;
bool muxButtons2CurrentState[NUMBER_MUX_2_BUTTONS] = {0};
bool muxButtons2PreviousState[NUMBER_MUX_2_BUTTONS] = {0};

unsigned long lastDebounceTimeMUX2[NUMBER_MUX_2_BUTTONS] = {0};
unsigned long debounceDelayMUX2 = 5;

// # -------------------

// # MULTIPLEXER 3 BUTTONS - 16 CH

const int NUMBER_MUX_3_BUTTONS = 16;
bool muxButtons3CurrentState[NUMBER_MUX_3_BUTTONS] = {0};
bool muxButtons3PreviousState[NUMBER_MUX_3_BUTTONS] = {0};

unsigned long lastDebounceTimeMUX3[NUMBER_MUX_3_BUTTONS] = {0};
unsigned long debounceDelayMUX3 = 5;

// # -------------------


// # MULTIPLEXER 4 POTS - 8 CH

const int NUMBER_MUX_4_POTS = 6;
int muxPots4CurrentState[NUMBER_MUX_4_POTS] = {0};
int muxPots4PreviousState[NUMBER_MUX_4_POTS] = {0};
int muxPots4PreviousMidiState[NUMBER_MUX_4_POTS] = {0}; // To store previous MIDI states

unsigned long lastDebounceTimeMUX4[NUMBER_MUX_4_POTS] = {0};
const unsigned long debounceDelayMUX4 = 5;
const int varThreshold = 10;

// # -------------------

// Encoder
// switching the encoder from 19 and 20 to 8 and 9, and switching the muxSwitchesInput2 and muxPushButtonsInput form 8 and 9 to 19 and 20
int encoderPinA = 2;
int encoderPinB = 3;

Encoder myEncoder(encoderPinA, encoderPinB); // Use the Encoder library for easy handling
int oldEncoderPos = 0; // To track changes in encoder position






int selectedChannel = 0;
int octave = 0;
int notesOctave = 0;


// Define menu items
const int totalMenuItems = 3;
String menuItems[totalMenuItems] = {"INSTRUMENT", "FX CTRL", "MIXER"};
int currentMenuIndex = 0;
bool inMenu = false;

void setup() {
  // Initialize serial communication for debugging (optional)
  Serial.begin(9600);

  // Configure multiplexer output pins
  pinMode(muxChannel1, OUTPUT);
  pinMode(muxChannel2, OUTPUT);
  pinMode(muxChannel3, OUTPUT);
  pinMode(muxChannel4, OUTPUT);
  digitalWrite(muxChannel1, LOW);
  digitalWrite(muxChannel2, LOW);
  digitalWrite(muxChannel3, LOW);
  digitalWrite(muxChannel4, LOW);

  // Set up multiplexer input pins
  pinMode(muxPotsInput, INPUT); // Analog input for potentiometers
  pinMode(muxSwitchesInput1, INPUT_PULLUP); // Digital input for the first set of switches
  pinMode(muxSwitchesInput2, INPUT_PULLUP); // Digital input for the second set of switches
  pinMode(muxPushButtonsInput, INPUT_PULLUP); // Digital input for push buttons
  //
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Initialize the display
  tft.initR(INITR_GREENTAB);

  tft.setRotation(-1);
  tft.invertDisplay(1);
  tft.fillScreen(ST7735_BLACK);

  tft.setTextColor(ST7735_ORANGE);
  tft.setTextSize(2);
  tft.setCursor(22, 20);
  tft.println("Risto");
  tft.setCursor(20, 40);
  tft.println("Innovates");

  // Print setup completion message
  Serial.println("Setup complete.");
  delay(2000);
  updateDisplay(oldEncoderPos, currentMenuIndex);  // Initial display update

}

void loop() {

  updateEncoder();
  updateMUX1PushButtons();
  updateMUX2Buttons();
  updateMUX3Buttons();
  updateMUX4Pots();

  //    delay(50); // Reduce output speed
}

void updateMUX1PushButtons() {
  for (int i = 0; i <  NUMBER_MUX_1_BUTTONS; i++) {
    //// Select channel of the multiplexer
    int A = bitRead(i, 0); //Take first bit from binary value of i channel.
    int B = bitRead(i, 1); //Take second bit from binary value of i channel.
    int C = bitRead(i, 2); //Take third bit from value of i channel.
    digitalWrite(muxChannel1, A);
    digitalWrite(muxChannel2, B);
    digitalWrite(muxChannel3, C);
    delay(1);
    
    muxButtons1CurrentState[i] = digitalRead(muxPushButtonsInput);

    if ((millis() - lastDebounceTimeMUX1[i]) > debounceDelayMUX1) {
      if (muxButtons1CurrentState[i] != muxButtons1PreviousState[i]) {
        lastDebounceTimeMUX1[i] = millis();

        if (muxButtons1CurrentState[i] == LOW) {
          pressButton(1, i, 0);
          Serial.print("Mux 1 Button: ");
          Serial.println(i);
        } else {
          pressButton(1, i, 1);
        }

        muxButtons1PreviousState[i] = muxButtons1CurrentState[i];
      }
    }
  }
}

void updateMUX2Buttons() {
  for (int i = 0; i <  NUMBER_MUX_2_BUTTONS; i++) {
    //// Select channel of the multiplexer
    int A = bitRead(i, 0); //Take first bit from binary value of i channel.
    int B = bitRead(i, 1); //Take second bit from binary value of i channel.
    int C = bitRead(i, 2); //Take third bit from value of i channel.
    int D = bitRead(i, 3); //Take third bit from value of i channel.
    digitalWrite(muxChannel1, A);
    digitalWrite(muxChannel2, B);
    digitalWrite(muxChannel3, C);
    digitalWrite(muxChannel4, D);
    delay(1);
    muxButtons2CurrentState[i] = digitalRead(muxSwitchesInput1);

    if ((millis() - lastDebounceTimeMUX2[i]) > debounceDelayMUX2) {
      if (muxButtons2CurrentState[i] != muxButtons2PreviousState[i]) {
        lastDebounceTimeMUX2[i] = millis();

        if (muxButtons2CurrentState[i] == LOW) {
          pressButton(2, i, 0);
          Serial.print("Mux 2 Button: ");
          Serial.println(i);
        } else {
          pressButton(2, i, 1);
        }

        muxButtons2PreviousState[i] = muxButtons2CurrentState[i];
      }
    }
  }
}

void updateMUX3Buttons() {
  for (int i = 0; i <  NUMBER_MUX_3_BUTTONS; i++) {
    //// Select channel of the multiplexer
    int A = bitRead(i, 0); //Take first bit from binary value of i channel.
    int B = bitRead(i, 1); //Take second bit from binary value of i channel.
    int C = bitRead(i, 2); //Take third bit from value of i channel.
    int D = bitRead(i, 3); //Take third bit from value of i channel.
    digitalWrite(muxChannel1, A);
    digitalWrite(muxChannel2, B);
    digitalWrite(muxChannel3, C);
    digitalWrite(muxChannel4, D);
    delay(1);
    muxButtons3CurrentState[i] = digitalRead(muxSwitchesInput2);

    if ((millis() - lastDebounceTimeMUX3[i]) > debounceDelayMUX3) {
      if (muxButtons3CurrentState[i] != muxButtons3PreviousState[i]) {
        lastDebounceTimeMUX3[i] = millis();

        if (muxButtons3CurrentState[i] == LOW) {
          pressButton(3, i, 0);
          Serial.print("Mux 3 Button: ");
          Serial.println(i);
        } else {
          pressButton(3, i, 1);
        }

        muxButtons3PreviousState[i] = muxButtons3CurrentState[i];
      }
    }
  }
}

void updateMUX4Pots() {
  for (int i = 0; i < NUMBER_MUX_4_POTS; i++) {
    digitalWrite(muxChannel1, bitRead(i, 0));
    digitalWrite(muxChannel2, bitRead(i, 1));
    digitalWrite(muxChannel3, bitRead(i, 2));
    delay(1);  // Minimize this delay if response is too slow

    int potValue = analogRead(muxPotsInput);
    int midiValue = map(potValue, 0, 1023, 127, 0);
    if (i == 1 || i == 2) {
      midiValue = map(potValue, 0, 1023, 0, 127);
    } 
    

    if (i == 4) {  // Assuming channel 4 is for modulation
      midiValue = map(potValue, 0, 1023, 127, 0);
      int correctedValueMod = potValue - 511;  // Adjusted to your actual midpoint
      midiValue = map(correctedValueMod, -511, 512, 127, 0);
      
    } else if (i == 5) {  // Assuming channel 5 is for pitch bend
      int correctedValuePitch = potValue - 511;  // Adjusted to your actual midpoint
      midiValue = map(correctedValuePitch, -512, 512, -8192, 8191);
    }

    if (abs(potValue - muxPots4PreviousState[i]) > varThreshold) {
      unsigned long currentTime = millis();
      if ((currentTime - lastDebounceTimeMUX4[i]) > debounceDelayMUX4) {
        lastDebounceTimeMUX4[i] = currentTime;

        if (midiValue != muxPots4PreviousMidiState[i]) {
          Serial.print("Pot ");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(midiValue);
          switch (i){
            case 0:
              // left slider
              midiControlChange(selectedChannel, 80, midiValue);
              break;
            case 1:
              // left pot
              midiControlChange(selectedChannel, 82, midiValue);
              break;
            case 2:
              // right pot
              midiControlChange(selectedChannel, 83, midiValue);
              break;
            case 3:
              // right slider
              midiControlChange(selectedChannel, 81, midiValue);
              break;
            case 4:
              // Modulation
              midiControlChange(selectedChannel, 1, midiValue);
              break;
            case 5:
              // Pitch Bend
              midiPitchBend(selectedChannel, midiValue + 8192);  // Normalize the pitch
              break;
          }

          muxPots4PreviousState[i] = potValue;
          muxPots4PreviousMidiState[i] = midiValue;
        }
      }
    }
  }
}

void updateEncoder() {
  int newEncoderPos = myEncoder.read() / 2; // Read the current encoder position
  if (newEncoderPos != oldEncoderPos) {
//    Serial.print("Old Pos: ");
//    Serial.print(oldEncoderPos);
//    Serial.print(" New Pos: ");
//    Serial.println(newEncoderPos);

    // Calculate direction of rotation
    int direction = (newEncoderPos > oldEncoderPos) ? 1 : -1;
    if (!inMenu){
      currentMenuIndex += direction;

      // Wrap around the menu index
      if (currentMenuIndex >= totalMenuItems) currentMenuIndex = 0;
      else if (currentMenuIndex < 0) currentMenuIndex = totalMenuItems - 1;
  
        updateDisplay(oldEncoderPos, currentMenuIndex);  
      } else {
        if (currentMenuIndex == 0){
          if (direction == -1) {
            Keyboard.press(91);
            Keyboard.release(91);
          } else {
            Keyboard.press(93);
            Keyboard.release(93);
          }
        }
      }
    
    oldEncoderPos = newEncoderPos; // Update the last known position
  }
}

void updateDisplay(int oldIndex, int newIndex) {
  Serial.print("Updating display from ");
  Serial.print(oldIndex);
  Serial.print(" to ");
  Serial.println(newIndex);

  if (inMenu){
    switch (newIndex) {
      case 0:
        tft.fillScreen(ST7735_BLACK);  // Clear screen before updating new values
        tft.setTextColor(ST7735_ORANGE, ST7735_BLACK);
        tft.setTextSize(2);
        tft.setCursor(5, 0);
        tft.println("< INSTRUMENT");
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.setTextSize(1);
        tft.setCursor(5, 20);
        tft.print("Octave: ");
        tft.print(octave);
        tft.print("  Transpose: ");
        tft.println(notesOctave);
        tft.setCursor(5, 20 + 40);
        tft.println("test 3");
        break;
      case 1:
        tft.fillScreen(ST7735_BLACK);  // Clear screen before updating new values
        tft.setTextColor(ST7735_ORANGE, ST7735_BLACK);
        tft.setTextSize(2);
        tft.setCursor(5, 0);
        tft.println("< FX CTRL");
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.setCursor(5, 20);
        tft.println("test 4");
        tft.setCursor(5, 20 + 20);
        tft.println("test 5");
        tft.setCursor(5, 20 + 40);
        tft.println("test 6");
        break;
      case 2:
        tft.fillScreen(ST7735_BLACK);  // Clear screen before updating new values
        tft.setTextColor(ST7735_ORANGE, ST7735_BLACK);
        tft.setTextSize(2);
        tft.setCursor(5, 0);
        tft.println("< MIXER");
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.setCursor(5, 20);
        tft.println("test 7");
        tft.setCursor(5, 20 + 20);
        tft.println("test 8");
        tft.setCursor(5, 20 + 40);
        tft.println("test 9");
        break;
        break;
    }
  } else {
    tft.fillScreen(ST7735_BLACK);  // Clear screen before updating new values
    tft.setTextColor(ST7735_ORANGE, ST7735_BLACK);
    tft.setTextSize(2);
    tft.setCursor(5, 0);
    tft.println("Select MODE:");
    tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
    for (int i = 0; i < totalMenuItems; i++) {
      tft.setCursor(5, 20 + i * 20);
      if (i == newIndex) {
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        tft.print("* ");
        tft.println(menuItems[i]);
      } else {
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.print("  ");
        tft.println(menuItems[i]);
      }
    }
  }

  
}

void midiNoteOn(byte channel, unsigned short note) {
  midiEventPacket_t midiNoteOn = { 0x09, 0x90 | channel, note, 100 };
  MidiUSB.sendMIDI(midiNoteOn);
  MidiUSB.flush();  // Ensure the message is sent immediately
}

void midiNoteOff(byte channel, unsigned short note) {
  midiEventPacket_t midiNoteOff = { 0x08, 0x80 | channel, note, 0 };
  MidiUSB.sendMIDI(midiNoteOff);
  MidiUSB.flush();  // Ensure the message is sent immediately
}

void midiControlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = { 0x0B, 0xB0 | channel, control, value };
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();  // Ensure the message is sent immediately
}

void midiPitchBend(byte channel, int bendValue) {
  // Ensure the bendValue is within the 14-bit range
  bendValue = constrain(bendValue, 0, 16383);

  // Split the 14-bit value into two 7-bit values
  byte lowByte = bendValue & 0x7F;         // Mask for the lower 7 bits
  byte highByte = (bendValue >> 7) & 0x7F; // Shift down by 7 bits and mask for the lower 7 bits

  // Form the MIDI event packet for pitch bend
  midiEventPacket_t event = {0x0E, 0xE0 | channel, lowByte, highByte};

  // Send the MIDI message
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();  // Ensure the message is sent immediately
}

void pressButton(int muxNumber, int buttonNumber, int state) {
  // MUX NUMBER 
  // 1 - Push Buttons
  // 2 - Top row plus encoder button
  // 3 - Bottom row plus joystick button

  // state 0 = button Pressed
  // state 1 = button Released
  int midiNote = 36; // starting midi note
  int n; // note + current octave increase
  if (muxNumber == 1) {
    // pushbuttons
    Serial.println(buttonNumber);
    switch (buttonNumber) {
      case 0:
        if (state == 0) {
          midiControlChange(selectedChannel, 70, 127);
        } else {
          midiControlChange(selectedChannel, 70, 0);
        }
        break;
      case 1:
        // Default: Channel Solo
        // Instrument: Octave Down
        // FX CTRL: CC NUMBER
        // MIXER: CC NUMBER
        if (state == 0) {
          midiControlChange(selectedChannel, 71, 127);
        } else {
          midiControlChange(selectedChannel, 71, 0);
        }
        break;
      case 2:
        // Default: Channel Mute
        // Instrument: CC NUMBER
        // FX CTRL: CC NUMBER
        // MIXER: CC NUMBER
        if (state == 0) {
          midiControlChange(selectedChannel, 72, 127);
        } else {
          midiControlChange(selectedChannel, 72, 0);
        }
        break;
      case 3:
        if (state == 0) {
          midiControlChange(selectedChannel, 73, 127);
        } else {
          midiControlChange(selectedChannel, 73, 0);
        }
        break;
      case 4:
        // Default: Channel Record
        // Instrument: Octave Down
        // FX CTRL: CC NUMBER
        // MIXER: CC NUMBER
        
        if (state == 0) {
          if (inMenu) {
            switch (currentMenuIndex) {
              case 0: // instrument
                if (octave > -3){
                  octave--;
                }
                updateDisplay(oldEncoderPos, currentMenuIndex);  
                break;
              case 1: // fx control 
                break;
              case 2: // mixer
                break;
            }
          } else {
            midiControlChange(selectedChannel, 74, 127);  
          }
          
        } else {
          midiControlChange(selectedChannel, 74, 0);
        }
        break;
      case 5:
        // Default: Channel Input Monitor
        // Instrument: Transpose Up
        // FX CTRL: CC NUMBER
        // MIXER: CC NUMBER
        if (state == 0) {
          if (inMenu) {
            switch (currentMenuIndex) {
              case 0: // instrument
                
                notesOctave++;
                updateDisplay(oldEncoderPos, currentMenuIndex);  
                break;
              case 1: // fx control 
                break;
              case 2: // mixer
                break;
            }
          } else {
            midiControlChange(selectedChannel, 74, 127);  
          } 
        }
          else {
          midiControlChange(selectedChannel, 75, 0);
        }
        break;
      case 6:
      // Default: Channel Input Monitor
        // Instrument: Octave Up
        // FX CTRL: CC NUMBER
        // MIXER: CC NUMBER
        if (state == 0) {
          if (inMenu) {
            switch (currentMenuIndex) {
              case 0: // instrument
                if (octave < 4){
                  octave++;
                  updateDisplay(oldEncoderPos, currentMenuIndex);  
                }
                
                break;
              case 1: // fx control
                
                break;
              case 2: // mixer
                break;
            }
          } else {
            midiControlChange(selectedChannel, 74, 127);  
          }
        } else {
          midiControlChange(selectedChannel, 76, 0);
        }
        break;
      case 7:
        // Default: Channel Input Monitor
        // Instrument: Transpose Down
        // FX CTRL: CC NUMBER
        // MIXER: CC NUMBER
        if (state == 0) {
          if (inMenu) {
            switch (currentMenuIndex) {
              case 0: // instrument
                notesOctave--;
                updateDisplay(oldEncoderPos, currentMenuIndex);  
                break;
              case 1: // fx control 
                break;
              case 2: // mixer
                break;
            }
          } else {
            midiControlChange(selectedChannel, 74, 127);  
          }
        } else {
          midiControlChange(selectedChannel, 77, 0);
        }
        break;
    }
  } 
 else if (muxNumber == 2) {
  if (buttonNumber <= 7){
    n = midiNote + buttonNumber + 15 + notesOctave + 12*octave;
    if (state == 0) {
      midiNoteOn(selectedChannel, n);
    } else {
      midiNoteOff(selectedChannel, n);
    }
  } else if (buttonNumber != 15) {
    switch (buttonNumber) {
      case 8:
        n = midiNote + 12 + 15 + notesOctave + 12*octave;
        break;
      case 9:
        n = midiNote + 9 + 15 + notesOctave + 12*octave;
        break;
      case 10:
        n = midiNote + 8 + 15 + notesOctave + 12*octave;
        break;
      case 11:
        n = midiNote + 10 + 15 + notesOctave + 12*octave;
        break;
      case 12:
        n = midiNote + 14 + 15 + notesOctave + 12*octave;
        break;
      case 13:
        n = midiNote + 13 + 15 + notesOctave + 12*octave;
        break;
      case 14:
        n = midiNote + 11 + 15 + notesOctave + 12*octave;
        break;
      case 15:
        break;
    }
    if (state == 0) {
      midiNoteOn(selectedChannel, n);
    } else {
      midiNoteOff(selectedChannel, n);
    }
  } else {
    // ENCODER BUTTON
    if (state == 0) {
      if (inMenu){
        inMenu = false;
      } else {
        inMenu = true; 
      }
      updateDisplay(oldEncoderPos, currentMenuIndex);
//      midiControlChange(selectedChannel, 79, 127);
    } else {
//      midiControlChange(selectedChannel, 79, 0);
    }
  }
  
 }
  else if (muxNumber == 3) {
    if (buttonNumber == 0){
      if (state == 0) {
          midiControlChange(selectedChannel, 78, 127);
        } else {
          midiControlChange(selectedChannel, 78, 0);
        }
    } else {
      n = midiNote + buttonNumber - 1 + notesOctave + 12*octave;
      if (state == 0) {
        midiNoteOn(selectedChannel, n);
      } else {
        midiNoteOff(selectedChannel, n);
      }
    }
    
  }
}
