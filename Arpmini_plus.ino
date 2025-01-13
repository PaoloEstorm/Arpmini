/*!
 *  @file       Arpmini_plus.ino
 *  Project     Estorm - Arpmini+
 *  @brief      MIDI Sequencer & Arpeggiator
 *  @version    2.02
 *  @author     Paolo Estorm
 *  @date       09/12/24
 *  @license    GPL v3.0 
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Inspired by the "arduino based midi sequencer" project by Brendan Clarke
// https://brendanclarke.com/wp/2014/04/23/arduino-based-midi-sequencer/

#include "Vocabulary.h"

// system
char version[] = "2.02";

// leds
#define redled 3     // red led pin
#define yellowled 2  // yellow led pin
#define greenled 4   // green led pin
#define blueled 5    // blue led pin

// sound
#define speaker 21          // buzzer pin
bool uisound = true;        // is ui sound on?
uint8_t soundmode = 1;      // 1 audio on, 2 ui sounds off, 3 off
bool sound = true;          // speaker sound toggle
bool metro = false;         // metronome toggle
bool confirmsound = false;  // at button press, play the confirmation sound instead of the click

// screen
#define USE_MICRO_WIRE
#define OLED_SPI_SPEED 8000000ul  // set SPI at 8MHz
#include "GyverOLED.h"
#define RES 19  // SPI reset pin
#define DC 20   // SPI DC pin
#define CS 18   // SPI CS pin

GyverOLED<SSD1306_128x64, OLED_NO_BUFFER, OLED_SPI, CS, DC, RES> oled;  // screen initialization

bool StartScreenTimer = true;  // activate the screen-on timer

// memory
#include <EEPROM.h>

// midi
#include <MIDI.h>
#include <SoftwareSerial.h>
SoftwareSerial Serial2(10, -1);                        // midi2 pins (input, output)
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);   // initialize midi1
MIDI_CREATE_INSTANCE(SoftwareSerial, Serial2, MIDI2);  // initialize midi2
uint8_t midiChannel = 1;                               // MIDI channel to use for sequencer 1 to 16
int8_t syncport = 1;                                   // activate midi-in sync 0=none, 1=port1, 2=port2

// buttons
#define redbutton 7     // red button pin
#define yellowbutton 6  // yellow button pin
#define greenbutton 8   // green button pin
#define bluebutton 9    // blue button pin

bool EnableButtons = true;  // if controlled externally (by CC) disable all buttons
uint8_t greentristate = 1;  // 0=off, 1=null, 2=on
bool redstate = false;      // is redbutton pressed?
bool yellowstate = false;   // is yellowbutton pressed?
bool greenstate = false;    // is greenbutton pressed?
bool bluestate = false;     // is bluebutton pressed?

uint8_t redbuttonCC;             // cc for external control
uint8_t yellowbuttonCC;          // cc for external control
uint8_t bluebuttonCC;            // cc for external control
uint8_t greenbuttonCC;           // cc for external control
int8_t numbuttonspressedCC = 0;  // number of externally pressed buttons

uint8_t mapButtonSelect = 1;  // 1=red, 2=yellow, 3=blue, 4=green

// menu
uint8_t menuitem = 2;              // is the number assiciated with every voice in the main menu. 0 to 16
uint8_t menunumber = 0;            // 0=mainscreen, 1=menu, 2=submenu, 3=load\save menu
uint8_t savemode = 0;              // 0=new, 1=save, 2=load, 3=delete
uint8_t savecurX = 0;              // cursor position in load/save menu 0-5
bool confirmation = false;         // confirmation popup in load/save menu
bool full = false;                 // is the selected save slot full?
int8_t curpos = 0;                 // cursor position in songmode, 0-7
bool PrintSongLiveCursor = false;  // send the cursor to screen, song & live mode
bool PrintTimeBar = false;         // send the BPM bar to screen
bool StartMenuTimer = true;        // activate the go-to-menu timer

// time keeping
volatile uint8_t clockTimeout;  // variable for keeping track of external/internal clock
uint8_t StepSpeed = 2;          // 1=2x, 2=1x, 3=3/4, 4=1/2
bool FixSync = false;           // tries to re-allign the sequence when changing step-speeds
bool internalClock = true;      // is the sequencer using the internal clock (otherwise external sync)?
bool flipflopEnable;            // part of the frameperstep flipflop
uint8_t sendrealtime = 1;       // send midi realtime messages. 0=off, 1=on (@internalclock, only if playing), 2=on (@internalclock, always)
int8_t GlobalStep;              // keep track of note steps only for metronome and tempo indicator in arp mode
int8_t tSignature = 4;          // time signature for led indicator/metronome and beats, 1 - 8 (1*/4, 2*/4..to 8/4)
int8_t GlobalDivison = 4;       // how many steps per beat
int8_t countBeat;               // keep track of the time beats
int8_t countStep;               // keep track of note steps in seq/song mode
int8_t IntercountStep;          // keep track of note steps while inter-recording
int8_t countTicks;              // the frame number (frame=external clock or 24 frames per quarter note)
uint8_t ticksPerStep = 6;       // how many clock ticks to count before the sequencer moves to another step.
uint8_t flip = 6;               // part of the frameperstep's flipflop
uint8_t flop = 6;               // part of the frameperstep's flipflop
bool swing = false;             // is swing enabled?
uint8_t BPM = 120;              // beats per minute for internalclock - min 20, max 250 bpm
bool playing = false;           // is the sequencer playing?
uint8_t snapmode = 1;           // when play/stop the next sequence in live mode. 0=pattern, 1=up-beat, 2=beat
bool start = false;             // dirty fix for a ableton live 10 bug. becomes true once at sequence start and send a sync command

// arpeggiator
const uint8_t holdNotes = 8;    // maximum number of notes that can be held at once
int8_t activeNotes[holdNotes];  // contains the MIDI notes the arpeggiator plays
bool ArpDirection;              // alternate up-down for arpstyle 3 & 4
uint8_t arpstyle = 1;           // 1=up, 2=down, 3=up-down, 4=down-up, 5=up+down, 6=down+up, 7=random
uint8_t arpcount;               // number of times the arpeggio repeats
uint8_t arprepeat = 1;          // number of times the arpeggios gets trasposed by "stepdistance"
int8_t stepdistance = 12;       // distance in semitones between each arp repeat, -12 to +12
int8_t numNotesHeld = 0;        // how many notes are currently pressed on the keyboard
int8_t numActiveNotes = 0;      // number of notes currently stored in the holdNotes array
uint8_t trigMode = 0;           // 0=hold, 1=trigger, 2=retrigged

// sequencer
bool recording = false;                         // is the sequencer in the recording state?
uint8_t rolandLowNote;                          // for 'roland' mode, the sequence is transposed based on the lowest note recorded
uint8_t rolandTransposeNote;                    // the last note number received for transposing the sequence
uint8_t seqLength = 16;                         // the number of steps the sequence is looped for - can be less than maxSeqLength
uint8_t NewSeqLength = 16;                      // 1-32 seq.length of the newly created song
uint8_t currentSeq = 0;                         // the currently selected sequence row from the big matrix below. 0=seq1, 1=seq2, 2=seq3, 3=seq4
uint8_t newcurrentSeq = 0;                      // the next sequence is going to play in live mode. 0-3
const uint8_t maxSeqLength = 32;                // the total possible number of steps (columns in the big matrix below)
const uint8_t numberSequences = 4;              // the number of total sequences
int8_t noteSeq[numberSequences][maxSeqLength];  // sequences arrays
const uint8_t patternLength = 8;                // number of patterns
uint8_t songPattern[patternLength];             // pattern array
uint8_t pattern = 0;                            // currently playing pattern
bool chainrec = false;                          // sequential recording in song mode?
bool lockpattern = false;                       // if true block the current pattern from sequencing

// notes
int8_t pitch = 0;                // pitch transposition: -12 to +12
bool pitchmode = false;          // 0=before scale, 1=scale root
uint8_t scale = 0;               // 0=linear, 1=penta. major, 2=penta. minor, 3=major, 4=minor, 5=arabic, 6=locrian, 7=lydian, 8=dorian, 9=inverted, 10=hexa.
int8_t posttranspose = 0;        // post-scale pitch transposition: -12 to +12
uint8_t noteLengthSelect = 3;    // set the notelength, 0=random, 1=10%, 2=30%, 3=50%, 4=80%, 5=100%, 6=120%
bool sortnotes = true;           // sort the ActiveNotes array?
bool muted = false;              // temporarily suspend playing any notes from the sequencer
bool TrigMute = false;           // toggle mute in sync with snapmode, in live mode
bool sustain = false;            // hold notes also if trigmode > 0
const uint8_t queueLength = 2;   // the number of notes that can overlap if notelenght is 120%
int8_t cronNote[queueLength];    // stores the notes playing
int8_t cronLength[queueLength];  // stores the amount of time a note has left to play

// general
uint8_t modeselect = 1;     // 1=arp.mode, 2=rec.mode, 3=song mode, 4=live mode
uint8_t premodeselect = 1;  // pre-selection of the mode in submenu

void setup() {  // initialization setup

  // oled screen initialization
  oled.init();
  oled.clear();
  oled.setScale(3);

  // disable onboard LEDs
  pinMode(LED_BUILTIN_TX, INPUT);
  pinMode(LED_BUILTIN_RX, INPUT);  // pin 17

  // initialize pins
  pinMode(redled, OUTPUT);
  pinMode(yellowled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);
  pinMode(speaker, OUTPUT);

  AllLedsOff();

  pinMode(redbutton, INPUT_PULLUP);
  pinMode(yellowbutton, INPUT_PULLUP);
  pinMode(bluebutton, INPUT_PULLUP);
  pinMode(greenbutton, INPUT_PULLUP);

  // initialize timer1 used for internal clock
  noInterrupts();
  TCCR1A = 0;               // set TCCR1A register to 0
  TCCR1B = 0;               // same for TCCR1B
  TCNT1 = 0;                // initialize counter value to 0
  OCR1A = 5208;             // initial tempo 120bpm
  TCCR1B |= (1 << WGM12);   // turn on CTC mode
  TCCR1B |= (1 << CS11);    // 64 prescaler
  TCCR1B |= (1 << CS10);    // 64 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();

  // midi initialization settings
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI2.begin(MIDI_CHANNEL_OMNI);

  MIDI.turnThruOff();
  MIDI2.turnThruOff();
  MIDI.setHandleControlChange(HandleCC);
  MIDI2.setHandleControlChange(HandleCC);
  MIDI.setHandleNoteOn(HandleNoteOn);
  MIDI2.setHandleNoteOn(HandleNoteOn);
  MIDI.setHandleNoteOff(HandleNoteOff);
  MIDI2.setHandleNoteOff(HandleNoteOff);
  MIDI.setHandleSongPosition(HandleSongPosition);
  MIDI2.setHandleSongPosition(HandleSongPosition);

  if (EEPROM.read(15) != 64) ResetEEPROM();  // if check is not valid, initialize the EEPROM with default values

  midiChannel = EEPROM.read(0);
  sendrealtime = EEPROM.read(1);
  soundmode = EEPROM.read(2);
  syncport = EEPROM.read(3);

  redbuttonCC = EEPROM.read(4);
  yellowbuttonCC = EEPROM.read(5);
  bluebuttonCC = EEPROM.read(6);
  greenbuttonCC = EEPROM.read(7);

  SetBPM(BPM);
  SetSound(soundmode);
  SetSyncPort(syncport);
  ClearSeqPatternArray();
  Startposition();
  SynthReset();

  Bip(2);  // Startup Sound
  oled.println(F("ARPMINI"));
  oled.print(F("   +"));
  oled.setScale(1);
  oled.setCursorXY(25, 56);
  oled.print(F("FIRMWARE "));
  oled.print(version);
  delay(2000);
  PrintMainScreen();
}

void loop() {  // run continuously

  MIDI.read();
  MIDI2.read();

  DebounceButtons();
  ScreenOnTimer();
  GoToMenuTimer();

  if (PrintSongLiveCursor) {
    PrintSongLiveCursor = false;
    PrintPatternSequenceCursor();
  }

  if (PrintTimeBar) {
    PrintTimeBar = false;
    PrintBPMBar();
  }
}

void ResetEEPROM() {  // initialize the EEPROM with default values

  for (uint8_t i = 0; i < 6; i++) {
    if (i < 4) {
      EEPROM.write(i, 1);           // midiChannel, sendrealtime, soundmode, syncport
      EEPROM.write(4 + i, i + 21);  // buttons cc
    }
    EEPROM.write(16 + (144 * i), 0);  // songs addresses
  }
  EEPROM.write(15, 64);  // write check
}

void AllLedsOn() {  // turn on all leds

  digitalWrite(redled, HIGH);
  digitalWrite(yellowled, HIGH);
  digitalWrite(blueled, HIGH);
  digitalWrite(greenled, HIGH);
}

void AllLedsOff() {  // turn of all leds

  digitalWrite(redled, LOW);
  digitalWrite(yellowled, LOW);
  digitalWrite(blueled, LOW);
  digitalWrite(greenled, LOW);
}

void GoToMenuTimer() {  // greenbutton longpress enter the menu

  static unsigned long SampleMenuTime;  // time in ms at which the timer was set
  static bool MenuTimerState;           // is the timer still running?

  if (StartMenuTimer) {
    StartMenuTimer = false;
    MenuTimerState = true;
    SampleMenuTime = millis();
  }

  if (MenuTimerState) {
    if (greentristate == 2) {
      if (millis() - SampleMenuTime > 1000) {
        MenuTimerState = false;
        greentristate = 1;
        Bip(2);
        if (modeselect == 4) menuitem = 2;
        PrintMenu(menuitem);
        if (recording) {
          recording = false;
          ManageRecording();
        }
      }
    }
  }
}

void ScreenOnTimer() {  // in the main screen if greenbutton is held for 4 seconds, go to menu

  static unsigned long SampleTime = 0;  // Time at which the timer was set
  static bool TimerState = false;       // Is the timer still running?

  // Start the timer
  if (StartScreenTimer) {
    StartScreenTimer = false;
    TimerState = true;
    SampleTime = millis();
    oled.sendCommand(OLED_DISPLAY_ON);
    oled.setContrast(255);
  }

  // If the timer is running, check if it has expired
  if (TimerState && millis() - SampleTime > 4000) {
    TimerState = false;

    // Check if the screen should be turned off or dimmed
    if (menunumber == 0 && ((modeselect != 3) || (modeselect == 3 && !playing && internalClock))) {
      oled.sendCommand(OLED_DISPLAY_OFF);
    } else {
      oled.setContrast(1);
    }
  }
}

void SynthReset() {  // clear cache

  for (uint8_t i = 0; i < holdNotes; i++) {
    activeNotes[i] = -1;
    if (i < queueLength) {
      cronNote[i] = -1;
      cronLength[i] = -1;
    }
  }
  numNotesHeld = 0;
  AllNotesOff();
}

void AllNotesOff() {  // send allnoteoff control change

  MIDI.sendControlChange(123, 0, midiChannel);
}

void ConnectPort(uint8_t portselect) {  // connects sync ports

  if (portselect == 1) {
    MIDI.setHandleClock(HandleClock);
    MIDI.setHandleStart(HandleStart);
    MIDI.setHandleStop(HandleStop);
    MIDI.setHandleContinue(HandleContinue);
  } else if (portselect == 2) {
    MIDI2.setHandleClock(HandleClock);
    MIDI2.setHandleStart(HandleStart);
    MIDI2.setHandleStop(HandleStop);
    MIDI2.setHandleContinue(HandleContinue);
  }
}

void DisconnectPort(uint8_t portselect) {  // disconnects sync ports

  if (portselect == 1) {
    MIDI.disconnectCallbackFromType(midi::Clock);
    MIDI.disconnectCallbackFromType(midi::Start);
    MIDI.disconnectCallbackFromType(midi::Stop);
    MIDI.disconnectCallbackFromType(midi::Continue);
  } else if (portselect == 2) {
    MIDI2.disconnectCallbackFromType(midi::Clock);
    MIDI2.disconnectCallbackFromType(midi::Start);
    MIDI2.disconnectCallbackFromType(midi::Stop);
    MIDI2.disconnectCallbackFromType(midi::Continue);
  }
}

void SetSyncPort(uint8_t port) {  // 0-none, 1-port1, 2-port2 define port for external sync

  DisconnectPort(1);
  DisconnectPort(2);

  if (port == 1) {
    ConnectPort(1);
  } else if (port == 2) {
    ConnectPort(2);
  }
}

unsigned short int eepromaddress(unsigned short int address, uint8_t slot) {  // calculate eeprom addresses // (address, slot number 0-5)

  return (144 * slot) + address;
}

void ScreenBlink(bool mode = false) {  // blinks screen - 0 bip on button-press, 1 bip regardless

  oled.sendCommand(OLED_DISPLAY_OFF);
  oled.sendCommand(OLED_DISPLAY_ON);
  if (!mode) confirmsound = true;
  else Bip(2);
}

void SetSound(uint8_t mode) {  // sound settings

  if (mode == 1) {
    sound = true;
    uisound = true;
  } else if (mode == 2) {
    uisound = false;
    sound = true;
  } else if (mode == 3) sound = false;
}

void Bip(uint8_t type) {  // sounds

  if (sound) {
    if (uisound) {
      if (type == 1) tone(speaker, 3136, 1);        // click
      else if (type == 2) tone(speaker, 2637, 10);  // startup/confirmation
    }
    if (type == 3) tone(speaker, 3136, 10);      // metronome
    else if (type == 4) tone(speaker, 2349, 3);  // metronome
  }
}

void SetBPM(uint8_t tempo) {  // change Timer1 speed to match BPM, 20-250

  OCR1A = (25 * (25000 / tempo));  // 2.5*[(16MHz/prescaler)/BPM]
}

ISR(TIMER1_COMPA_vect) {  // internal clock

  static const uint8_t MaxClockTimeout = 100;

  if (internalClock) {
    if ((sendrealtime == 1 && playing) || (sendrealtime == 2)) MIDI.sendRealTime(midi::Clock);
    RunClock();
  } else {
    clockTimeout++;
    if (clockTimeout > MaxClockTimeout) {
      HandleStop();
    }
  }
}

void HandleClock() {  // external clock

  if (!internalClock) {
    clockTimeout = 0;
    RunClock();
    if (sendrealtime) MIDI.sendRealTime(midi::Clock);
  }
}

void RunClock() {  // main clock

  countTicks = (countTicks + 1) % ticksPerStep;

  if (countTicks == 0) {
    if (start) {
      start = false;
      if (sendrealtime) MIDI.sendSongPosition(0);  // make ableton live happy, without would be slightly out of sync
    }
    if ((modeselect != 4 && (!internalClock || (internalClock && playing))) || modeselect == 4) HandleStep();
  }

  if (countTicks == 2) {
    if (menunumber == 0) {
      if (modeselect != 4 && (!recording || (recording && playing))) digitalWrite(yellowled, LOW);  // turn yellowled off before 2 ticks - Tempo indicator
      else if (modeselect == 4) {
        AllLedsOff();
      }
    }
  }

  if (modeselect < 3) {  // not song & live mode
    if (trigMode > 0 && numNotesHeld == 0 && !sustain) muted = true;
  }

  for (uint8_t i = 0; i < queueLength; i++) {  // decrement crons
    if (cronLength[i] > -1) {
      cronLength[i] = cronLength[i] - SetNoteLength();  // this number is subtracted from the cronLength buffer for each frame a note is active
    }
    if (cronLength[i] < -1) {
      MIDI.sendNoteOff(cronNote[i], 0, midiChannel);
      cronLength[i] = -1;
      cronNote[i] = -1;
    }
  }
}

void SetTicksPerStep() {  // set ticksPerStep values
  switch (StepSpeed) {
    case 1:  // 2x
      flip = swing ? 4 : 3;
      flop = swing ? 2 : 3;
      GlobalDivison = 8;
      break;
    case 2:  // 1x
      flip = swing ? 8 : 6;
      flop = swing ? 4 : 6;
      GlobalDivison = 4;
      break;
    case 3:  // 3/4
      flip = 8;
      flop = 8;
      GlobalDivison = 3;
      break;
    case 4:  // 1/2
      flip = swing ? 16 : 12;
      flop = swing ? 8 : 12;
      GlobalDivison = 2;
      break;
  }
}

void FlipFlopFPS() {  // alternates ticksPerStep values (for swing)

  flipflopEnable = !flipflopEnable;
  if (flipflopEnable) ticksPerStep = flip;
  else ticksPerStep = flop;
}

uint8_t SetNoteLength() {  // set the note duration

  uint8_t noteLength;
  static const uint8_t noteLengths[] = { 55, 35, 30, 25, 20, 15 };  // noteLengths

  if (noteLengthSelect == 0) {  // random
    noteLength = random(15, 55);
  } else noteLength = noteLengths[noteLengthSelect - 1];

  if (swing && StepSpeed != 3) {
    if (flipflopEnable) noteLength = noteLength - 5;
    else noteLength = noteLength + 10;
  }

  if (StepSpeed == 1) noteLength = noteLength * 2;
  else if (StepSpeed == 3) noteLength = noteLength - 5;
  else if (StepSpeed == 4) noteLength = noteLength / 2;

  return (noteLength);
}

void HandleStep() {  // step sequencer

  GlobalStep++;
  if (GlobalStep > (GlobalDivison - 1)) {
    GlobalStep = 0;
  }

  if (FixSync) {
    if (GlobalStep == 0) {
      Startposition();
      FixSync = false;
    }
  } else SetTicksPerStep();

  FlipFlopFPS();

  if (modeselect == 1) {  // arp.mode

    if (numActiveNotes > 0) {
      SetArpStyle(arpstyle);  // arp sequencer
      if (!muted && playing) {
        QueueNote(activeNotes[countStep % numActiveNotes]);  // enqueue and play
      }
    } else countStep = -1;
  }

  else if (modeselect != 1) {  // rec. & song mode

    countStep = (countStep + 1) % seqLength;  // seq. sequencer

    if (modeselect == 2 || modeselect == 3) {
      if (playing && recording && bluestate && menunumber == 0) {  // in rec.mode bluebutton delete note
        noteSeq[currentSeq][countStep] = -1;
      }

      if (!playing && recording && internalClock) {  // in recording and not playing yellowled indicate steps
        if (countStep % 4 == 0) {
          digitalWrite(yellowled, HIGH);
        } else digitalWrite(yellowled, LOW);
      }

      if (modeselect == 3) {  // call pattern sequencer
        if (countStep == 0) {
          if ((!recording) || (recording && chainrec)) {
            HandlePattern();
          }
        }
      }
    }

    else if (modeselect == 4) {
      if ((snapmode == 0 && countStep == 0) || (snapmode == 1 && GlobalStep == 0 && countBeat == (tSignature - 1)) || (snapmode == 2 && GlobalStep == 0)) {

        if (TrigMute) {
          TrigMute = false;
          muted = !muted;
        }

        if (currentSeq != newcurrentSeq) {
          currentSeq = newcurrentSeq;
          muted = false;
          if (menunumber == 0) PrintSongLiveCursor = true;
        }
      }
    }

    if (playing && (noteSeq[currentSeq][countStep] >= 0) && !muted) {  // step has a note - enqueue and play
      int8_t note;
      if (recording) {
        note = noteSeq[currentSeq][countStep];
      } else note = (noteSeq[currentSeq][countStep] + rolandTransposeNote - rolandLowNote);
      QueueNote(note);
    }
  }

  if (GlobalStep == 0) {
    HandleBeat();
  }

  if (menunumber == 0 && modeselect == 4) {  // live mode blinking

    if (GlobalStep == 0 || (TrigMute && !(GlobalStep % 2))) {
      if (playing) {
        if (!muted || TrigMute) {
          if (currentSeq == 0) digitalWrite(redled, HIGH);
          else if (currentSeq == 1) digitalWrite(yellowled, HIGH);
          else if (currentSeq == 2) digitalWrite(blueled, HIGH);
          else if (currentSeq == 3) digitalWrite(greenled, HIGH);
        } else AllLedsOn();
      }
    }
  }
}

void HandleBeat() {  // tempo indicator and metronome

  countBeat = (countBeat + 1) % tSignature;

  if (menunumber == 0) {

    if (modeselect != 4) {  // turn yellowled on every beat
      if (playing) digitalWrite(yellowled, HIGH);
    }
  }

  if (playing && (recording || metro)) Metronome();
}

void HandlePattern() {  // pattern sequencer in songmode

  if (!lockpattern) {
    pattern = (pattern + 1) % patternLength;
    SkipPattern();
  }

  currentSeq = songPattern[pattern] - 1;

  if (menunumber == 0) PrintSongLiveCursor = true;
}

void SkipPattern() {  // skip pattern if empty

  if ((songPattern[pattern]) == 0) pattern = (pattern + 1) % patternLength;
  if ((songPattern[pattern]) == 0) SkipPattern();
}

void Metronome() {  // manage the metronome

  if (countBeat == 0) Bip(3);
  else Bip(4);
}

void HandleStart() {  // start message - re-start the sequence

  if (sendrealtime) {
    MIDI.sendRealTime(midi::Start);
  }

  internalClock = false;
  playing = true;
  Startposition();

  if (menunumber == 0) {  // switch on display and restart screen-on timer
    StartScreenTimer = true;
    PrintTimeBar = true;
  }
}

void HandleContinue() {  // continue message - re-start the sequence

  HandleStop();
  HandleStart();
}

void HandleStop() {  // stop the sequence and switch over to internal clock

  if (sendrealtime) MIDI.sendRealTime(midi::Stop);

  playing = false;
  internalClock = true;
  Startposition();
  numNotesHeld = 0;

  if (menunumber == 0) {  // switch on display and restart screen-on timer
    StartScreenTimer = true;
    PrintTimeBar = true;
  }
}

void StartAndStop() {  // manage starts and stops

  playing = !playing;

  if (internalClock) {
    Startposition();
    if (sendrealtime) {
      if (playing) {
        MIDI.sendRealTime(midi::Start);
        start = true;  // to make ableton live 10 happy
      } else MIDI.sendRealTime(midi::Stop);
    }
  }
}

void HandleSongPosition(unsigned int position) {  // send song position midi messages

  if (sendrealtime) MIDI.sendSongPosition(position);
}

void Startposition() {  // called every time the sequencing start or stop

  SetTicksPerStep();

  if (!FixSync) {
    GlobalStep = -1;
    countBeat = -1;
    countTicks = -1;
  }

  if (numNotesHeld > 0) AllNotesOff();

  arpcount = 0;
  countStep = -1;
  flipflopEnable = false;
  ArpDirection = true;

  if (playing && modeselect == 3) {
    if ((!recording) || (recording && chainrec)) {
      if (!lockpattern) {
        pattern = -1;
      }
    }
  }

  if (menunumber == 2 && menuitem == 3) {  // refresh BPM page
    PrintSubmenu(3);
  }
}

void HandleCC(uint8_t channel, uint8_t cc, uint8_t value) {  // handle midi CC messages

  if (channel == midiChannel) {
    if (cc != 64) {                             // not sustain pedal cc
      if (menunumber == 2 && menuitem == 15) {  // if cc mapping mode
        EnableButtons = true;

        if (value > 0) {
          if (mapButtonSelect == 1) redbuttonCC = cc;
          else if (mapButtonSelect == 2) yellowbuttonCC = cc;
          else if (mapButtonSelect == 3) bluebuttonCC = cc;
          else if (mapButtonSelect == 4) greenbuttonCC = cc;

          PrintSubmenu(menuitem);
          StartScreenTimer = true;
          Bip(2);
        } else {
          mapButtonSelect++;
          PrintSubmenu(menuitem);
          if (mapButtonSelect > 4) {
            mapButtonSelect = 1;
            ScreenBlink(1);
            EEPROM.update(4, redbuttonCC);
            EEPROM.update(5, yellowbuttonCC);
            EEPROM.update(6, bluebuttonCC);
            EEPROM.update(7, greenbuttonCC);
            PrintMenu(menuitem);
          }
        }
      }

      else {  // external control
        bool *state = nullptr;

        if (cc == redbuttonCC) state = &redstate;
        else if (cc == yellowbuttonCC) state = &yellowstate;
        else if (cc == bluebuttonCC) state = &bluestate;
        else if (cc == greenbuttonCC) state = &greenstate;

        if (state) {
          *state = value;
          numbuttonspressedCC += (*state ? 1 : -1);
          ButtonsCommands(*state);
        }

        EnableButtons = (numbuttonspressedCC == 0);
        if (numbuttonspressedCC < 0) numbuttonspressedCC = 0;
      }
    }

    else {  // sustain pedal cc
      sustain = value;
      if (!playing || (playing && trigMode == 0)) MIDI.sendControlChange(cc, value, channel);  // pass through sustain pedal cc
    }
  } else MIDI.sendControlChange(cc, value, channel);
}

void HandleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {  // handle a noteOn event

  if (channel == midiChannel) {

    if (!playing /*&& !recording*/) {  // bypass if not playing
      note = TransposeAndScale(note);
      MIDI.sendNoteOn(note, velocity, channel);
    }

    if (modeselect == 1) {  // arp mode
      if (playing) {

        if (numNotesHeld <= 0) {  // clear the activeNotes array
          numActiveNotes = 0;
          for (uint8_t i = 0; i < holdNotes; i++) {
            activeNotes[i] = -1;
          }
        }

        if (numActiveNotes < holdNotes) {  // store the note only if the activeNotes array is not full
          activeNotes[numActiveNotes] = note;
          numActiveNotes++;
          SortArray();  // sort the activeNotes array
        }
      }
    }

    else if (modeselect != 1) {  // not arp mode
      if (recording) {
        if (playing) {
          if ((countTicks + 2) > (ticksPerStep / 2)) {
            noteSeq[currentSeq][(countStep + 1) % seqLength] = note;  // recording & playing
          } else {
            noteSeq[currentSeq][countStep] = note;
            QueueNote(note);
          }
        } else {  // recording & !playing
          HandleStep();
          noteSeq[currentSeq][countStep] = note;
          QueueNote(note);
        }
      } else if (!recording && playing) rolandTransposeNote = note;  // in recmode transpose to note only if playing
    }

    if (modeselect < 3) {  // not song & live mode
      if (trigMode > 0) {
        muted = false;
        if ((trigMode == 2) && (numNotesHeld == 1)) {  // trigmode 2 retrig start sequence
          Startposition();
          if (sendrealtime) {
            if (internalClock) {
              MIDI.sendSongPosition(0);
            }
          }
        }
      }
    }

    numNotesHeld++;

  } else MIDI.sendNoteOn(note, velocity, channel);
}

void HandleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {  // handle a noteOff event

  if (channel == midiChannel) {

    if (!playing /* && !recording*/) {  // pass trough note off messages
      note = TransposeAndScale(note);
      MIDI.sendNoteOff(note, velocity, channel);
    }

    numNotesHeld--;

    if (numNotesHeld < 0) numNotesHeld = 0;

    if (modeselect == 1 && trigMode > 0 && !sustain) {  // arp mode
      if (numNotesHeld == 0) muted = true;
      for (uint8_t i = 0; i < holdNotes; i++) {
        if (activeNotes[i] == note) {
          activeNotes[i] = -1;
          numActiveNotes--;
          SortArray();
        }
      }
    }

  } else MIDI.sendNoteOff(note, velocity, channel);
}

void SortArray() {  // sort activeNotes array

  if (sortnotes) {  // sort from small to large
    for (uint8_t i = 0; i < holdNotes - 1; i++) {
      for (uint8_t j = 0; j < holdNotes - 1; j++) {
        if (activeNotes[j + 1] < activeNotes[j] && activeNotes[j + 1] >= 0) {
          int8_t temp = activeNotes[j + 1];
          activeNotes[j + 1] = activeNotes[j];
          activeNotes[j] = temp;
        }
      }
    }
  }

  if (trigMode == 0) {  // remove duplicates and replace them with -1
    for (uint8_t i = 0; i < holdNotes - 1; i++) {
      if ((activeNotes[i] > 0) && (activeNotes[i] == activeNotes[i + 1])) {
        activeNotes[i + 1] = -1;
        numActiveNotes--;
      }
    }
  }

  for (uint8_t i = 0; i < holdNotes - 1; i++) {  // shift all the -1s to the right
    if (activeNotes[i] == -1) {
      int8_t temp = activeNotes[i + 1];
      activeNotes[i + 1] = activeNotes[i];
      activeNotes[i] = temp;
    }
  }
}

uint8_t SetScale(int8_t note, uint8_t scale) {  // filter incoming note to fit in to a music scale

  static const PROGMEM int8_t scaleOffsets[][12] = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },             // Scale 0: Linear
    { 0, -1, -2, -1, -2, -1, -2, -3, -1, -2, -1, -2 },  // Scale 1: Pentatonic Major
    { 0, -1, -2, 0, -1, 0, -1, 0, -1, -2, 0, -1 },      // Scale 2: Pentatonic Minor
    { 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0 },        // Scale 3: Major
    { 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, 0, -1 },        // Scale 4: Minor
    { 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, +1, 0 },        // Scale 5: Arabic
    { 0, -1, -2, 0, -1, 0, 0, 0, -1, -2, 0, -1 },       // Scale 6: Blues
    { 0, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1 },        // Scale 7: Locrian
    { 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0 },        // Scale 8: Lydian
    { 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, 0, -1 },        // Scale 9: Dorian
    { 11, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -11 },     // Scale 10: Inverted
    { 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1 }        // Scale 11: Hexatonal
  };

  uint8_t reminder = note % 12;

  int8_t offset = pgm_read_byte(&scaleOffsets[scale][reminder]);  // read from PROGMEM

  return note + offset;
}

uint8_t TransposeAndScale(short int note) {  // apply transposition and scale

  if ((!pitchmode) || (pitchmode && scale == 0)) note = SetScale(note + pitch, scale);
  else note = SetScale(note - pitch, scale) + pitch;

  note = note + posttranspose;  // apply post-transposition

  if (note > 127) {  // if note gets transposed out of range, raise or lower an octave
    note = note - 12;
  } else if (note < 0) {
    note = note + 12;
  }

  return note;
}

void QueueNote(int8_t note) {  // send notes to the midi port

  const uint8_t noteMaxLength = 100;  // this number gets put in the cronLength buffer when the sequencer generates a note
  bool queued = 0;                    // has current note been queued?
  int8_t shortest = noteMaxLength;
  int8_t shortIdx = 0;

  if (modeselect == 1) {  // only in arp mode
    if (arpcount > 0) note = note + ((arpcount - 1) * stepdistance);
  }

  note = TransposeAndScale(note);

  if (modeselect == 1 && recording) {  // inter-recording
    IntercountStep++;
    noteSeq[currentSeq][IntercountStep] = note;
    if (IntercountStep == seqLength) {
      IntercountStep = 0;
      recording = false;
      ManageRecording();
    }
  }

  for (uint8_t i = 0; (i < queueLength) && (!queued); i++) {  // check avail queue
    if ((cronLength[i] < 0) || (cronNote[i] == note)) {       // free queue slot
      MIDI.sendNoteOn(note, 64, midiChannel);
      if (cronNote[i] >= 0) {  // contains a note that hasn't been collected yet
        MIDI.sendNoteOff(cronNote[i], 0, midiChannel);
      }
      cronNote[i] = note;
      cronLength[i] = noteMaxLength;
      queued = true;
    } else {
      if (cronLength[i] < shortest) {
        shortest = cronLength[i];
        shortIdx = i;
      }
    }
  }

  if (!queued) {  // no free queue slots, steal the next expiry
    MIDI.sendNoteOff(cronNote[shortIdx], 0, midiChannel);
    MIDI.sendNoteOn(note, 64, midiChannel);
    cronNote[shortIdx] = note;
    cronLength[shortIdx] = noteMaxLength;
    queued = true;
  }
}

void ManageRecording() {  // recording setup

  if (!recording) {
    rolandLowNote = 128;  // get the low note for transpose
    for (uint8_t i = 0; i < maxSeqLength; i++) {
      if ((noteSeq[currentSeq][i] < rolandLowNote) && (noteSeq[currentSeq][i] >= 0)) {
        rolandLowNote = noteSeq[currentSeq][i];
      }
    }
  }

  digitalWrite(redled, recording);

  rolandTransposeNote = rolandLowNote;

  if (internalClock && !playing) {
    Startposition();
  }
}

void SetArpStyle(uint8_t style) {  // 1=up, 2=down, 3=up-down, 4=down-up, 5=up+down, 6=down+up, 7=random

  if (arpstyle == 2 || arpstyle == 4 || arpstyle == 6) {
    if (countStep == -1) countStep = numActiveNotes;
  }

  uint8_t arpup = (countStep + 1) % numActiveNotes;
  uint8_t arpdown = (numActiveNotes + (countStep - 1)) % numActiveNotes;
  uint8_t arpcountplus = 1 + (arpcount % arprepeat);

  switch (style) {
    case 1:  // up
      countStep = arpup;
      if (countStep == 0) arpcount = arpcountplus;
      break;

    case 2:  // down
      countStep = arpdown;
      if (countStep == (numActiveNotes - 1)) arpcount = arpcountplus;
      break;

    case 3:  // up-down
      if (ArpDirection) {
        countStep = arpup;
        if (countStep == 0) arpcount = arpcountplus;
        if ((countStep + 1) == numActiveNotes) ArpDirection = false;
      } else {
        countStep = arpdown;
        if (countStep == 0) arpcount = arpcountplus;
        if (countStep == 0) ArpDirection = true;
      }
      break;

    case 4:  // down-up
      if (ArpDirection) {
        countStep = arpdown;
        if (countStep == (numActiveNotes - 1)) arpcount = arpcountplus;
        if (countStep == 0) ArpDirection = false;
      } else {
        countStep = arpup;
        if (countStep == (numActiveNotes - 1)) arpcount = arpcountplus;
        if ((countStep + 1) == numActiveNotes) ArpDirection = true;
      }
      break;

    case 5:  // up+down
      if (ArpDirection) {
        countStep++;
        if (countStep > (numActiveNotes - 1)) {
          countStep = numActiveNotes - 1;
          ArpDirection = false;
        }
      } else {
        countStep--;
        if (countStep < 0) {
          countStep = 0;
          ArpDirection = true;
          arpcount = arpcountplus;
        }
      }
      break;

    case 6:  // down+up
      if (ArpDirection) {
        countStep--;
        if (countStep < 0) {
          countStep = 0;
          ArpDirection = false;
        }
      } else {
        countStep++;
        if (countStep > (numActiveNotes - 1)) {
          countStep = numActiveNotes - 1;
          ArpDirection = true;
          arpcount = arpcountplus;
        }
      }
      break;

    case 7:  // random
      countStep = random(0, numActiveNotes);
      arpcount = random(1, arprepeat + 1);
      break;
  }

  if (arprepeat < 2) arpcount = 1;
}

void PrintMainScreen() {  // print menu 0 to the screen

  menunumber = 0;

  oled.clear();
  oled.home();
  oled.setScale(2);
  oled.invertText(1);

  if (modeselect == 1) {  // arp mode screen
    oled.setCursorXY(4, 0);
    oled.print(F(" ARP MODE "));
    oled.invertText(0);
    oled.setScale(3);
    oled.setCursorXY(0, 19);

    switch (arpstyle) {
      case 1:
        oled.setCursorXY(48, 19);
        oled.print(printup);
        break;
      case 2:
        oled.setCursorXY(28, 19);
        oled.print(printdown);
        break;
      case 3:
        oled.print(printup);
        oled.print(minus);
        oled.print(printdown);
        break;
      case 4:
        oled.print(printdown);
        oled.print(minus);
        oled.print(printup);
        break;
      case 5:
        oled.print(printup);
        oled.print(plus);
        oled.print(printdown);
        break;
      case 6:
        oled.print(printdown);
        oled.print(plus);
        oled.print(printup);
        break;
      case 7:
        oled.setCursorXY(10, 19);
        oled.print(printrandom);
        break;
    }
    oled.setScale(2);
  }

  else if (modeselect == 2) {  // rec mode screen
    oled.setCursorXY(4, 0);
    oled.print(F(" REC MODE "));
    oled.invertText(0);
    oled.setCursorXY(36, 16);
    oled.print(seq);
    oled.print(currentSeq + 1);

    if (seqLength > 9) oled.setCursorXY(13, 32);
    else oled.setCursorXY(18, 32);
    oled.print(length);
    if (seqLength > 9) oled.setCursorXY(90, 32);
    else oled.setCursorXY(95, 32);
    oled.print(seqLength);
  }

  else if (modeselect == 3) {  // song mode screen
    oled.print(F(" SONG MODE "));
    oled.invertText(0);
    for (uint8_t i = 0; i < patternLength; i++) {
      oled.setCursorXY((i * 16) + 2, 16);
      if (songPattern[i] > 0) oled.print(songPattern[i]);
      else oled.print(printx);
    }
    PrintSongLiveCursor = true;
  }

  else if (modeselect == 4) {  // live mode screen
    oled.print(F(" LIVE MODE "));
    oled.invertText(0);
    for (uint8_t i = 0; i < 4; ++i) {
      uint8_t x = (i < 2) ? 13 : 102;
      uint8_t y = (i % 2 == 0) ? 16 : 48;
      oled.setCursorXY(x, y);
      oled.print(i + 1);
    }
    PrintSongLiveCursor = true;
  }

  PrintTimeBar = true;
}

void PrintBPMBar() {  // print BPM status bar to the screen

  oled.setScale(2);

  if (modeselect != 4) oled.clear(0, 54, 127, 64);
  else oled.clear(40, 16, 86, 31);

  if ((internalClock && BPM > 99) || (!internalClock)) {
    if (modeselect != 4) oled.setCursorXY(4, 48);
    else oled.setCursorXY(46, 16);
  }

  else if (internalClock && BPM < 100) {
    if (modeselect != 4) oled.setCursorXY(11, 48);
    else oled.setCursorXY(52, 16);
  }

  if (internalClock) oled.print(BPM);
  else oled.print(ext);

  if (modeselect != 4) {
    oled.setScale(1);
    oled.print(space);
    oled.setScale(2);
  } else oled.setCursorXY(46, 32);

  oled.print(printbpm);

  if (modeselect != 4) {
    oled.setScale(1);
    oled.print(space);
    oled.setScale(2);
  } else oled.setCursorXY(45, 48);

  oled.print(tSignature);
  oled.print(fourth);
}

void PrintPatternSequenceCursor() {  // print the cursor to the screen - song mode

  oled.setScale(2);

  if (modeselect == 3) {

    oled.clear(0, 34, 127, 40);
    oled.setCursorXY((pattern * 16) + 2, 32);

    if (!lockpattern) oled.print(upcursor);
    else oled.print(F("="));
  }

  else if (modeselect == 4) {

    oled.clear(11, 32, 25, 40);
    oled.clear(100, 32, 114, 40);

    if (currentSeq < 2) oled.setCursorXY(13, 32);
    else oled.setCursorXY(102, 32);

    if (currentSeq == 0 || currentSeq == 2) oled.print(upcursor);
    else oled.print(downcursor);
  }
}

void PrintMenu(uint8_t item) {  // print menu 1 to the screen

  menunumber = 1;

  oled.clear();
  oled.home();
  oled.setScale(3);
  oled.print(printnext);

  switch (item) {

    case 0:  // load-save
      oled.print(file);
      break;

    case 1:  // mode select
      oled.println(printmode);
      oled.print(select);
      break;

    case 2:  // seq.select
      if (modeselect == 1) {
        oled.println(arp);
        oled.print(printstyle);
      }

      else if (modeselect == 2) {
        oled.println(seq);
        oled.print(select);
      }

      else if (modeselect == 3) {
        oled.println(edit);
        oled.print(song);
      }

      else if (modeselect == 4) {
        if (playing) oled.println(F("STOP"));
        else oled.println(F("START"));
        oled.print(printlive);
      }
      break;

    case 3:  // tempo
      oled.print(F("TEMPO"));
      break;

    case 4:  // pitch
      oled.println(printpitch);
      break;

    case 5:  // scale
      oled.println(printscale);
      break;

    case 6:  // note lenght
      oled.println(printnote);
      oled.print(length);
      break;

    case 7:  // seq.length / arp steps
      if (modeselect != 1) {
        oled.println(seq);
        oled.print(length);
      } else {
        oled.println(arp);
        oled.print(steps);
      }
      break;

    case 8:  // arp/seq. speed
      if (modeselect != 1) oled.println(seq);
      else oled.println(arp);
      oled.print(speed);
      break;

    case 9:  // trig.mode / chain rec / snap mode
      if (modeselect < 3) {
        oled.println(F("TRIG"));
        oled.print(printmode);
      } else if (modeselect == 3) {
        oled.println(chain);
        oled.print(rec);
      } else if (modeselect == 4) {
        oled.println(F("SNAP"));
        oled.print(printmode);
      }
      break;

    case 10:  // swing
      oled.print(printswing);
      break;

    case 11:  // metro
      oled.print(printmetro);
      break;

    case 12:  // midi channel
      oled.println(F("MIDI"));
      oled.print(F("CHANNEL"));
      break;

    case 13:  // midi realtime messages
      oled.println(F("SEND"));
      oled.print(sync);
      break;

    case 14:  // sync port
      oled.println(ext);
      oled.print(sync);
      break;

    case 15:  // map buttons
      oled.println(printmap);
      oled.print(F("KEYS"));
      break;

    case 16:  // sound
      oled.print(printsound);
      break;

    case 17:  // restart
      oled.print(F("REBOOT"));
      break;
  }
}

void PrintSubmenu(uint8_t item) {  // print menu 2 to the screen

  menunumber = 2;

  oled.clear();
  oled.home();
  oled.setScale(3);

  if (item == 0) oled.print(printnext);
  else if (item < 17) oled.print(printback);

  switch (item) {

    case 0:  // load-save
      if (modeselect == 1 && savemode == 0) savemode = 1;
      if (savemode == 0) oled.println(printbake);
      else if (savemode == 1) oled.println(printnew);
      else if (savemode == 2) oled.println(printsave);
      else if (savemode == 3) oled.println(printload);
      else if (savemode == 4) oled.println(printdelete);
      if (savemode == 0 && ((modeselect == 2) || (modeselect == 3 && lockpattern))) oled.print(seq);
      else oled.print(song);
      break;

    case 1:  // mode select
      oled.println(printmode);
      if (premodeselect == 1) oled.print(arp);
      else if (premodeselect == 2) oled.print(rec);
      else if (premodeselect == 3) oled.print(song);
      else if (premodeselect == 4) oled.print(printlive);
      break;

    case 2:  // arp style/seq.select/edit song/start-stop live
      if (modeselect == 1) {
        oled.println(printstyle);

        switch (arpstyle) {
          case 1:
            oled.print(printup);
            break;
          case 2:
            oled.print(printdown);
            break;
          case 3:
            oled.print(printup);
            oled.print(minus);
            oled.print(printdown);
            break;
          case 4:
            oled.print(printdown);
            oled.print(minus);
            oled.print(printup);
            break;
          case 5:
            oled.print(printup);
            oled.print(plus);
            oled.print(printdown);
            break;
          case 6:
            oled.print(printdown);
            oled.print(plus);
            oled.print(printup);
            break;
          case 7:
            oled.print(printrandom);
            break;
        }

        oled.setScale(2);
        oled.setCursorXY(0, 48);
        if (sortnotes) {
          oled.print(F("ORDERED"));
        } else oled.print(F("KEY-ORDER"));

      }

      else if (modeselect == 2) {
        oled.println(seq);
        oled.print(currentSeq + 1);
      }

      else if (modeselect == 3) {
        oled.println(edit);
        oled.setScale(2);
        for (uint8_t i = 0; i < patternLength; i++) {
          oled.setCursorXY(i * 16, 30);
          if (songPattern[i] > 0) oled.print(songPattern[i]);
          else oled.print(printx);
        }
        oled.setCursorXY((curpos * 16), 48);
        oled.print(upcursor);
      }

      else if (modeselect == 4) {
        if (!playing) muted = true;
        StartAndStop();
        confirmsound = true;
        greentristate = 1;
        PrintMainScreen();
      }
      break;

    case 3:  // tempo
      oled.println(printbpm);
      oled.print(BPM);
      oled.setScale(1);
      oled.print(space);
      oled.setScale(3);
      if (!internalClock) oled.print(F("INT"));
      break;

    case 4:  // pitch
      oled.println(printpitch);
      if (pitch > 0) oled.print(plus);
      oled.print(pitch);

      if (pitchmode) {
        oled.print(space);

        uint8_t normalizedTranspose = (pitch % 12 + 12) % 12;  // normalize the transpose value

        switch (normalizedTranspose) {
          case 0:
            oled.print(F("C"));
            break;
          case 1:
            oled.print(F("C#"));
            break;
          case 2:
            oled.print(F("D"));
            break;
          case 3:
            oled.print(F("D#"));
            break;
          case 4:
            oled.print(F("E"));
            break;
          case 5:
            oled.print(F("F"));
            break;
          case 6:
            oled.print(F("F#"));
            break;
          case 7:
            oled.print(F("G"));
            break;
          case 8:
            oled.print(F("G#"));
            break;
          case 9:
            oled.print(F("A"));
            break;
          case 10:
            oled.print(F("A#"));
            break;
          case 11:
            oled.print(F("B"));
            break;
        }
      }
      oled.setScale(2);
      oled.setCursorXY(0, 48);
      if (!pitchmode) {
        oled.print(F("PRE-"));
        oled.print(printscale);
      } else {
        oled.print(printscale);
        oled.print(F("-ROOT"));
      }
      break;

    case 5:  // scale
      oled.println(printscale);
      switch (scale) {
        case 0:
          oled.print(F("LINEAR"));
          break;
        case 1:
          oled.print(F("PENTMAJ"));
          break;
        case 2:
          oled.print(F("PENTMIN"));
          break;
        case 3:
          oled.print(F("MAJOR"));
          break;
        case 4:
          oled.print(F("MINOR"));
          break;
        case 5:
          oled.print(F("ARABIC"));
          break;
        case 6:
          oled.print(F("BLUES"));
          break;
        case 7:
          oled.print(F("LOCRIAN"));
          break;
        case 8:
          oled.print(F("LYDIAN"));
          break;
        case 9:
          oled.print(F("DORIAN"));
          break;
        case 10:
          oled.print(F("INVERSE"));
          break;
        case 11:
          oled.print(F("HEXATON"));
          break;
      }
      oled.setScale(2);
      oled.setCursorXY(0, 48);
      oled.print(F("TRANSP."));
      if (posttranspose > 0) oled.print(plus);
      oled.print(posttranspose);
      break;

    case 6:  // note lenght
      oled.println(length);
      if (noteLengthSelect == 0) {
        oled.print(printrandom);
      } else if (noteLengthSelect == 1) {
        oled.print(10);
      } else if (noteLengthSelect == 2) {
        oled.print(30);
      } else if (noteLengthSelect == 3) {
        oled.print(50);
      } else if (noteLengthSelect == 4) {
        oled.print(70);
      } else if (noteLengthSelect == 5) {
        oled.print(100);
      } else if (noteLengthSelect == 6) {
        oled.print(120);
      }
      if (noteLengthSelect > 0) oled.print(F("%"));
      break;

    case 7:  // seq.length / arp steps
      if (modeselect != 1) {
        oled.println(length);
        oled.print(seqLength);
      } else {
        oled.println(steps);
        if (arprepeat > 1) oled.print(plus);
        oled.print(arprepeat - 1);
        oled.setScale(2);
        oled.setCursorXY(0, 48);
        oled.print(F("DIST."));
        if (stepdistance > 0) oled.print(plus);
        oled.print(stepdistance);
      }
      break;

    case 8:  // arp/seq. speed
      oled.println(speed);
      if (StepSpeed == 4) {
        oled.print(half);
      } else if (StepSpeed == 3) {
        oled.print(third);
      } else if (StepSpeed == 2) {
        oled.print(normal);
      } else if (StepSpeed == 1) {
        oled.print(twice);
      }
      break;

    case 9:  // trig.mode / chain rec / snap mode
      if (modeselect < 3) {
        oled.println(printmode);
        if (trigMode == 0) oled.print(F("HOLD"));
        else if (trigMode == 1) {
          oled.print(F("GATE"));
          //numActiveNotes = 0;
        } else if (trigMode == 2) oled.print(F("RETRIG"));
      }

      else if (modeselect == 3) {
        oled.println(chain);
        if (chainrec) oled.print(on);
        else oled.print(off);
      }

      else if (modeselect == 4) {
        oled.println(printmode);
        if (snapmode == 0) oled.print(F("PATTERN"));
        if (snapmode == 1) oled.print(F("UP-"));
        if (snapmode > 0) oled.print(F("BEAT"));
      }
      break;

    case 10:  // swing
      oled.println(printswing);
      if (!swing) oled.print(off);
      else if (swing) oled.print(on);
      break;

    case 11:  // metro
      oled.println(printmetro);
      if (!metro) oled.print(off);
      else if (metro) oled.print(on);
      oled.setScale(2);
      oled.setCursorXY(0, 48);
      oled.print(F("SIGN."));
      oled.print(tSignature);
      oled.print(fourth);
      break;

    case 12:  // midi channel
      oled.println(F("CH"));
      oled.print(midiChannel);
      break;

    case 13:  // midi realtime messages
      oled.println(sync);
      if (!sendrealtime) oled.print(off);
      else if (sendrealtime == 1) oled.print(on);
      else if (sendrealtime == 2) oled.print(F("ALWAYS"));
      break;

    case 14:  // sync port
      oled.println(F("PORT"));
      if (syncport == 0) oled.print(off);
      else if (syncport > 0) oled.print(syncport);
      break;

    case 15:  // map buttons
      oled.println(printmap);
      oled.print(printCC);
      AllLedsOff();
      if (mapButtonSelect == 1) {
        oled.print(redbuttonCC);
        digitalWrite(redled, HIGH);
      } else if (mapButtonSelect == 2) {
        oled.print(yellowbuttonCC);
        digitalWrite(yellowled, HIGH);
      } else if (mapButtonSelect == 3) {
        oled.print(bluebuttonCC);
        digitalWrite(blueled, HIGH);
      } else if (mapButtonSelect == 4) {
        oled.print(greenbuttonCC);
        digitalWrite(greenled, HIGH);
      }
      break;

    case 16:  // sound
      oled.println(printsound);
      if (soundmode == 1) oled.print(on);
      else if (soundmode == 2) oled.print(F("UI-"));
      if (soundmode != 1) oled.print(off);
      break;

    case 17:  // restart
      asm volatile(" jmp 0");
      break;
  }
}

void SubmenuSettings(uint8_t item, bool dir) {  // handles changing settings in menu 2

  switch (item) {

    case 0:  // load-save
      if (dir == LOW) {
        if (savemode < 4) savemode++;
      } else {
        if (savemode > 0) savemode--;
      }
      break;

    case 1:  // mode select
      if (dir == LOW) {
        if (premodeselect < 4) premodeselect++;
      } else {
        if (premodeselect > 1) premodeselect--;
      }
      break;

    case 2:  // seq.select
      if (modeselect == 1) {
        if (!greenstate) {
          if (dir == HIGH) {
            if (arpstyle > 1) arpstyle--;
          } else {
            if (arpstyle < 7) arpstyle++;
          }
        } else {
          if (dir == HIGH) {
            sortnotes = true;
          } else {
            sortnotes = false;
          }
        }
      }

      else if (modeselect == 2) {
        if (dir == HIGH) {
          if (currentSeq > 0) currentSeq--;
        } else {
          if (currentSeq < (numberSequences - 1)) currentSeq++;
        }
      }

      else if (modeselect == 3) {
        if (!greenstate) {
          if (dir == HIGH) {
            if (curpos < 7) curpos++;
          } else {
            if (curpos > 0) curpos--;
          }
        } else {
          if (dir == HIGH) {
            if (songPattern[curpos] < 4) songPattern[curpos]++;
          } else {
            if (songPattern[curpos] > 0) {
              if (curpos > 0) songPattern[curpos]--;
              else if (curpos == 0 && (songPattern[curpos] > 1)) songPattern[curpos]--;
            }
          }
        }
      }
      break;

    case 3:  // tempo
      if (dir == HIGH) {
        if (!greenstate) BPM++;
        else BPM += 5;
      } else {
        if (!greenstate) BPM--;
        else BPM -= 5;
      }
      BPM = constrain(BPM, 20, 250);
      SetBPM(BPM);
      break;

    case 4:  // pitch
      if (!greenstate) {
        if (dir == HIGH) {
          if (pitch < 12) pitch++;
        } else {
          if (pitch > -12) pitch--;
        }
      } else {
        if (dir == HIGH) {
          pitchmode = true;
        } else {
          pitchmode = false;
        }
      }
      if (!playing) AllNotesOff();
      break;

    case 5:  // scale
      if (!greenstate) {
        if (dir == LOW) {
          if (scale < 11) scale++;
        } else {
          if (scale > 0) scale--;
        }
      } else {
        if (dir == HIGH) {
          if (posttranspose < 12) posttranspose++;
        } else {
          if (posttranspose > -12) posttranspose--;
        }
      }
      if (!playing) AllNotesOff();
      break;

    case 6:  // note lenght
      if (dir == HIGH) {
        if (noteLengthSelect < 6) noteLengthSelect++;
      } else {
        if (noteLengthSelect > 0) noteLengthSelect--;
      }
      break;

    case 7:  // seq.length / arp steps
      if (modeselect != 1) {
        if (dir == HIGH) {
          if (seqLength < maxSeqLength) seqLength++;
        } else {
          if (seqLength > 1) seqLength--;
        }
      } else {
        if (!greenstate) {
          if (dir == HIGH) {
            if (arprepeat < 5) arprepeat++;
          } else {
            if (arprepeat > 1) arprepeat--;
          }
        } else {
          if (dir == HIGH) {
            if (stepdistance < 12) stepdistance++;
          } else {
            if (stepdistance > -12) stepdistance--;
          }
        }
      }
      break;

    case 8:  // arp/seq. speed
      if (dir == HIGH) {
        if (StepSpeed > 1) StepSpeed--;
      } else {
        if (StepSpeed < 4) StepSpeed++;
      }
      FixSync = true;
      break;

    case 9:  // trig.mode / chain rec
      if (modeselect < 3) {
        if (dir == HIGH) {
          if (trigMode > 0) trigMode--;
        } else {
          if (trigMode < 2) trigMode++;
        }
      }

      else if (modeselect == 3) {
        if (dir == HIGH) chainrec = true;
        else chainrec = false;
      }

      else if (modeselect == 4) {
        if (dir == HIGH) {
          if (snapmode < 2) snapmode++;
        } else {
          if (snapmode > 0) snapmode--;
        }
      }
      break;

    case 10:  // swing
      if (dir == HIGH) swing = true;
      else swing = false;
      break;

    case 11:  // metro
      if (!greenstate) {
        if (dir == HIGH) metro = true;
        else metro = false;
      } else {
        if (dir == HIGH) {
          if (tSignature < 8) tSignature++;
        } else {
          if (tSignature > 1) tSignature--;
        }
      }
      break;

    case 12:  // midi channel
      if (!greenstate) {
        AllNotesOff();
        if (dir == HIGH) {
          if (midiChannel < 16) midiChannel++;
        } else {
          if (midiChannel > 1) midiChannel--;
        }
      } else {
        EEPROM.update(0, midiChannel);
        ScreenBlink();
      }
      break;

    case 13:  // send midi realtime messages
      if (!greenstate) {
        if (dir == LOW) {
          if (sendrealtime < 2) sendrealtime++;
        } else {
          if (sendrealtime > 0) sendrealtime--;
        }
      } else {
        EEPROM.update(1, sendrealtime);
        ScreenBlink();
      }
      break;

    case 14:  // sync port
      if (internalClock) {
        if (!greenstate) {
          if (dir == HIGH) {
            if (syncport < 2) syncport++;
          } else {
            if (syncport > 0) syncport--;
          }
        } else {
          EEPROM.update(3, syncport);
          ScreenBlink();
        }
      }
      SetSyncPort(syncport);
      break;

    case 15:  // map buttons
      if (!greenstate) {
        if (dir == HIGH) {
          if (mapButtonSelect > 1) mapButtonSelect--;
        } else {
          if (mapButtonSelect < 4) mapButtonSelect++;
        }
      } else {
        EEPROM.update(4, redbuttonCC);
        EEPROM.update(5, yellowbuttonCC);
        EEPROM.update(6, bluebuttonCC);
        EEPROM.update(7, greenbuttonCC);
        ScreenBlink();
      }
      break;

    case 16:  // sound
      if (!greenstate) {
        if (dir == HIGH) {
          if (soundmode > 1) soundmode--;
        } else {
          if (soundmode < 3) soundmode++;
        }
      } else {
        EEPROM.update(2, soundmode);
        ScreenBlink();
      }
      SetSound(soundmode);
      break;

    case 17:  // restart
      break;
  }
  PrintSubmenu(item);
}

void PrintLoadSaveMenu(uint8_t mode) {  // print menu 3 to the screen

  menunumber = 3;

  oled.home();
  oled.setScale(2);
  oled.invertText(1);

  if (mode == 0) {  // bake song
    confirmation = true;
    PrintConfirmationPopup();
  }

  else if (mode == 1) {  // new song
    oled.clear(0, 32, 127, 48);
    oled.setCursorXY(5, 0);
    oled.print(F(" NEW SONG "));
    oled.invertText(0);
    oled.setCursorXY(5, 16);
    oled.print(seq);
    oled.print(length);
    oled.setScale(3);
    if (NewSeqLength > 9) oled.setCursorXY(45, 32);
    else oled.setCursorXY(56, 32);
    oled.print(NewSeqLength);
  }

  else if (mode > 1) {  // save, load, delete song
    oled.clear(48, 20, 64, 64);
    oled.clear(112, 20, 127, 127);

    if (mode == 2) {
      oled.print(F(" SAVE SONG "));
    }

    else if (mode == 3) {
      oled.print(F(" LOAD SONG "));
    }

    else if (mode == 4) {
      oled.print(F("  DELETE   "));
    }

    oled.invertText(0);

    for (uint8_t i = 0; i < 6; i++) {
      if (i < 3) oled.setCursorXY(0, (i + 1) * 16);
      else oled.setCursorXY(65, (i - 2) * 16);
      if (EEPROM.read(eepromaddress(16, i)) > 0) {
        oled.print(F("SNG"));
        oled.print(i + 1);
      } else oled.print(F("----"));
    }

    if (savecurX <= 2) {
      oled.setCursorXY(50, (savecurX + 1) * 16);
    }

    if (savecurX >= 3) {
      oled.setCursorXY(115, (savecurX - 2) * 16);
    }

    oled.print(printback);
  }
}

void PrintConfirmationPopup() {  // print confirmation popup in load-save menu

  oled.setScale(2);
  oled.invertText(1);
  oled.setCursorXY(28, 33);

  if (savemode < 2) {  // bake & new
    oled.setCursorXY(8, 33);
    oled.print(F(" PROCEED?"));
  }

  else if (savemode == 2) {  // save
    if (!full) oled.print(F(" SAVE?"));
    else {
      oled.setCursorXY(4, 33);
      oled.print(F(" OVERRIDE?"));
    }
  }

  else if (savemode > 2) {  // load
    if (!full) {
      oled.setCursorXY(22, 33);
      oled.print(F(" EMPTY!"));
    } else {
      if (savemode == 3) oled.print(F(" LOAD?"));
      else if (savemode == 4) {  // delete
        oled.setCursorXY(16, 33);
        oled.print(F(" DELETE?"));
      }
    }
  }

  oled.invertText(0);
}

void PrintInterRecordingPopup() {  // seq. select popup

  oled.setScale(2);
  oled.invertText(1);
  oled.setCursorXY(28, 33);
  oled.print(space);
  oled.print(seq);
  oled.print(newcurrentSeq + 1);
  oled.invertText(0);
}

void ClearSeqPatternArray() {  // clear all Pattern and Sequences arrays

  for (uint8_t i = 0; i < patternLength; i++) {  // clean SongPattern array
    if (i < 4) songPattern[i] = (i + 1);
    else songPattern[i] = (i - 3);
  }

  for (uint8_t j = 0; j < numberSequences; j++) {  // clean Seq arrays
    for (uint8_t i = 0; i < maxSeqLength; i++) {
      noteSeq[j][i] = -1;
    }
  }
}

void BakeSequence() {  // bake transpose & scale in to current seq / all seqs

  if ((modeselect == 2) || (modeselect == 3 && lockpattern)) {  // bake current Seq array
    for (uint8_t i = 0; i < maxSeqLength; i++) {
      if (noteSeq[currentSeq][i] > 0) noteSeq[currentSeq][i] = TransposeAndScale(noteSeq[currentSeq][i]);
    }
  }

  else {
    for (uint8_t j = 0; j < numberSequences; j++) {  // bake all Seq arrays
      for (uint8_t i = 0; i < maxSeqLength; i++) {
        if (noteSeq[j][i] > 0) noteSeq[j][i] = TransposeAndScale(noteSeq[j][i]);
      }
    }
  }

  scale = 0;
  pitch = 0;
  posttranspose = 0;
}

void LoadSave(uint8_t mode, uint8_t number) {  // (bake/new/save/load/delete, slot number 0-5)

  if (mode == 0) {  // bake song
    BakeSequence();
    ScreenBlink();
    PrintMainScreen();
  }

  else if (mode == 1) {  // new song
    playing = false;
    StepSpeed = 2;
    swing = false;
    noteLengthSelect = 4;
    seqLength = NewSeqLength;
    BPM = 120;
    SetBPM(BPM);
    lockpattern = false;
    pattern = 0;
    NewSeqLength = 16;

    ClearSeqPatternArray();

    modeselect = 3;
    currentSeq = 0;
    ScreenBlink();
    PrintMainScreen();
  }

  else if (mode > 1) {

    if (mode == 2) {  // save song
      EEPROM.update((eepromaddress(16, number)), 1);
      EEPROM.update((eepromaddress(17, number)), pitchmode);
      EEPROM.update((eepromaddress(18, number)), pitch);
      EEPROM.update((eepromaddress(19, number)), scale);
      EEPROM.update((eepromaddress(20, number)), posttranspose);
      EEPROM.update((eepromaddress(21, number)), seqLength);
      EEPROM.update((eepromaddress(22, number)), BPM);
      EEPROM.update((eepromaddress(23, number)), swing);

      for (uint8_t i = 24; i <= 159; i++) {
        if (i <= 31) EEPROM.update((eepromaddress(i, number)), songPattern[i - 24]);        // save SongPattern array
        else if (i <= 63) EEPROM.update((eepromaddress(i, number)), noteSeq[0][i - 32]);    // save Seq.1 array
        else if (i <= 95) EEPROM.update((eepromaddress(i, number)), noteSeq[1][i - 64]);    // save Seq.2 array
        else if (i <= 127) EEPROM.update((eepromaddress(i, number)), noteSeq[2][i - 96]);   // save Seq.3 array
        else if (i <= 159) EEPROM.update((eepromaddress(i, number)), noteSeq[3][i - 128]);  // save Seq.4 array
      }

      ScreenBlink();
      oled.clear();
    }

    else if (mode == 3) {  // load song
      if (full) {
        playing = false;
        lockpattern = false;
        pattern = 0;

        pitchmode = EEPROM.read(eepromaddress(17, number));
        pitch = EEPROM.read(eepromaddress(18, number));
        scale = EEPROM.read(eepromaddress(19, number));
        posttranspose = EEPROM.read(eepromaddress(20, number));
        seqLength = EEPROM.read(eepromaddress(21, number));
        BPM = EEPROM.read(eepromaddress(22, number));
        swing = EEPROM.read(eepromaddress(23, number));

        for (uint8_t i = 24; i <= 159; i++) {
          if (i <= 31) songPattern[i - 24] = EEPROM.read(eepromaddress(i, number));        // load SongPattern array
          else if (i <= 63) noteSeq[0][i - 32] = EEPROM.read(eepromaddress(i, number));    // load Seq.1 array
          else if (i <= 95) noteSeq[1][i - 64] = EEPROM.read(eepromaddress(i, number));    // load Seq.2 array
          else if (i <= 127) noteSeq[2][i - 96] = EEPROM.read(eepromaddress(i, number));   // load Seq.3 array
          else if (i <= 159) noteSeq[3][i - 128] = EEPROM.read(eepromaddress(i, number));  // load Seq.4 array
        }

        SetBPM(BPM);
        if (modeselect != 4) modeselect = 3;
        currentSeq = 0;
        ManageRecording();
        ScreenBlink();
      } else confirmation = false;
    }

    else if (mode == 4) {  // delete song
      if (full) {
        EEPROM.update((eepromaddress(16, number)), 0);
        ScreenBlink();
        oled.clear();
      } else confirmation = false;
    }

    PrintLoadSaveMenu(savemode);
  }
}

void DebounceButtons() {

  static bool lastGreenDeb = false;
  static bool lastYellowDeb = false;
  static bool lastRedDeb = false;
  static bool lastBlueDeb = false;
  static unsigned long lastDebounceTime = 0;
  static const uint8_t debounceDelay = 10;

  bool greenReading = !digitalRead(greenbutton);
  bool yellowReading = !digitalRead(yellowbutton);
  bool redReading = !digitalRead(redbutton);
  bool blueReading = !digitalRead(bluebutton);

  bool stateChanged = (greenReading != lastGreenDeb) || (yellowReading != lastYellowDeb) || 
                      (redReading != lastRedDeb) || (blueReading != lastBlueDeb);

  if (stateChanged) {
    lastDebounceTime = millis();
    if (!EnableButtons) {
      EnableButtons = true;
      numbuttonspressedCC = 0;
    }
  }

  if ((millis() - lastDebounceTime) > debounceDelay && EnableButtons) {
    if (greenReading != greenstate) {
      greenstate = greenReading;
      ButtonsCommands(greenstate);
    } else if (yellowReading != yellowstate) {
      yellowstate = yellowReading;
      ButtonsCommands(yellowstate);
    } else if (redReading != redstate) {
      redstate = redReading;
      ButtonsCommands(redstate);
    } else if (blueReading != bluestate) {
      bluestate = blueReading;
      ButtonsCommands(bluestate);
    }
  }

  lastGreenDeb = greenReading;
  lastYellowDeb = yellowReading;
  lastRedDeb = redReading;
  lastBlueDeb = blueReading;
}

void ButtonsCommands(bool anystate) {  // manage the buttons's commands and funcions

  static bool lockmute = false;  // keep the mute muted. only in arp, seq & song mode

  static bool redispressed = false;     // is redbutton still pressed?
  static bool yellowispressed = false;  // is yellowbutton still pressed?
  static bool greenispressed = false;   // is greenbutton still pressed?
  static bool blueispressed = false;    // is bluebutton still pressed?

  bool newredstate = false;     // reset redstate snapshot
  bool newyellowstate = false;  // reset yellowstate snapshot
  bool newgreenstate = false;   // reset bluestate snapshot
  bool newbluestate = false;    // reset greenstate snapshot

  if (!redispressed) newredstate = redstate;           // take a redstate snapshot
  if (!yellowispressed) newyellowstate = yellowstate;  // take a yellowstate snapshot
  if (!blueispressed) newbluestate = bluestate;        // take a bluestate snapshot
  if (!greenispressed) newgreenstate = greenstate;     // take a greenstate snapshot

  StartScreenTimer = true;  // if any button is pressed or released, start the screen timer

  if (!greentristate) greentristate = 1;  // set greentristate to null

  if (menunumber == 0) {  // main screen

    if (newgreenstate) {  // go to menu
      greentristate = 2;
      StartMenuTimer = true;
    }

    if (greentristate == 2 && !newgreenstate) {
      greentristate = 0;
    }

    if (modeselect != 4) {

      if (yellowispressed && newgreenstate) {  // restart sequence
        if (!playing) playing = true;
        Startposition();
      }

      else if (blueispressed) {
        if (newgreenstate) SynthReset();       // reset synth
        else if (newredstate && !recording) {  // lock mute
          lockmute = !lockmute;
        }
      }

      else if (greenispressed) {
        if (newredstate && modeselect == 3) {  // lock pattern
          lockpattern = !lockpattern;
          PrintSongLiveCursor = true;
        }
      }

      else if (!greenispressed) {
        if (newbluestate) {  // recording and not playing bluebutton add a space
          if (!playing && internalClock && recording) {
            HandleStep();
            noteSeq[currentSeq][countStep] = -1;
          }
        }

        else if (newyellowstate) {  // play/stop
          StartAndStop();
        }

        if (!blueispressed && newredstate) {  // start/stop recording

          if ((modeselect == 3 && !playing) || modeselect == 1) {
            if (!recording) {
              menunumber = 4;
              PrintInterRecordingPopup();
            } else {
              recording = false;
              ManageRecording();
            }
          } else {
            recording = !recording;
            ManageRecording();
          }
        }
      }

      if (!lockmute) muted = newbluestate;  // toggle mute
      else {
        muted = !newbluestate;
        digitalWrite(blueled, !newbluestate);
      }
    }

    else if (modeselect == 4) {

      if (newredstate) newcurrentSeq = 0;
      if (newyellowstate) newcurrentSeq = 1;
      if (newbluestate) newcurrentSeq = 2;
      if (!greentristate) newcurrentSeq = 3;

      if (newredstate || newyellowstate || newbluestate || !greentristate) {
        if (!playing) {

          currentSeq = newcurrentSeq;
          PrintSongLiveCursor = true;
          StartAndStop();
          muted = false;
        } else {
          TrigMute = true;
          if (muted) {
            currentSeq = newcurrentSeq;
            PrintSongLiveCursor = true;
          }
        }
      }
    }

  }

  else if (menunumber == 1) {  // menu

    if (newgreenstate) {  // go to submenu
      PrintSubmenu(menuitem);
    }

    else if (newbluestate) {  // go back to main screen
      PrintMainScreen();
      if (lockmute) digitalWrite(blueled, HIGH);
    }

    else if (newyellowstate) {  // go up in the menu
      if (menuitem < 17) {
        menuitem++;
        PrintMenu(menuitem);
      }
    }

    else if (newredstate) {  // go down in the menu
      if (menuitem > 0) {
        menuitem--;
        PrintMenu(menuitem);
      }
    }
  }

  else if (menunumber == 2) {  // submenu

    if (newbluestate) PrintMenu(menuitem);  // go back to the menu

    else if (newgreenstate) {
      if (menuitem == 0) {
        savecurX = 0;
        if (savemode > 0) oled.clear();
        PrintLoadSaveMenu(savemode);
        newgreenstate = false;
      }

      if (menuitem == 1) {
        newcurrentSeq = currentSeq;
        modeselect = premodeselect;
        ScreenBlink();
        PrintMainScreen();
        //newgreenstate = false;
      }
    }

    else if (newyellowstate) SubmenuSettings(menuitem, LOW);
    else if (newredstate) SubmenuSettings(menuitem, HIGH);
  }

  else if (menunumber == 3) {  // load/save menu

    if (!confirmation) {

      if (savemode == 0) {  // bake song
        // nothing to do
      }

      else if (savemode == 1) {  // new song
        if (newredstate || newyellowstate) {

          if (newyellowstate) {
            if (NewSeqLength > 1) {
              NewSeqLength--;
            }
          }

          else if (newredstate) {
            if (NewSeqLength < 32) {
              NewSeqLength++;
            }
          }

          PrintLoadSaveMenu(savemode);
        }
      }

      else if (savemode > 1) {
        if (newredstate || newyellowstate) {

          if (newredstate) {
            if (savecurX > 0) {
              savecurX--;
            }
          }

          else if (newyellowstate) {
            if (savecurX < 5) {
              savecurX++;
            }
          }

          PrintLoadSaveMenu(savemode);
        }
        full = EEPROM.read(eepromaddress(16, savecurX));
      }

      if (newgreenstate) {
        confirmation = true;
        PrintConfirmationPopup();
        newgreenstate = false;
      }
    }

    if (newgreenstate && confirmation) {
      LoadSave(savemode, savecurX);
      confirmation = false;
    }

    else if (newbluestate) {
      if (!confirmation) PrintSubmenu(menuitem);
      else {
        confirmation = false;
        if (savemode > 0) PrintLoadSaveMenu(savemode);
        else PrintSubmenu(menuitem);
      }
    }
  }

  else if (menunumber == 4) {  // Inter-Recording Popup

    if (newbluestate) {
      menunumber = 0;
      PrintMainScreen();
    }

    else if (newgreenstate) {
      menunumber = 0;
      currentSeq = newcurrentSeq;
      pattern = currentSeq;
      PrintMainScreen();
      IntercountStep = -1;
      recording = true;
      ManageRecording();
    }

    else if (newyellowstate) {
      if (newcurrentSeq > 0) newcurrentSeq--;
      PrintInterRecordingPopup();
    }

    else if (newredstate) {
      if (newcurrentSeq < (numberSequences - 1)) newcurrentSeq++;
      PrintInterRecordingPopup();
    }
  }

  if (!(menunumber == 2 && menuitem == 15)) {  // if not in CC mapping menu, turn the led on if button is pressed

    if (!recording) {
      digitalWrite(redled, redstate);
      digitalWrite(yellowled, yellowstate);
    }

    digitalWrite(greenled, greenstate);

    if (!lockmute || menunumber > 0) digitalWrite(blueled, newbluestate);
  }

  if (anystate) {  // bips
    if (confirmsound) {
      confirmsound = false;
      Bip(2);
    } else Bip(1);  // do the buttons click if any button is pressed
  }

  greenispressed = greenstate;
  blueispressed = bluestate;
  redispressed = redstate;
  yellowispressed = yellowstate;
}
