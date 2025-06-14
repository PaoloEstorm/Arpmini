/*!
 *  @file       Arpmini_plus.ino
 *  Project     Estorm - Arpmini+
 *  @brief      MIDI Sequencer & Arpeggiator
 *  @version    2.21
 *  @author     Paolo Estorm
 *  @date       06/14/25
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

// system
const char version[] PROGMEM = "2.21";
#include "Vocabulary.h"
#include "Random8.h"
Random8 Random;
#define bitToggle8(value, bit) ((value) ^= (1 << (bit)))  // flip bit (8 bit version)
#define bitSet8(value, bit) ((value) |= (1 << (bit)))     // set bit high (8 bit version)

// LEDs
#define yellowLED 2  // yellow LED pin
#define redLED 3     // red LED pin
#define greenLED 4   // green LED pin
#define blueLED 5    // blue LED pin

const uint8_t LEDs[4] = {
  redLED, yellowLED, blueLED, greenLED
};

// sound
bool uisound = true;        // are ui sounds enabled?
uint8_t soundmode = 1;      // 1 audio on, 2 ui sounds off, 3 off
bool sound = true;          // speaker sound toggle
bool metro = false;         // metronome toggle
bool confirmsound = false;  // at button press, play the confirmation sound instead of the click
#include "Tone8.h"
Tone8 tone8;

// screen
#include "SPI_OLED.h"
SPI_OLED oled;                 // screen initialization
bool StartScreenTimer = true;  // activate the screen-on timer

// memory
#define memorySlots 60
#include "I2C_EEPROM.h"
I2C_EEPROM EEPROM;

// midi
#include <MIDI.h>
#include "SoftwareSerial.h"
SoftwareSerial Serial2(10);                            // midi2 pin (input)
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);   // initialize midi1
MIDI_CREATE_INSTANCE(SoftwareSerial, Serial2, MIDI2);  // initialize midi2
uint8_t midiChannel = 1;                               // MIDI channel to use for sequencer 1 to 16
int8_t syncport = 1;                                   // activate midi-in sync 0=none, 1=port1, 2=port2

// buttons
#define yellowbutton 6  // yellow button pin
#define redbutton 7     // red button pin
#define greenbutton 8   // green button pin
#define bluebutton 9    // blue button pin

const uint8_t buttonPins[4] = { yellowbutton, redbutton, greenbutton, bluebutton };

bool EnableButtons = true;  // if controlled externally (by CC) disable all buttons
uint8_t greentristate = 1;  // 0=off, 1=null, 2=on
bool redstate = false;      // is redbutton pressed?
bool yellowstate = false;   // is yellowbutton pressed?
bool greenstate = false;    // is greenbutton pressed?
bool bluestate = false;     // is bluebutton pressed?

uint8_t buttonCC[4];             // 0=red, 1=yellow, 2=blue, 3=green
int8_t numbuttonspressedCC = 0;  // number of externally pressed buttons
uint8_t mapButtonSelect = 0;     // 0=red, 1=yellow, 2=blue, 3=green

// menu
uint8_t menuitem = 2;        // is the number associated with every voice in the main menu. 0 to 18
uint8_t menunumber = 0;      // 0=mainscreen, 1=menu, 2=submenu, 3=load\save menu, 4=inter-recording popup
uint8_t savemode = 0;        // 0=bake, 1=clone, 2=new, 3=save, 4=load, 5=delete
uint8_t savecurX = 0;        // cursor position in load/save menu 0-5
uint8_t savepage = 0;        // page number for external eeprom
bool confirmation = false;   // confirmation popup in load/save menu
bool full = false;           // is the selected save slot full?
int8_t curpos = 0;           // cursor position in songmode, 0-7
bool StartMenuTimer = true;  // activate the go-to-menu timer
uint8_t Jittercur = 1;       // cursor position in the jitter submenu
bool cloneCur = false;       // cursor position in the clone seq submenu
#define goUp 2
#define goDown 1

// time keeping
uint8_t clockTimeout;              // variable for keeping track of external/internal clock
bool internalClock = true;         // is the sequencer using the internal clock?
uint8_t StepSpeed = 1;             // 0=2x, 1=1x, 2=3/4, 3=1/2
bool FixSync = false;              // tries to re-allign the sequence when changing step-speeds
bool flipflopEnable;               // part of the frameperstep flipflop
uint8_t sendrealtime = 1;          // send midi realtime messages. 0=off, 1=on (@internalclock, only if playing), 2=on (@internalclock, always)
int8_t GlobalStep;                 // keep track of note steps for metronome and tempo indicator
uint8_t tSignature = 4;            // time signature for LED indicator/metronome and beats, 1 - 8 (1*/4, 2*/4..to 8/4)
int8_t GlobalDivison = 4;          // how many steps per beat
int8_t countBeat;                  // keep track of the time beats
int8_t countStep;                  // keep track of note steps in seq/song mode
int8_t IntercountStep;             // keep track of note steps while inter-recording
int8_t countTicks;                 // the frame number (frame=external clock or 24 frames per quarter note)
uint8_t ticksPerStep = 6;          // how many clock ticks to count before the sequencer moves to another step.
uint8_t flip = 6;                  // part of the frameperstep's flipflop
uint8_t flop = 6;                  // part of the frameperstep's flipflop
bool swing = false;                // is swing enabled?
uint8_t BPM = 120;                 // beats per minute for internalclock - min 20, max 250 bpm
bool playing = false;              // is the sequencer playing?
uint8_t snapmode = 1;              // when play/stop the next sequence in live mode. 0=pattern, 1=up-beat, 2=beat
bool start = false;                // dirty fix for a ableton live 10 bug. becomes true once at sequence start and send a sync command
const uint8_t iterations = 8;      // how many BPM "samples" to averege out for the tap tempo. more = more accurate
uint8_t BPMbuffer[iterations];     // BPM "samples" buffer for tap tempo
volatile bool stepEnable = false;  // keep track of internal timer clicks

// arpeggiator
const uint8_t holdNotes = 10;    // maximum number of notes that can be arpeggiated
uint8_t activeNotes[holdNotes];  // contains the MIDI notes the arpeggiator plays
bool ArpDirection;               // alternate up-down for arpstyle 3 & 4
uint8_t arpstyle = 0;            // 0=up, 1=down, 2=up-down, 3=down-up, 4=up+down, 5=down+up, 6=random
uint8_t arpcount;                // number of times the arpeggio repeats
uint8_t arprepeat = 1;           // number of times the arpeggios gets trasposed by "stepdistance"
int8_t stepdistance = 12;        // distance in semitones between each arp repeat, -12 to +12
int8_t numNotesHeld = 0;         // how many notes are currently pressed on the keyboard
uint8_t numActiveNotes = 0;      // number of notes currently stored in the holdNotes array
uint8_t trigMode = 0;            // 0=hold, 1=trigger, 2=retrigged

// sequencer
bool recording = false;                          // is the sequencer in the recording state?
uint8_t rolandLowNote;                           // for 'roland' mode, the sequence is transposed based on the lowest note recorded
uint8_t rolandTransposeNote;                     // the last note number received for transposing the sequence
uint8_t seqLength = 16;                          // the number of steps the sequence is looped for - can be less than maxSeqLength
uint8_t NewSeqLength = 16;                       // 1-32 seq.length of the newly created song
uint8_t currentSeq = 0;                          // the currently selected sequence row from the big matrix below. 0=seq1, 1=seq2, 2=seq3, 3=seq4
uint8_t newcurrentSeq = 0;                       // the next sequence is going to play in live mode. 0-7
uint8_t currentSeqDestination = 0;               // destination sequence for the cloning function
uint8_t currentSeqSource = 0;                    // source sequence for the cloning function
const uint8_t maxSeqLength = 32;                 // the total possible number of steps (columns in the big matrix below)
const uint8_t numberSequences = 8;               // the number of total sequences
uint8_t noteSeq[numberSequences][maxSeqLength];  // sequences arrays
const uint8_t patternLength = 8;                 // number of patterns
uint8_t songPattern[patternLength];              // pattern array
int8_t pattern = 0;                              // currently playing pattern
bool chainrec = false;                           // sequential recording in song mode?
bool lockpattern = false;                        // block the current pattern from sequencing
bool nopattern = false;                          // inhibit current pattern from sequencing only once

// notes
int8_t pitch = 0;                // pitch transposition: -12 to +12
bool pitchmode = false;          // 0=before scale, 1=scale root
uint8_t scale = 0;               // 0=linear, 1=penta.major, 2=penta.minor, 3=major, 4=minor, 5=harmonic minor, 6=locrian, 7=lydian, 8=dorian, 9=phrygian, 10=inverted, 11=hexatonal
int8_t posttranspose = 0;        // post-scale pitch transposition: -12 to +12
uint8_t noteLengthSelect = 3;    // set the notelength, 0=random, 1=20%, 2=40%, 3=60%, 4=80%, 5=100%, 6=120%
bool sortnotes = true;           // sort the ActiveNotes array?
bool muted = false;              // temporarily suspend playing any notes from the sequencer
bool lockmute = false;           // keep the mute muted. only in arp, seq & song mode
bool TrigMute = false;           // toggle mute in sync with snapmode, in live mode
bool sustain = false;            // hold notes also if trigmode > 0
const uint8_t queueLength = 8;   // the number of notes that can overlap if notelenght is 120%
int8_t cronNote[queueLength];    // stores the notes playing
int8_t cronLength[queueLength];  // stores the amount of time a note has left to play
uint8_t jitrange = 0;            // jitter range 0-24
uint8_t jitprob = 0;             // jitter probability 0-10 (0-100%)
uint8_t jitmiss = 0;             // probability of a note to be not played (0-90%)

// general
uint8_t modeselect = 0;     // 0=arp.mode, 1=rec.mode, 2=song mode, 3=live mode
uint8_t premodeselect = 0;  // pre-selection of the mode in submenu

// drum mode
bool drumMode = false;            // is drum mode enabled?
bool drumModeSelect = false;      // pre-selection for drum mode
const uint8_t drumSlots = 8;      // number of drum slots available. must be 8 (8bit)
uint8_t DrumNotes[drumSlots];     // midi notes associated to each drum slot
bool ActiveDrumNotes[drumSlots];  // keep track of the currently pressed midi drum notes

bool editorPopup = false;    // normal popup or beat editor popup?
uint8_t currentSeqEdit = 0;  // sequence that you are editing in beat editor

uint8_t colum = 0;   // vertical position in beat editor
uint8_t row = 0;     // orizontal position in beat editor
uint8_t frame = 0;   // vertical page/section in beat editor
uint8_t window = 0;  // orizontal page/section in beat editor

void setup() {  // initialization setup

  DDRD &= ~(1 << PD5);  // disable LED_BUILTIN_TX (on-board LED) (set as input)

  EEPROM.init();

  // oled screen initialization
  oled.init();
  oled.clear();
  oled.setSize(3);

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

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(LEDs[i], OUTPUT);              // initialize LEDs
    pinMode(buttonPins[i], INPUT_PULLUP);  // initialize buttons
  }

  // speaker initialization
  DDRF |= (1 << PF4);    // set pin 21 (A3) as output
  PORTF &= ~(1 << PF4);  // set pin 21 (A3) LOW

  if (!(digitalRead(yellowbutton) && digitalRead(bluebutton))) ResetEEPROM();  // yellowbutton + bluebutton, initialize the EEPROM with default values

  // load default settings from EEPROM
  midiChannel = EEPROM.read(0);
  sendrealtime = EEPROM.read(1);
  soundmode = EEPROM.read(2);
  syncport = EEPROM.read(3);

  for (uint8_t i = 0; i < 8; i++) {
    if (i < 4) buttonCC[i] = EEPROM.read(4 + i);  // read from eeprom cc-mapping
    DrumNotes[i] = EEPROM.read(8 + i);            // initialize the drum notes from eeprom
    BPMbuffer[i] = BPM;                           // initialize the tap tempo array
  }

  AllLedsOff();

  SetSound(soundmode);
  SetSyncPort(syncport);

  SetBPM(BPM);
  ClearSeqPatternArray();
  SynthReset();
  Startposition();

  Bip(2);  // Startup Sound
  oled.setCursorXY(2, 0);
  oled.println(F("ARPMINI\n\r   +"));
  oled.setSize(1);
  oled.setCursorXY(25, 56);
  oled.print(F("FIRMWARE "));
  oled.printF(version);

  delay(2000);
  PrintMainScreen();
}

void safedigitalWrite(uint8_t pin, bool state) {  // avoid digitalwrite and i2c at the same time (external eeprom)

  if (!EEPROM.busy) digitalWrite(pin, state);
}

void SystemReset() {  // restart system

  asm volatile(" jmp 0");
}

ISR(TIMER1_COMPA_vect) {  // hardware timer for internal clock

  stepEnable = true;
}

void SetBPM(uint8_t tempo) {  // change Timer1 speed to match BPM (20-250)

  OCR1A = ((250000UL * 5) - tempo) / (2 * tempo);
}

void loop() {  // run continuously

  MIDI.read();
  MIDI2.read();

  DebounceButtons();
  ScreenOnTimer();
  GoToMenuTimer();

  if (stepEnable) {
    stepEnable = false;
    HandleInternalClock();
  }
}

void ResetEEPROM() {  // initialize the EEPROM with default values

  for (uint8_t i = 0; i < memorySlots; i++) {

    if (i < 4) {
      EEPROM.update(i, 1);           // midiChannel, sendrealtime, soundmode, syncport
      EEPROM.update(4 + i, i + 30);  // buttons cc
    }

    EEPROM.update(eepromaddress(16, i), 0);  // songs addresses
  }

  for (uint8_t i = 0; i < drumSlots; i++) {
    EEPROM.update(8 + i, i + 48);
  }
}

unsigned int eepromaddress(unsigned int address, uint8_t slot) {  // calculate eeprom addresses (address, slot number 0-5)

  uint16_t slotsize = (numberSequences * maxSeqLength) + 16;
  return (slotsize * slot) + address;
}

void AllLedsOn() {  // turn on all LEDs

  for (uint8_t i = 0; i <= 3; i++) {
    safedigitalWrite(LEDs[i], HIGH);
  }
}

void AllLedsOff() {  // turn off all LEDs

  for (uint8_t i = 0; i <= 3; i++) {
    safedigitalWrite(LEDs[i], LOW);
  }
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
        if (modeselect == 3) menuitem = 2;
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
    if (menunumber == 0 && !recording && !playing && internalClock) {
      oled.sendCommand(OLED_DISPLAY_OFF);
    } else {
      oled.setContrast(1);
    }
  }
}

void SynthReset() {  // clear cache

  for (uint8_t i = 0; i < holdNotes; i++) {
    activeNotes[i] = 0;
    if (i < queueLength) {
      cronNote[i] = -1;
      cronLength[i] = -1;
    }
  }
  numActiveNotes = 0;
  AllNotesOff();
}

void AllNotesOff() {  // send allnoteoff control change

  MIDI.sendControlChange(123, 0, midiChannel);
}

void ConnectPort(uint8_t portselect) {  // connects sync ports

  if (portselect == 1) {
    MIDI.setHandleClock(HandleExternalClock);
    MIDI.setHandleStart(HandleStart);
    MIDI.setHandleStop(HandleStop);
    MIDI.setHandleContinue(HandleContinue);
  } else if (portselect == 2) {
    MIDI2.setHandleClock(HandleExternalClock);
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

  for (uint8_t i = 1; i <= 2; i++) {
    if (i == port) ConnectPort(i);
    else DisconnectPort(i);
  }
}

void ScreenBlink() {  // blinks screen

  oled.sendCommand(OLED_DISPLAY_OFF);
  oled.sendCommand(OLED_DISPLAY_ON);
  confirmsound = true;
}

void PrintTitle() {  // title's printing setup

  oled.setSize(2);
  oled.invertText(1);
}

void PrintBottomText() {  // bottom small text printing setup

  oled.setSize(2);
  oled.setCursorXY(0, 48);
}

void SetSound(uint8_t mode) {  // sound settings

  sound = (mode != 3);
  uisound = (mode == 1);
}

void Bip(uint8_t type) {  // sounds

  if (sound) {
    if (uisound) {
      if (type == 1) tone8.tone(3136, 1);        // click
      else if (type == 2) tone8.tone(2637, 10);  // startup/confirmation
    }
    if (type == 3) tone8.tone(3136, 5);       // metronome
    else if (type == 4) tone8.tone(2349, 5);  // metronome
  }
}

void TapTempo() {  // calculate tempo based on the tapping frequency

  static unsigned long lastTapTime = 0;
  static uint8_t bufferIndex = 0;
  unsigned long currentTapTime = millis();
  unsigned long tapInterval = currentTapTime - lastTapTime;

  if (tapInterval < 1500 && tapInterval > 240) {  // if the time between taps is not too long or too short

    BPMbuffer[bufferIndex] = 60250 / tapInterval;  // store iteration
    bufferIndex = (bufferIndex + 1) % iterations;  // go to the next slot

    unsigned int BPMsum = 0;  // sum all iterations
    for (uint8_t i = 0; i < iterations; i++) {
      BPMsum += BPMbuffer[i];
    }

    BPM = BPMsum / iterations;  // calculate averege
    SetBPM(BPM);
    SubmenuSettings(menuitem, 0);
  }

  lastTapTime = currentTapTime;  // store last interval
}

void HandleInternalClock() {  // internal clock

  if (internalClock) {
    if ((sendrealtime == 1 && playing) || (sendrealtime == 2)) MIDI.sendRealTime(midi::Clock);
    RunClock();
  } else {
    clockTimeout++;
    if (clockTimeout > 100) {
      HandleStop();
    }
  }
}

void HandleExternalClock() {  // external clock

  if (!internalClock) {
    clockTimeout = 0;
    if (sendrealtime) MIDI.sendRealTime(midi::Clock);
    RunClock();
  }
}

void RunClock() {  // main clock

  bool clockenable = (internalClock && playing) || (!internalClock);

  if (clockenable) countTicks = (countTicks + 1) % ticksPerStep;

  if (countTicks == 0) {
    if (start) {
      start = false;
      if (sendrealtime) MIDI.sendSongPosition(0);  // make ableton live happy, without would be slightly out of sync
    }
    if ((modeselect != 3 && clockenable) || modeselect == 3) HandleStep();
  }

  if (countTicks == 2 && playing) {  // turn yellowLED off before 2 ticks - Tempo indicator

    if ((modeselect != 3 && (menunumber == 0 || menunumber == 5)) || (modeselect == 3 && menunumber == 5)) safedigitalWrite(yellowLED, LOW);
    else if (modeselect == 3 && menunumber == 0) AllLedsOff();
  }

  if (modeselect < 2) {  // not song & live mode
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

  static const uint8_t flipSwing[] = { 4, 8, 8, 16 };
  static const uint8_t noSwingValues[] = { 3, 6, 8, 12 };
  static const uint8_t flopSwing[] = { 2, 4, 8, 8 };
  static const uint8_t globalDiv[] = { 8, 4, 3, 2 };

  if (swing) {
    flip = flipSwing[StepSpeed];
    flop = flopSwing[StepSpeed];
  } else {
    flip = noSwingValues[StepSpeed];
    flop = flip;
  }
  GlobalDivison = globalDiv[StepSpeed];
}

void FlipFlopFPS() {  // alternates ticksPerStep values (for swing)

  flipflopEnable = !flipflopEnable;
  if (flipflopEnable) {
    if (!FixSync) SetTicksPerStep();
    ticksPerStep = flip;
  } else ticksPerStep = flop;
}

uint8_t SetNoteLength() {  // set the note duration

  uint8_t noteLength;
  static const uint8_t noteLengths[] = { 55, 35, 30, 25, 20, 15 };  // noteLengths

  if (noteLengthSelect == 0) {  // random
    noteLength = Random.get(15, 55);
  } else noteLength = noteLengths[noteLengthSelect - 1];

  if (swing && StepSpeed != 2) {
    if (flipflopEnable) noteLength = noteLength - 5;
    else noteLength = noteLength + 10;
  }

  if (StepSpeed == 0) noteLength = noteLength * 2;
  else if (StepSpeed == 2) noteLength = noteLength - 5;
  else if (StepSpeed == 3) noteLength = noteLength / 2;

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
  }

  FlipFlopFPS();

  if (modeselect == 0) {  // arp.mode

    if (numActiveNotes > 0) {
      SetArpStyle(arpstyle);  // arp sequencer
      if (!muted && playing) {
        QueueNote(activeNotes[countStep]);  // enqueue and play
      }
    } else countStep = -1;
  }

  else {  // rec, song & live mode

    countStep = (countStep + 1) % seqLength;  // seq. sequencer

    if (!muted && playing && !recording && drumMode) {

      for (uint8_t i = 0; i < drumSlots; i++) {  // play drums in sync
        if (ActiveDrumNotes[i]) {
          QueueNote(DrumNotes[i]);
          ActiveDrumNotes[i] = false;
        }
      }
    }

    if (modeselect == 1 || modeselect == 2) {
      if (recording) {
        if (playing && bluestate && menunumber == 0) {  // if recording, bluebutton delete note
          noteSeq[currentSeq][countStep] = 0;
        }

        if (!playing && internalClock) {  // if recording and not playing, yellowLED indicate steps
          if (countStep % 4 == 0) {
            safedigitalWrite(yellowLED, HIGH);
          } else safedigitalWrite(yellowLED, LOW);
        }
      }

      if (modeselect == 2) {  // call pattern sequencer
        if (countStep == 0) {
          if ((!recording) || (recording && chainrec && !nopattern)) {
            HandlePattern();
          } else nopattern = false;
        }
      }
    }

    else if (modeselect == 3) {
      if ((snapmode == 0 && countStep == 0) || (snapmode == 1 && GlobalStep == 0 && countBeat == (tSignature - 1)) || (snapmode == 2 && GlobalStep == 0)) {

        if (TrigMute) {
          TrigMute = false;
          muted = !muted;
        }

        if (currentSeq != newcurrentSeq) {
          currentSeq = newcurrentSeq;
          muted = false;
          if (menunumber == 0) PrintPatternSequenceCursor();
        }
      }
    }

    if ((playing && !muted) || (!playing && recording)) {
      uint8_t note = noteSeq[currentSeq][countStep];
      if (note > 0) {
        if (!drumMode) {
          int8_t offset = recording ? 0 : (rolandTransposeNote - rolandLowNote);
          QueueNote(note + offset);
        } else {
          for (uint8_t i = 0; i < drumSlots; i++) {
            if (bitRead(note, i)) {
              QueueNote(DrumNotes[i]);
            }
          }
        }
      }
    }
  }

  if (GlobalStep == 0) {
    HandleBeat();
  }

  if (menunumber == 0 && modeselect == 3) {  // live mode blinking

    if (GlobalStep == 0 || (TrigMute && !(GlobalStep % 2))) {
      if (playing) {
        if (!muted || TrigMute) {
          if (currentSeq < 4) safedigitalWrite((LEDs[currentSeq]), HIGH);
          else if (currentSeq == 4) {
            safedigitalWrite(yellowLED, HIGH);
            safedigitalWrite(redLED, HIGH);
          } else if (currentSeq == 5) {
            safedigitalWrite(blueLED, HIGH);
            safedigitalWrite(greenLED, HIGH);
          } else if (currentSeq == 6) {
            safedigitalWrite(blueLED, HIGH);
            safedigitalWrite(redLED, HIGH);
          } else if (currentSeq == 7) {
            safedigitalWrite(yellowLED, HIGH);
            safedigitalWrite(greenLED, HIGH);
          }
        } else AllLedsOn();
      }
    }
  }
}

void HandleBeat() {  // tempo indicator and metronome

  countBeat = (countBeat + 1) % tSignature;

  if (((menunumber == 0 || menunumber == 5) && modeselect != 3) || (menunumber == 5 && modeselect == 3)) {
    if (playing) safedigitalWrite(yellowLED, HIGH);
  }

  if (playing && (recording || metro)) Metronome();
}

void HandlePattern() {  // pattern sequencer in songmode

  if (!lockpattern) {
    pattern = (pattern + 1) % patternLength;

    while (songPattern[pattern] == 0) {  // skip if empty
      pattern = (pattern + 1) % patternLength;
    }
  }

  if (songPattern[pattern]) currentSeq = songPattern[pattern] - 1;  // if current pattern is active set the sequence

  if (menunumber == 0) PrintPatternSequenceCursor();  // update screen
}

void Metronome() {  // manage the metronome

  if (countBeat == 0) Bip(3);
  else Bip(4);
}

void HandleStart() {  // start message - re-start the sequence

  if (sendrealtime) MIDI.sendRealTime(midi::Start);

  internalClock = false;
  playing = true;
  Startposition();
  UpdateScreenBPM();
}

void HandleContinue() {  // continue message - start the sequence

  if (sendrealtime) MIDI.sendRealTime(midi::Continue);

  internalClock = false;
  countTicks = 0;
  playing = true;
  if (recording) Startposition();
  UpdateScreenBPM();
}

void HandleStop() {  // stop the sequence and switch over to internal clock

  if (sendrealtime) MIDI.sendRealTime(midi::Stop);

  if (!internalClock) {
    if (menunumber == 0) {
      if (modeselect != 3) safedigitalWrite(yellowLED, LOW);  // turn yellowLED off before 2 ticks - Tempo indicator
      else AllLedsOff();
    }

    playing = false;
    internalClock = true;
    numNotesHeld = 0;
    UpdateScreenBPM();
  }
}

void StartAndStop() {  // manage starts and stops

  playing = !playing;

  if (internalClock) {
    if (playing) Startposition();
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

void Startposition() {  // called every time the sequencing starts or stops

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

  if (modeselect == 2) {
    if (recording) nopattern = true;
    else if (playing && !lockpattern) pattern = -1;
  }
}

void UpdateScreenBPM() {  // refresh BPM indicators

  if (menunumber == 0) {  // switch on display and restart screen-on timer
    StartScreenTimer = true;
    PrintBPMBar();
  } else if (menunumber == 2 && menuitem == 4) {  // refresh BPM page
    SubmenuSettings(menuitem, 0);
  }
}

void HandleCC(uint8_t channel, uint8_t cc, uint8_t value) {  // handle midi CC messages

  if (channel == midiChannel) {
    if ((cc != 64) && (cc != 123)) {  // not sustain pedal or all notes off cc

      if (menunumber == 2 && menuitem == 16) {  // if cc mapping mode
        EnableButtons = true;

        if (value > 0) {
          buttonCC[mapButtonSelect] = cc;
          SubmenuSettings(menuitem, 0);
          StartScreenTimer = true;
          Bip(2);
        }
      }

      else {  // external control
        bool* state = nullptr;

        for (uint8_t i = 0; i < 4; i++) {
          if (cc == buttonCC[i]) {
            if (i == 0) state = &redstate;
            else if (i == 1) state = &yellowstate;
            else if (i == 2) state = &bluestate;
            else if (i == 3) state = &greenstate;
            break;
          }
        }

        if (state) {
          *state = value;
          numbuttonspressedCC += (*state ? 1 : -1);
          ButtonsCommands(*state);
        }

        EnableButtons = (numbuttonspressedCC == 0);
        if (numbuttonspressedCC < 0) numbuttonspressedCC = 0;
      }

    }

    else if (cc == 64) {  // sustain pedal cc
      sustain = value;
      if (!playing || (playing && trigMode == 0)) MIDI.sendControlChange(cc, value, channel);  // pass through sustain pedal cc
    } else MIDI.sendControlChange(cc, value, channel);                                         // pass through every other cc
  } else MIDI.sendControlChange(cc, value, channel);                                           // pass through if different channel cc
}

void HandleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {  // handle a noteOn event

  if (channel == midiChannel) {

    numNotesHeld++;

    if (drumMode) {
      if (menunumber == 2 && menuitem == 5) {  // map drum notes menu
        DrumNotes[curpos] = note;
        SubmenuSettings(menuitem, 0);
        StartScreenTimer = true;
        Bip(2);
      }

      for (uint8_t i = 0; i < drumSlots; i++) {               // check if incoming note is in the drum notes range
        if (note == DrumNotes[i]) ActiveDrumNotes[i] = true;  // update array
      }
    }

    if (!playing) {

      MIDI.sendNoteOn(TransposeAndScale(note), velocity, channel);  // bypass

      if (modeselect != 0 && recording) {  // recording & !playing

        if (countStep == -1) HandleStep();
        if (!drumMode) noteSeq[currentSeq][countStep] = note;
        else {
          for (uint8_t i = 0; i < drumSlots; i++) {
            if (ActiveDrumNotes[i]) {
              bitToggle8(noteSeq[currentSeq][countStep], i);
              ActiveDrumNotes[i] = false;
            }
          }
        }
      }

    } else {  // if playing

      if (modeselect < 2 && trigMode > 0) {  // not song & live mode
        muted = false;
        arpcount = 0;
        if (trigMode == 2 && numNotesHeld == 1) {  // trigmode 2 retrig start sequence
          Startposition();
          if (internalClock && sendrealtime) {
            MIDI.sendSongPosition(0);
          }
        }
      }

      if (modeselect == 0) {  // arp mode

        if (numNotesHeld == 1) {  // when first key is pressed
          numActiveNotes = 0;
          for (uint8_t i = 0; i < holdNotes; i++) {  // clear the activeNotes array
            activeNotes[i] = 0;
          }
        }

        if (numActiveNotes < holdNotes) {  // store the note only if the activeNotes array is not full
          activeNotes[numActiveNotes] = note;
          numActiveNotes++;
          SortArray();  // sort the activeNotes array
        }
      }

      else {  // not arp mode & playing

        if (recording) {
          uint8_t stepIndex = (countTicks + 2 > (ticksPerStep / 2)) ? (countStep + 1) % seqLength : countStep;

          if (!drumMode) {
            noteSeq[currentSeq][stepIndex] = note;
          } else {
            for (uint8_t i = 0; i < drumSlots; i++) {
              if (ActiveDrumNotes[i]) {
                bitSet8(noteSeq[currentSeq][stepIndex], i);
                ActiveDrumNotes[i] = false;
              }
            }
          }

          if (stepIndex == countStep) {
            QueueNote(note);
          }
        } else rolandTransposeNote = note;  // transpose to note only if playing and not recording
      }
    }

  } else MIDI.sendNoteOn(note, velocity, channel);  // bypass if different channel
}

void HandleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {  // handle a noteOff event

  if (channel == midiChannel) {

    numNotesHeld--;
    if (numNotesHeld < 0) numNotesHeld = 0;

    if (drumMode) {
      for (uint8_t i = 0; i < drumSlots; i++) {                // check if incoming note is in the drum notes range
        if (note == DrumNotes[i]) ActiveDrumNotes[i] = false;  // update array
      }
    }

    if (!playing) MIDI.sendNoteOff(TransposeAndScale(note), velocity, channel);  // pass trough note off messages

    else {  // if playing

      if (modeselect == 0 && trigMode > 0 && !sustain) {  // arp mode and playing
        if (numNotesHeld == 0) muted = true;
        for (uint8_t i = 0; i < holdNotes; i++) {
          if (activeNotes[i] == note) {
            activeNotes[i] = 0;
            numActiveNotes--;
            SortArray();
          }
        }
      }
    }

  } else MIDI.sendNoteOff(note, velocity, channel);  // bypass if different channel
}

void SortArray() {  // sort activeNotes array

  if (sortnotes) {  // sort from small to large
    for (uint8_t i = 0; i < holdNotes - 1; i++) {
      for (uint8_t j = 0; j < holdNotes - 1; j++) {
        if (activeNotes[j + 1] < activeNotes[j] && activeNotes[j + 1] > 0) {
          int8_t temp = activeNotes[j + 1];
          activeNotes[j + 1] = activeNotes[j];
          activeNotes[j] = temp;
        }
      }
    }
  }

  for (uint8_t i = 0; i < holdNotes - 1; i++) {  // remove duplicates and replace them with 0
    if ((activeNotes[i] > 0) && (activeNotes[i] == activeNotes[i + 1])) {
      activeNotes[i + 1] = 0;
      numActiveNotes--;
    }
  }

  for (uint8_t i = 0; i < holdNotes - 1; i++) {  // shift all the 0  to the right
    if (activeNotes[i] == 0) {
      int8_t temp = activeNotes[i + 1];
      activeNotes[i + 1] = activeNotes[i];
      activeNotes[i] = temp;
    }
  }
}

int SetScale(int note, uint8_t scale) {  // adapt note to fit in to a music scale

  static const int8_t scaleOffsets[][13] PROGMEM = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },             // Scale 0: Linear
    { 0, -1, -2, -1, -2, -1, -2, -3, -1, -2, -1, -2 },  // Scale 1: Pentatonic Major
    { 0, -1, -2, 0, -1, 0, -1, 0, -1, -2, 0, -1 },      // Scale 2: Pentatonic Minor
    { 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0 },        // Scale 3: Major
    { 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, 0, -1 },        // Scale 4: Minor
    { 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, +1, 0 },        // Scale 5: Harmonic Minor
    { 0, -1, -2, 0, -1, 0, 0, 0, -1, -2, 0, -1 },       // Scale 6: Blues
    { 0, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1 },        // Scale 7: Locrian
    { 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0 },        // Scale 8: Lydian
    { 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, 0, -1 },        // Scale 9: Dorian
    { 0, 0, -1, 0, 0, 0, -1, 0, 0, -1, 0, -1 },         // Scale 10: Phrigian
    { 11, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -11 },     // Scale 11: Inverted
    { 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1 }        // Scale 12: Hexatonal
  };

  uint8_t reminder = note % 12;

  int8_t offset = pgm_read_byte(&scaleOffsets[scale][reminder]);  // read from PROGMEM

  return note + offset;
}

int8_t Jitter(uint8_t range, uint8_t probability) {  // add random variations

  int8_t jitter = 0;
  uint8_t chance = Random.get(1, 10);
  if (chance <= probability) jitter = Random.get(-range, range);
  return jitter;
}

bool ProbabilityMiss(uint8_t missProb) {  // return true or false randomly

  uint8_t chance = Random.get(1, 11);
  if (missProb < chance) return true;
  else return false;
}

uint8_t TransposeAndScale(int note) {  // apply transposition and scale

  if (drumMode) return note;

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

  static const uint8_t defaultVelocity = 64;

  if (modeselect == 0) {  // only in arp mode
    if (arpcount > 0) note = note + ((arpcount - 1) * stepdistance);
  }

  note = note + Jitter(jitrange, jitprob);  // apply jitter
  note = TransposeAndScale(note);           // apply scale & transpositions

  if (modeselect == 0 && recording) {  // inter-recording
    IntercountStep++;
    noteSeq[currentSeq][IntercountStep] = note;
    if (IntercountStep == seqLength) {
      IntercountStep = 0;
      recording = false;
      ManageRecording();
    }
  }

  if (ProbabilityMiss(jitmiss)) {  // pass note or not?
    for (uint8_t i = 0; i < queueLength; i++) {
      if ((cronLength[i] < 0) || (cronNote[i] == note)) {
        MIDI.sendNoteOn(note, defaultVelocity, midiChannel);
        if (cronNote[i] >= 0) MIDI.sendNoteOff(cronNote[i], 0, midiChannel);
        cronNote[i] = note;
        cronLength[i] = 100;
        return;
      }
    }
  }
}

void ManageRecording() {  // recording ending setup

  if (!recording) {
    rolandLowNote = 128;  // get the low note for transpose
    for (uint8_t i = 0; i < maxSeqLength; i++) {
      if ((noteSeq[currentSeq][i] < rolandLowNote) && (noteSeq[currentSeq][i] > 0)) {
        rolandLowNote = noteSeq[currentSeq][i];
      }
    }
  }
  rolandTransposeNote = rolandLowNote;

  safedigitalWrite(redLED, recording);  // redLED

  if (/*internalClock &&*/ !playing) {
    Startposition();
  }
}

void SetArpStyle(uint8_t style) {  // arpeggiator algorithms

  if (style % 2 != 0) {
    if (countStep == -1) countStep = numActiveNotes;
  }

  uint8_t arpup = (countStep + 1) % numActiveNotes;
  uint8_t arpdown = (numActiveNotes + (countStep - 1)) % numActiveNotes;
  uint8_t arpcountplus = 1 + (arpcount % arprepeat);
  int8_t activenotes = numActiveNotes - 1;

  switch (style) {
    case 0:  // up
      countStep = arpup;
      if (countStep == 0) arpcount = arpcountplus;
      break;

    case 1:  // down
      countStep = arpdown;
      if (countStep == activenotes) arpcount = arpcountplus;
      break;

    case 2:  // up-down
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

    case 3:  // down-up
      if (ArpDirection) {
        countStep = arpdown;
        if (countStep == activenotes) arpcount = arpcountplus;
        if (countStep == 0) ArpDirection = false;
      } else {
        countStep = arpup;
        if (countStep == activenotes) arpcount = arpcountplus;
        if ((countStep + 1) == numActiveNotes) ArpDirection = true;
      }
      break;

    case 4:  // up+down
      if (ArpDirection) {
        countStep++;
        if (countStep > activenotes) {
          countStep = activenotes;
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

    case 5:  // down+up
      if (ArpDirection) {
        countStep--;
        if (countStep < 0) {
          countStep = 0;
          ArpDirection = false;
        }
      } else {
        countStep++;
        if (countStep > activenotes) {
          countStep = activenotes;
          ArpDirection = true;
          arpcount = arpcountplus;
        }
      }
      break;

    case 6:  // random
      countStep = Random.get(0, numActiveNotes);
      arpcount = Random.get(1, arprepeat + 1);
      break;
  }

  if (arpcount < 1) arpcount = 1;
}

void ClearSeqPatternArray() {  // clear all pattern and sequences arrays

  for (uint8_t i = 0; i < patternLength; i++) {  // clean SongPattern array
    if (i < 4) songPattern[i] = (i + 1);         // set default 1-2-3-4-1-2-3-4
    else songPattern[i] = (i - 3);
  }

  for (uint8_t j = 0; j < numberSequences; j++) {  // clean Seq arrays
    for (uint8_t i = 0; i < maxSeqLength; i++) {
      noteSeq[j][i] = 0;
    }
  }
}

void PrintMainScreen() {  // print menu 0 to the screen

  menunumber = 0;

  oled.clear();
  PrintTitle();

  if (modeselect < 2) oled.setCursorXY(4, 0);

  oled.printlnF(modenames[modeselect]);
  oled.invertText(0);

  if (modeselect == 0) {  // arp mode screen
    oled.setSize(3);

    if (arpstyle == 0) oled.setCursorXY(48, 19);
    else if (arpstyle == 1) oled.setCursorXY(28, 19);
    else if (arpstyle == 6) oled.setCursorXY(12, 19);
    else oled.setCursorXY(2, 19);

    oled.printF(arpmodes[arpstyle]);
    oled.setSize(2);
  }

  else if (modeselect == 1) {  // rec mode screen
    oled.setCursorXY(36, 16);
    oled.printF(seq);
    oled.print(currentSeq + 1);
    oled.setCursorXY(((seqLength > 9) ? 12 : 17), 32);
    oled.printF(length);
    oled.printF(space);
    oled.print(seqLength);
  }

  else if (modeselect == 2) {  // song mode screen
    PrintPatternSequence();
    PrintPatternSequenceCursor();
  }

  else if (modeselect == 3) {  // live mode screen
    if (lockmute) {
      lockmute = false;
      muted = false;
    }
    oled.shiftX();
    oled.println(F("1 5 7 6 3"));
    oled.shiftX();
    oled.print(F("2   8   4"));
    PrintPatternSequenceCursor();
  }

  PrintBPMBar();
}

void PrintBPMBar() {  // print BPM status bar to the screen

  oled.setSize(2);
  oled.clear(0, 54, 127, 63);

  if ((internalClock && BPM > 99) || (!internalClock)) {
    oled.setCursorXY(4, 48);
  } else oled.setCursorXY(11, 48);

  if (internalClock) oled.print(BPM);
  else oled.printF(ext);
  oled.shiftX();
  oled.printF(printbpm);
  oled.shiftX();
  oled.print(tSignature);
  oled.printF(fourth);
}

void PrintPatternSequence() {  // print the sequence to the screen - song mode

  for (uint8_t i = 0; i < patternLength; i++) {
    if (menunumber == 0) oled.setCursorXY((i * 16) + 2, 16);
    else oled.setCursorXY(i * 16, 30);
    if (songPattern[i] > 0) oled.print(songPattern[i]);
    else oled.printF(printx);
  }
}

void PrintPatternSequenceCursor() {  // print the cursor to the screen - song/live mode

  if (modeselect == 2) {  // song mode

    static uint8_t lastpattern = 0;

    if (pattern >= 0) {  // if outside the range don't print anything

      for (uint8_t i = 0; i < 2; i++) {  // clear previous cursor and set new position
        oled.setCursorXY((lastpattern * 16) + 2, 32);
        if (i) break;
        oled.printF(space);
        lastpattern = pattern;
      }

      if (!lockpattern) oled.printF(upcursor);
      else oled.print(F("="));
    }
  }

  else if (modeselect == 3) {  // live mode

    static uint8_t cursorX[8] = { 18, 18, 114, 114, 42, 90, 66, 66 };
    static uint8_t cursorY[8] = { 16, 32, 16, 32, 16, 16, 16, 32 };
    static uint8_t lastseq = 0;

    for (uint8_t i = 0; i < 2; i++) {  // clear previous cursor and set new position
      oled.setCursorXY(cursorX[lastseq], cursorY[lastseq]);
      if (i) break;
      oled.printF(space);
      lastseq = currentSeq;
    }

    oled.printF(printback);
  }
}

void PrintMenu(uint8_t item) {  // print main menu - menu 1

  menunumber = 1;

  oled.clear();
  oled.setSize(3);
  oled.printF(printnext);

  switch (item) {

    case 0:  // load-save
      oled.printlnF(file);
      break;

    case 1:  // mode select
      oled.printlnF(printmode);
      oled.printF(printselect);
      break;

    case 2:  // seq.select
      if (modeselect == 0) {
        oled.printlnF(arp);
        oled.printF(printstyle);
      }

      else if (modeselect == 1) {
        oled.printlnF(seq);
        oled.printF(printselect);
      }

      else if (modeselect == 2) {
        oled.printlnF(chain);
        oled.printF(editor);
      }

      else if (modeselect == 3) {
        if (playing) oled.printlnF(printstop);
        else oled.printlnF(printstart);
        oled.printF(printlive);
      }
      break;

    case 3:  // pitch
      if (!drumMode) oled.printlnF(printpitch);
      else {
        oled.println(F("BEAT"));
        oled.printF(editor);
      }
      break;

    case 4:  // tempo
      oled.printlnF(printtempo);
      break;

    case 5:  // scale / drum notes
      if (!drumMode) oled.printlnF(printscale);
      else {
        oled.println(F("DRUM*"));
        oled.print(F("NOTES"));
      }
      break;

    case 6:  // jitter
      oled.printlnF(printrandom);
      break;

    case 7:  // note lenght
      oled.printlnF(printnote);
      oled.printF(length);
      break;

    case 8:  // seq.length / arp steps
      if (modeselect == 0) {
        oled.printlnF(arp);
        oled.printF(steps);
      } else {
        oled.printlnF(seq);
        oled.printF(length);
      }
      break;

    case 9:  // arp/seq. speed
      oled.printlnF(speed);
      break;

    case 10:  // trig.mode / chain rec / snap mode
      if (modeselect == 2) {
        oled.printlnF(chain);
        oled.printF(rec);
      } else if (modeselect == 3) {
        oled.printlnF(printsnap);
        oled.printF(printmode);
      } else {
        oled.printlnF(printtrig);
        oled.printF(printmode);
      }
      break;

    case 11:  // swing
      oled.printlnF(printswing);
      break;

    case 12:  // metro
      oled.printlnF(printmetro);
      break;

    case 13:  // midi channel
      oled.printF(printmidi);
      break;

    case 14:  // midi realtime messages
      oled.printlnF(printsend);
      oled.printF(sync);
      break;

    case 15:  // sync port
      oled.printlnF(printext);
      oled.printF(sync);
      break;

    case 16:  // map buttons
      oled.printF(printmap);
      break;

    case 17:  // sound
      oled.printlnF(printsound);
      break;

    case 18:  // restart
      oled.printlnF(printreboot);
      break;
  }
}

void SubmenuSettings(uint8_t item, uint8_t dir) {  // print & handles changing settings in submenu - menu 2

  bool keyEnable = dir;

  menunumber = 2;

  oled.clear();
  oled.setSize(3);

  if (item == 0) oled.printF(printnext);
  else if ((item != 6 && item != 3 && item != 18) || (item == 3 && !drumMode)) oled.printF(printback);

  switch (item) {

    case 0:  // load-save

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (dir == goDown) {
          if (savemode < 5) savemode++;
        } else {
          if (savemode > 0) savemode--;
        }
      }

      //-----SCREEN COMMANDS----//
      if (modeselect == 0 && savemode < 2) savemode = 2;
      oled.printlnF(savemodes[savemode]);
      if ((savemode == 0 && modeselect == 1) || (modeselect == 2 && lockpattern) || (savemode == 1)) oled.printF(seq);
      else oled.printF(song);
      break;

    case 1:  // mode select

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (dir == goDown) {
          if (premodeselect < 3) premodeselect++;
        } else {
          if (premodeselect > 0) premodeselect--;
        }
      }

      if (drumMode && premodeselect == 0) premodeselect++;

      //-----SCREEN COMMANDS----//
      oled.printlnF(printmode);
      oled.printF(modes[premodeselect]);
      break;

    case 2:  // arp style / seq.select / edit song

      if (modeselect == 0) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (dir == goUp) {
              if (arpstyle > 0) arpstyle--;
            } else {
              if (arpstyle < 6) arpstyle++;
            }
          } else {
            if (dir == goUp) {
              sortnotes = true;
            } else {
              sortnotes = false;
            }
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(printstyle);
        oled.printF(arpmodes[arpstyle]);
        PrintBottomText();
        oled.printF(sortmodes[sortnotes]);
      }

      else if (modeselect == 1) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (dir == goUp) {
            if (currentSeq > 0) currentSeq--;
          } else {
            if (currentSeq < (numberSequences - 1)) currentSeq++;
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(seq);
        oled.print(currentSeq + 1);
      }

      else if (modeselect == 2) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {

          int8_t count = -1;

          for (uint8_t i = 0; i < patternLength; i++) {  // count number of active song-patterns (minus one)
            if (songPattern[i] > 0) count++;
          }

          if (!greenstate) {
            if (dir == goUp) {
              if (curpos < 7) curpos++;
            } else {
              if (curpos > 0) curpos--;
            }
          } else {
            if (dir == goUp) {
              if (songPattern[curpos] < numberSequences) songPattern[curpos]++;
            } else {
              if ((count && songPattern[curpos] > 0) || (!count && songPattern[curpos] > 1)) songPattern[curpos]--;
            }
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(chain);
        oled.setSize(2);
        PrintPatternSequence();
        oled.setCursorXY((curpos * 16), 48);
        oled.printF(upcursor);
      }

      else if (modeselect == 3) {
        StartAndStop();
        muted = playing;
        confirmsound = true;
        greentristate = 1;
        PrintMainScreen();
      }
      break;

    case 3:  // pitch / edit sequence

      if (drumMode) {
        currentSeqEdit = currentSeq;
        row = 0;
        colum = 0;
        window = 0;
        frame = 0;
        PrintBeatEditor();
        break;
      }

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (dir == goUp) {
            if (pitch < 12) pitch++;
          } else {
            if (pitch > -12) pitch--;
          }
        } else {
          if (dir == goUp) {
            pitchmode = true;
          } else {
            pitchmode = false;
          }
        }
        if (!playing) AllNotesOff();
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(printpitch);
      if (pitch > 0) oled.printF(plus);
      oled.print(pitch);

      if (pitchmode) {
        oled.printF(space);
        uint8_t normalizedTranspose = (pitch % 12 + 12) % 12;  // normalize the transpose value
        oled.printF(noteNames[normalizedTranspose]);
      }

      PrintBottomText();
      if (!pitchmode) oled.print(F("PRE-"));
      else oled.print(F("ROOT-"));
      oled.printF(printscale);
      break;

    case 4:  // tempo

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (dir == goUp) {
          if (!greenstate) BPM++;
          else BPM += 5;
        } else {
          if (!greenstate) BPM--;
          else BPM -= 5;
        }
        BPM = constrain(BPM, 20, 250);
        SetBPM(BPM);
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(printbpm);
      oled.print(BPM);
      if (!internalClock) oled.print(F(" INT"));
      break;

    case 5:  // scale / map drum notes

      if (!drumMode) {
        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (dir == goDown) {
              if (scale < 12) scale++;
            } else {
              if (scale > 0) scale--;
            }
          } else {
            if (dir == goUp) {
              if (posttranspose < 12) posttranspose++;
            } else {
              if (posttranspose > -12) posttranspose--;
            }
          }
          if (!playing) AllNotesOff();
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(printscale);
        oled.printF(scaleNames[scale]);
        PrintBottomText();
        oled.print(F("TRANSP."));
        if (posttranspose > 0) oled.printF(plus);
        oled.print(posttranspose);

      } else {  // drumMode

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (dir == goUp) {
              if (curpos < 7) curpos++;
            } else {
              if (curpos > 0) curpos--;
            }
          } else {
            for (uint8_t i = 0; i < drumSlots; i++) {
              EEPROM.update(8 + i, DrumNotes[i]);
            }
            ScreenBlink();
          }
        }

        //-----SCREEN COMMANDS----//
        oled.print(F("NO."));
        oled.print(DrumNotes[curpos]);
        oled.setSize(2);
        for (uint8_t i = 0; i < drumSlots; i++) {
          oled.setCursorXY(i * 16, 30);
          char letter = 'A' + i;
          oled.print(letter);
        }
        oled.setCursorXY((curpos * 16), 48);
        oled.printF(upcursor);
      }
      break;

    case 6:  // jitter

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (dir == goDown) {
            if (Jittercur < 3) Jittercur++;
          } else {
            if (Jittercur > 1) Jittercur--;
          }
        } else {
          if (dir == goUp) {
            if (Jittercur == 1) {
              if (jitrange < 24) jitrange++;
            } else if (Jittercur == 2) {
              if (jitprob < 10) jitprob++;
            } else if (Jittercur == 3) {
              if (jitmiss < 9) jitmiss++;
            }
          } else {
            if (Jittercur == 1) {
              if (jitrange > 0) jitrange--;
            } else if (Jittercur == 2) {
              if (jitprob > 0) jitprob--;
            } else if (Jittercur == 3) {
              if (jitmiss > 0) jitmiss--;
            }
          }
        }
      }

      //-----SCREEN COMMANDS----//
      PrintTitle();
      oled.println(F("  RANDOM  "));
      oled.invertText(0);
      oled.print(F(" RANG "));
      oled.println(jitrange);
      oled.print(F(" PROB "));
      oled.print(jitprob * 10);
      oled.printlnF(percent);
      oled.print(F(" MISS "));
      oled.print(jitmiss * 10);
      oled.printF(percent);
      oled.setCursorXY(0, Jittercur * 16);
      oled.printF(printnext);
      break;

    case 7:  // note lenght

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (dir == goUp) {
          if (noteLengthSelect < 6) noteLengthSelect++;
        } else {
          if (noteLengthSelect > 0) noteLengthSelect--;
        }
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(length);
      if (noteLengthSelect == 0)
        oled.printF(printrandom);
      else {
        oled.print(noteLengthSelect * 20);
        oled.printF(percent);
      }
      break;

    case 8:  // seq.length / arp steps
      if (modeselect != 0) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (dir == goUp) {
            if (seqLength < maxSeqLength) seqLength++;
          } else {
            if (seqLength > 1) seqLength--;
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(length);
        oled.print(seqLength);

      } else {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (dir == goUp) {
              if (arprepeat < 5) arprepeat++;
            } else {
              if (arprepeat > 1) arprepeat--;
            }
          } else {
            if (dir == goUp) {
              if (stepdistance < 12) stepdistance++;
            } else {
              if (stepdistance > -12) stepdistance--;
            }
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(steps);
        if (arprepeat > 1) oled.printF(plus);
        oled.print(arprepeat - 1);
        PrintBottomText();
        oled.print(F("DIST."));
        if (stepdistance > 0) oled.printF(plus);
        oled.print(stepdistance);
      }
      break;

    case 9:  // arp/seq. speed

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (dir == goUp) {
          if (StepSpeed > 0) StepSpeed--;
        } else {
          if (StepSpeed < 3) StepSpeed++;
        }
        if (playing) FixSync = true;
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(speed);
      oled.printF(speeds[StepSpeed]);
      break;

    case 10:  // trig.mode / chain rec

      if (modeselect < 2) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (dir == goUp) {
            if (trigMode > 0) trigMode--;
          } else {
            if (trigMode < 2) trigMode++;
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(printmode);
        oled.printF(trigmodes[trigMode]);
      }

      else if (modeselect == 2) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (dir == goUp) chainrec = true;
          else chainrec = false;
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(chain);
        if (chainrec) oled.printF(on);
        else oled.printF(off);
      }

      else if (modeselect == 3) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (dir == goUp) {
            if (snapmode < 2) snapmode++;
          } else {
            if (snapmode > 0) snapmode--;
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(printmode);
        oled.printF(snapmodes[snapmode]);
      }
      break;

    case 11:  // swing

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (dir == goUp) swing = true;
        else swing = false;
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(printswing);
      if (!swing) oled.printF(off);
      else oled.printF(on);
      break;

    case 12:  // metro

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (dir == goUp) metro = true;
          else metro = false;
        } else {
          if (dir == goUp) {
            if (tSignature < 8) tSignature++;
          } else {
            if (tSignature > 1) tSignature--;
          }
        }
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(printmetro);
      if (!metro) oled.printF(off);
      else oled.printF(on);
      PrintBottomText();
      oled.print(F("SIGN."));
      oled.print(tSignature);
      oled.printF(fourth);
      break;

    case 13:  // midi channel

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          AllNotesOff();
          if (dir == goUp) {
            if (midiChannel < 16) midiChannel++;
          } else {
            if (midiChannel > 1) midiChannel--;
          }
        } else {
          EEPROM.update(0, midiChannel);
          ScreenBlink();
        }
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("CH"));
      oled.print(midiChannel);
      break;

    case 14:  // send midi realtime messages

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (dir == goUp) {
            if (sendrealtime < 2) sendrealtime++;
          } else {
            if (sendrealtime > 0) sendrealtime--;
          }
        } else {
          EEPROM.update(1, sendrealtime);
          ScreenBlink();
        }
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("SEND"));
      oled.printF(sendmodes[sendrealtime]);
      break;

    case 15:  // sync port

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (internalClock) {
          if (!greenstate) {
            if (dir == goUp) {
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
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("PORT"));
      if (syncport == 0) oled.printF(off);
      else oled.print(syncport);
      break;

    case 16:  // map buttons

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (dir == goUp) {
            if (mapButtonSelect > 0) mapButtonSelect--;
          } else {
            if (mapButtonSelect < 3) mapButtonSelect++;
          }
        } else {
          for (uint8_t i = 0; i < 4; i++) {
            EEPROM.update(4 + i, buttonCC[i]);
          }
          ScreenBlink();
        }
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("MAP"));
      oled.printF(printCC);
      AllLedsOff();
      for (uint8_t i = 0; i <= 3; i++) {
        if (mapButtonSelect == i) {
          oled.print(buttonCC[i]);
          safedigitalWrite(LEDs[i], HIGH);
        }
      }
      break;

    case 17:  // sound

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (dir == goUp) {
            if (soundmode > 1) soundmode--;
          } else {
            if (soundmode < 3) soundmode++;
          }
        } else {
          EEPROM.update(2, soundmode);
          ScreenBlink();
        }
        SetSound(soundmode);
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("SOUND"));
      if (soundmode == 1) oled.printF(on);
      else if (soundmode == 2) oled.print(F("UI-"));
      if (soundmode != 1) oled.printF(off);
      break;

    case 18:  // restart
      SystemReset();
      break;
  }
}

void PrintLoadSaveMenu(uint8_t mode) {  // print load-save menu - menu 3

  menunumber = 3;

  if (mode == 0) {  // bake song
    confirmation = true;
    PrintPopup();
  } else {

    if (mode != 2) oled.setCursorXY(0, 0);
    else oled.setCursorXY(5, 0);
    PrintTitle();

    oled.printF(space);
    oled.printF(savemodes[mode]);
    if (mode != 5) oled.printF(space);

    if (mode == 1) {  // clone seq.
      oled.print(F("SEQ "));
      oled.invertText(0);
      oled.setCursorXY(23, 16);

      if (!cloneCur) {
        oled.printF(printnext);
      } else {
        oled.printF(space);
        oled.clear(0, 32, 127, 48);
      }

      oled.printF(seq);
      oled.print(currentSeqSource + 1);
      oled.setCursorXY(58, 32);
      oled.print(F("$"));
      oled.setCursorXY(23, 48);

      if (cloneCur) {
        oled.printF(printnext);
      } else oled.printF(space);

      oled.printF(seq);
      oled.print(currentSeqDestination + 1);
    }

    else if (mode == 2) {  // new song

      oled.print(F("SONG "));
      oled.clear(0, 32, 127, 63);
      oled.invertText(0);
      oled.setCursorXY(5, 16);
      oled.printF(seq);
      oled.printF(length);
      oled.setSize(3);
      if (NewSeqLength > 9) oled.setCursorXY(45, 32);
      else oled.setCursorXY(56, 32);
      oled.print(NewSeqLength);
    }

    else {  // save, load, delete song

      oled.print(F("@>"));
      char letter = 'A' + savepage;
      oled.print(letter);
      oled.printlnF(space);  // go down

      oled.invertText(0);

      oled.clear(48, 32, 65, 42);  // clean popup-area

      for (uint8_t i = 0; i < 6; i++) {
        uint8_t loc = i + (6 * savepage);
        if (i < 3) oled.setCursorXY(0, (i + 1) * 16);
        else oled.setCursorXY(65, (i - 2) * 16);

        uint8_t drumsong = (EEPROM.read(eepromaddress(16, loc)));

        if (drumsong) {
          if (loc < 9) {
            if (drumsong == 1) oled.print(F("SNG"));
            else oled.print(F("DRM"));
          } else {
            if (drumsong == 1) oled.print(F("SN"));
            else oled.print(F("DR"));
          }
          oled.print(loc + 1);
        } else oled.print(F("----"));

        if (loc == savecurX) oled.print(F("<"));
        else oled.printF(space);
      }
    }
  }
}

void PrintPopup() {  // print popups - menu 4

  oled.setSize(2);
  if (!editorPopup) oled.invertText(1);
  oled.setCursorXY(28, 32);

  if (menunumber == 3) {  // load-save menu

    if (savemode == 0 || savemode == 1) {  // bake, clone
      oled.setCursorXY(10, 32);
      oled.print(F(" PROCEED?"));
    }

    else if (savemode == 2) {  // new
      if (!drumModeSelect) oled.printF(printnext);
      else oled.printF(space);
      oled.print(F("SONG?"));
      oled.setCursorXY(28, 48);
      if (drumModeSelect) oled.printF(printnext);
      else oled.printF(space);
      oled.print(F("BEAT?"));
    }

    else if (savemode == 3) {  // save
      if (!full) oled.print(F(" SAVE?"));
      else {
        oled.setCursorXY(6, 32);
        oled.print(F(" REWRITE?"));
      }
    }

    else if (savemode > 3) {  // load & delete
      if (!full) {
        oled.setCursorXY(22, 32);
        oled.print(F(" EMPTY!"));
      } else {
        if (savemode == 4) oled.print(F(" LOAD?"));  // load
        else if (savemode == 5) {                    // delete
          oled.setCursorXY(16, 32);
          oled.print(F(" DELETE?"));
        }
      }
    }
  } else {  // inter-recording popup
    menunumber = 4;
    oled.print(F(" SEQ."));
    oled.print(newcurrentSeq + 1);
  }
  oled.invertText(0);
}

void PrintBeatEditor() {  // print beat editor - menu 5

  menunumber = 5;

  oled.setCursorXY(0, 0);
  oled.setSize(2);
  if (!row && !colum) oled.print(F("<SEQ"));
  else oled.print(F(">SEQ"));
  oled.print(currentSeqEdit + 1);
  oled.printF(space);

  for (uint8_t i = 0; i < 4; i++) {  // title window view
    if (i == window) oled.print(F("["));
    else oled.print(F("]"));
  }

  oled.println();  // go down

  for (uint8_t i = (frame * 3); i < ((frame + 1) * 3); i++) {  // print editor
    if (i > 7) {
      oled.clear(0, 54, 127, 63);
      break;
    }
    char letter = 'A' + i;
    oled.print(letter);
    oled.invertText(1);
    oled.print(F(";"));

    for (uint8_t j = (window * 8); j < ((window + 1) * 8); j++) {
      if (j > (seqLength - 1)) {
        oled.clear(((seqLength - (window * 8)) + 2) * 12, 16, 127, 63);
        break;
      }
      bool pos = (colum == j && row == i);
      if ((bitRead(noteSeq[currentSeqEdit][j], i)) == 0) {
        if (pos) oled.print(F(")"));
        else oled.print(F("]"));
      } else {
        if (pos) oled.print(F("("));
        else oled.print(F("["));
      }
    }
    oled.invertText(0);
    oled.println();  // go down
  }
}

void LoadSave(uint8_t mode, uint8_t number) {  // (bake/clone/new/save/load/delete, slot number 0-5)

  if (mode == 0) {  // bake song
    BakeSequence();
    ScreenBlink();
    PrintMainScreen();
  }

  else if (mode == 1) {  // clone seq.
    if (currentSeqSource != currentSeqDestination) {
      CloneSequence(currentSeqSource, currentSeqDestination);
    }
    ScreenBlink();
    PrintMainScreen();
  }

  else if (mode == 2) {  // new song

    playing = false;
    StepSpeed = 1;
    noteLengthSelect = 3;
    lockpattern = false;
    pattern = 0;
    currentSeq = 0;
    pitch = 0;
    pitchmode = false;
    scale = 0;
    posttranspose = 0;
    ClearSeqPatternArray();
    seqLength = NewSeqLength;
    drumMode = drumModeSelect;
    modeselect = 1;
    premodeselect = modeselect;
    ScreenBlink();
    PrintMainScreen();
  }

  else {

    if (mode == 3) {  // save song
      EEPROM.update((eepromaddress(16, number)), (drumMode + 1));
      EEPROM.update((eepromaddress(17, number)), pitchmode);
      EEPROM.update((eepromaddress(18, number)), (pitch + 12));
      EEPROM.update((eepromaddress(19, number)), scale);
      EEPROM.update((eepromaddress(20, number)), (posttranspose + 12));
      EEPROM.update((eepromaddress(21, number)), seqLength);
      EEPROM.update((eepromaddress(22, number)), BPM);
      EEPROM.update((eepromaddress(23, number)), swing);

      for (uint8_t i = 0; i < numberSequences; i++) {
        EEPROM.update((eepromaddress(i + 24, number)), songPattern[i]);  // save SongPattern array
        for (uint8_t j = 0; j < seqLength; j++) {
          EEPROM.update((eepromaddress(j + (32 * (i + 1)), number)), noteSeq[i][j]);  // save all Seq. arrays
        }
      }

      ScreenBlink();
    }

    else if (mode == 4) {  // load song
      if (full) {
        playing = false;
        lockpattern = false;
        pattern = 0;
        ClearSeqPatternArray();

        drumMode = (EEPROM.read(eepromaddress(16, number)) - 1);
        pitchmode = EEPROM.read(eepromaddress(17, number));
        pitch = (EEPROM.read(eepromaddress(18, number)) - 12);
        scale = EEPROM.read(eepromaddress(19, number));
        posttranspose = (EEPROM.read(eepromaddress(20, number)) - 12);
        seqLength = EEPROM.read(eepromaddress(21, number));
        BPM = EEPROM.read(eepromaddress(22, number));
        swing = EEPROM.read(eepromaddress(23, number));

        for (uint8_t i = 0; i < numberSequences; i++) {
          songPattern[i] = EEPROM.read(eepromaddress(i + 24, number));  // load SongPattern array
          for (uint8_t j = 0; j < seqLength; j++) {
            noteSeq[i][j] = EEPROM.read(eepromaddress(j + (32 * (i + 1)), number));  // load Seq.1 array
          }
        }

        SetBPM(BPM);
        modeselect = 2;
        premodeselect = modeselect;
        currentSeq = 0;
        ManageRecording();
        ScreenBlink();
      } else confirmation = false;
    }

    else {  // delete song
      if (full) {
        EEPROM.update((eepromaddress(16, number)), 0);
        ScreenBlink();
      } else confirmation = false;
    }

    PrintLoadSaveMenu(savemode);
  }
}

void BakeSequence() {  // normalize transpose & scale in to current seq / all seqs

  if ((modeselect == 1) || (modeselect == 2 && lockpattern)) {  // bake current Seq array
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

void CloneSequence(uint8_t source, uint8_t destination) {  // copy notes from a sequence to another

  for (uint8_t i = 0; i < maxSeqLength; i++) {
    noteSeq[destination][i] = noteSeq[source][i];
  }
}

void DebounceButtons() {  // debounce buttons

  static bool lastgreenReading = false;
  static bool lastyellowReading = false;
  static bool lastredReading = false;
  static bool lastblueReading = false;

  static unsigned long lastDebounceTime = 0;
  const uint8_t debounceDelay = 10;  // time in ms

  bool greenReading = !digitalRead(greenbutton);
  bool yellowReading = !digitalRead(yellowbutton);
  bool redReading = !digitalRead(redbutton);
  bool blueReading = !digitalRead(bluebutton);

  if ((greenReading != lastgreenReading) || (yellowReading != lastyellowReading) || (redReading != lastredReading) || (blueReading != lastblueReading)) {
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

  lastgreenReading = greenReading;
  lastyellowReading = yellowReading;
  lastredReading = redReading;
  lastblueReading = blueReading;
}

void ButtonsCommands(bool anypressed) {  // manage buttons's commands

  static bool redispressed = false;     // is redbutton still pressed?
  static bool yellowispressed = false;  // is yellowbutton still pressed?
  static bool greenispressed = false;   // is greenbutton still pressed?
  static bool blueispressed = false;    // is bluebutton still pressed?

  static uint8_t redtristate = 1;
  static uint8_t yellowtristate = 1;
  static uint8_t bluetristate = 1;

  bool newredstate = false;     // reset redstate snapshot
  bool newyellowstate = false;  // reset yellowstate snapshot
  bool newgreenstate = false;   // reset bluestate snapshot
  bool newbluestate = false;    // reset greenstate snapshot

  bool redandblue = false;
  bool redandyellow = false;
  bool yellowandgreen = false;
  bool greenandblue = false;

  if (redstate && bluestate) redandblue = true;
  if (redstate && yellowstate) redandyellow = true;
  if (yellowstate && greenstate) yellowandgreen = true;
  if (greenstate && bluestate) greenandblue = true;

  if (!redispressed) newredstate = redstate;           // take a redstate snapshot
  if (!yellowispressed) newyellowstate = yellowstate;  // take a yellowstate snapshot
  if (!blueispressed) newbluestate = bluestate;        // take a bluestate snapshot
  if (!greenispressed) newgreenstate = greenstate;     // take a greenstate snapshot

  StartScreenTimer = true;  // if any button is pressed or released, start the screen timer

  if (!redtristate) redtristate = 1;
  if (!yellowtristate) yellowtristate = 1;
  if (!greentristate) greentristate = 1;  // set greentristate to null
  if (!bluetristate) bluetristate = 1;

  if (menunumber == 0) {  // main screen - menu 0

    if (newgreenstate) {  // go to menu
      greentristate = 2;
      StartMenuTimer = true;
    }

    if (newredstate) redtristate = 2;
    if (newyellowstate) yellowtristate = 2;
    if (newbluestate) bluetristate = 2;

    if (redtristate == 2 && !newredstate) {
      redtristate = 0;
    }

    if (yellowtristate == 2 && !newyellowstate) {
      yellowtristate = 0;
    }

    if (greentristate == 2 && !newgreenstate) {
      greentristate = 0;
    }

    if (bluetristate == 2 && !newbluestate) {
      bluetristate = 0;
    }

    if (modeselect != 3) {  // not live mode

      if (blueispressed) {
        if (newgreenstate) SynthReset();  // reset synth
        else if (newredstate) {           // lock mute
          if (!recording) lockmute = !lockmute;
        }
      }

      if (greenispressed) {

        if (newredstate) {                    // select sequence/lock pattern
          if (modeselect == 1) PrintPopup();  // select sequence
          else if (modeselect == 2) {         // lock pattern
            lockpattern = !lockpattern;
            PrintPatternSequenceCursor();
          }
        }

        else if (newbluestate) {  // step-recording, go backwards
          if (!playing && internalClock && recording) {
            if (countStep > 0) {
              countStep -= 2;
              nopattern = true;
              HandleStep();
            }
          }
        }

        else if (newyellowstate) {  // go to start position or continue

          if (internalClock) {
            if (playing) {
              Startposition();
              if (sendrealtime) {
                MIDI.sendRealTime(midi::Start);
                start = true;
              }
            } else {
              if (sendrealtime) MIDI.sendRealTime(midi::Continue);
              playing = true;
            }
          }
        }
      }

      else if (!greenispressed) {

        if (!playing && internalClock && recording) {
          if (newgreenstate) HandleStep();                       // step-recording, go forwars
          if (newbluestate) noteSeq[currentSeq][countStep] = 0;  // step-recording, clear step
        }

        if (newyellowstate) {  // play/stop
          StartAndStop();
        }

        if (!blueispressed && newredstate) {  // start/stop recording

          if ((modeselect == 2 && !playing && !lockpattern) || (modeselect == 0)) {
            if (!recording) {
              PrintPopup();
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

      if (blueispressed || newbluestate) {  // toggle mute logic
        muted = lockmute ? !newbluestate : newbluestate;
        safedigitalWrite(blueLED, muted);
      }
    }

    else {  // live mode

      if (redstate && bluestate) {
        redandblue = true;
        redtristate = 1;
        bluetristate = 1;
      }

      if (redstate && yellowstate) {
        redandyellow = true;
        redtristate = 1;
        yellowtristate = 1;
      }

      if (yellowstate && greenstate) {
        yellowandgreen = true;
        yellowtristate = 1;
        greentristate = 1;
      }

      if (greenstate && bluestate) {
        greenandblue = true;
        greentristate = 1;
        bluetristate = 1;
      }

      if (!redtristate || !yellowtristate || !greentristate || !bluetristate || redandblue || redandyellow || yellowandgreen || greenandblue) {

        if (!redtristate) newcurrentSeq = 0;
        else if (!yellowtristate) newcurrentSeq = 1;
        else if (!bluetristate) newcurrentSeq = 2;
        else if (!greentristate) newcurrentSeq = 3;
        else if (redandyellow) newcurrentSeq = 4;
        else if (greenandblue) newcurrentSeq = 5;
        else if (redandblue) newcurrentSeq = 6;
        else if (yellowandgreen) newcurrentSeq = 7;

        if (!playing) {
          currentSeq = newcurrentSeq;
          StartAndStop();
        } else {
          TrigMute = true;
          if (muted) {
            currentSeq = newcurrentSeq;
          }
        }

        PrintPatternSequenceCursor();
      }
    }
  }

  else if (menunumber == 1) {  // main menu - menu 1

    if (newgreenstate) {  // go to submenu
      curpos = 0;
      SubmenuSettings(menuitem, 0);
    }

    else if (newbluestate) {  // go back to main screen
      PrintMainScreen();
    }

    else if (newyellowstate) {  // go up in the menu
      if (menuitem < 18) {
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

  else if (menunumber == 2) {  // submenu - menu 2

    if (newbluestate) PrintMenu(menuitem);  // go back to the menu

    else if (newgreenstate) {
      if (menuitem == 0) {
        // savecurX = 0;
        // savepage = 0;
        cloneCur = false;
        currentSeqSource = currentSeq;
        if (savemode > 0) oled.clear();
        PrintLoadSaveMenu(savemode);
        newgreenstate = false;
      }

      else if (menuitem == 1) {
        newcurrentSeq = currentSeq;
        modeselect = premodeselect;
        ScreenBlink();
        PrintMainScreen();
        if (lockmute) safedigitalWrite(blueLED, HIGH);
      }

      else if (menuitem == 4) TapTempo();
    }

    else if (newyellowstate) SubmenuSettings(menuitem, goDown);
    else if (newredstate) SubmenuSettings(menuitem, goUp);
  }

  else if (menunumber == 3) {  // load/save menu - menu 3

    if (!confirmation) {

      if (savemode == 1) {  // clone seq.

        if (newgreenstate) {
          if (!cloneCur) {
            cloneCur = true;
            newgreenstate = false;
          }
        }

        else if (newredstate) {
          if (cloneCur) {
            if (currentSeqDestination < (numberSequences - 1)) currentSeqDestination++;
          } else {
            if (currentSeqSource < (numberSequences - 1)) currentSeqSource++;
          }
        }

        else if (newyellowstate) {
          if (cloneCur) {
            if (currentSeqDestination > 0) currentSeqDestination--;
          } else {
            if (currentSeqSource > 0) currentSeqSource--;
          }
        }

      }

      else if (savemode == 2) {  // new song

        if (newyellowstate) {
          if (NewSeqLength > 1) {
            NewSeqLength--;
          }
        }

        else if (newredstate) {
          if (NewSeqLength < maxSeqLength) {
            NewSeqLength++;
          }
        }
      }

      else {  // save/load/delete

        if (newredstate) {
          if (savecurX > 0) {
            savecurX--;
          }
        }

        else if (newyellowstate) {
          if (savecurX < (memorySlots - 1)) {
            savecurX++;
          }
        }
        savepage = (savecurX / 6);
        full = (EEPROM.read(eepromaddress(16, savecurX)));
      }

      if (anypressed) PrintLoadSaveMenu(savemode);

      if (newgreenstate) {
        confirmation = true;
        drumModeSelect = false;
        PrintPopup();
        newgreenstate = false;
      }
    }

    if (confirmation && savemode == 2) {
      if (newredstate) drumModeSelect = false;
      else if (newyellowstate) drumModeSelect = true;
      PrintPopup();
    }

    if (newgreenstate) {
      if (confirmation) {
        LoadSave(savemode, savecurX);
        confirmation = false;
      }
    }

    else if (newbluestate) {
      if (!confirmation) {
        if ((savemode != 1) || (savemode == 1 && !cloneCur)) SubmenuSettings(menuitem, 0);
        else if (savemode == 1 && cloneCur) {
          cloneCur = false;
          PrintLoadSaveMenu(savemode);
        }
      } else {
        confirmation = false;
        if (savemode > 0) {
          PrintLoadSaveMenu(savemode);
        } else SubmenuSettings(menuitem, 0);
      }
    }

  }

  else if (menunumber == 4) {  // inter-recording popup - menu 4

    if (newbluestate) {  // exit popup
      if (!editorPopup) PrintMainScreen();
      else {  // if inside editor
        editorPopup = false;
        PrintBeatEditor();
      }
    }

    else if (newgreenstate) {  // assign seq. selection and exit popup

      currentSeq = newcurrentSeq;
      currentSeqEdit = newcurrentSeq;

      if (!editorPopup) {

        pattern = -1;  // in case selected seq isn't present in the pattern sequence
        for (uint8_t i = 0; i < patternLength; i++) {
          if ((currentSeq + 1) == songPattern[i]) {
            pattern = i;
            break;
          }
        }

        IntercountStep = -1;
        if (modeselect != 1) {
          recording = true;
          ManageRecording();
        }
        PrintMainScreen();
      } else {
        editorPopup = false;
        PrintBeatEditor();
      }
    }

    else if (newyellowstate) {  // seq. selection down
      if (newcurrentSeq > 0) newcurrentSeq--;
      PrintPopup();
    }

    else if (newredstate) {  // seq. selection up
      if (newcurrentSeq < (numberSequences - 1)) newcurrentSeq++;
      PrintPopup();
    }
  }

  else if (menunumber == 5) {  // beat editor - menu 5

    if (newgreenstate) {
      greentristate = 2;
    }

    if (!greenispressed) {
      if (newredstate) {
        if (colum < (seqLength - 1)) colum++;
      } else if (newyellowstate) {
        if (colum > 0) colum--;
      }
    }

    else {  // greenispressed

      if (newyellowstate) {
        if (row < 7) row++;
        greentristate = 0;
      }

      else if (newredstate) {
        if (!row && !colum) {
          editorPopup = true;
          PrintPopup();
        }
        if (row > 0) row--;
        greentristate = 0;
      }

      else if (newbluestate) {
        StartAndStop();
        greentristate = 0;
      }
    }

    if (greentristate == 2 && !newgreenstate) {
      greentristate = 0;
      bitToggle8(noteSeq[currentSeqEdit][colum], row);
    }

    window = colum / 8;
    frame = row / 3;

    if (!editorPopup) PrintBeatEditor();

    if (newbluestate && !greenispressed) {
      PrintMenu(menuitem);
    }
  }

  if (anypressed) {
    if (confirmsound) {  // bips
      confirmsound = false;
      Bip(2);
    } else Bip(1);  // do the buttons click if any button is pressed
  }

  if (!(menunumber == 2 && menuitem == 16)) {  // if not in CC mapping menu, turn the LED on if button is pressed

    if (!recording) {
      safedigitalWrite(redLED, redstate);
      safedigitalWrite(yellowLED, yellowstate);
    }

    safedigitalWrite(greenLED, greenstate);

    if (menunumber > 0 || (menunumber == 0 && !lockmute)) safedigitalWrite(blueLED, newbluestate);
  }

  greenispressed = greenstate;
  blueispressed = bluestate;
  redispressed = redstate;
  yellowispressed = yellowstate;
}
