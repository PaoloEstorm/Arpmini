/*!
 *  @file       Arpmini.ino
 *  Project     Estorm - Arpmini
 *  @brief      MIDI Sequencer & Arpeggiator
 *  @version    2.32
 *  @author     Paolo Estorm
 *  @date       2025/10/26
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
const char version[] PROGMEM = "V2.32";
#include "Vocabulary.h"
#include "Random8.h"
Random8 Random;
#define bitToggle8(value, bit) ((value) ^= (1 << (bit)))  // flip bit (8 bit version)
#define bitSet8(value, bit) ((value) |= (1 << (bit)))     // set bit high (8 bit version)
#include <avr/wdt.h>

// LEDs
#define yellowLED 2  // yellow LED pin
#define redLED 3     // red LED pin
#define greenLED 4   // green LED pin
#define blueLED 5    // blue LED pin

const uint8_t LEDs[4] = {
  redLED, yellowLED, blueLED, greenLED
};

// sound
uint8_t soundmode = 1;      // 1 audio on, 2 ui sounds off, 3 off
bool metro = false;         // metronome toggle
bool confirmsound = false;  // at button press, play the confirmation sound instead of the click
#include "Tone8.h"
Tone8 tone8;

// screen
#include "SPI_OLED.h"
SPI_OLED oled;                   // screen initialization
bool StartScreenTimer = true;    // activate the screen-on timer
bool alertnotification = false;  // keep track of notifications, timeout timer

// memory
#define memorySlots 60   // max number of songs (max 99) (absolute max is 110 but with graphical glitches)
#include "I2C_EEPROM.h"  // external 32kb eeprom
I2C_EEPROM EEPROM2;

// midi
#include "MIDIUSB.h"
#include <MIDI.h>
#include "SoftwareSerial_MiniRX.h"
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
uint8_t menunumber = 0;      // 0=mainscreen, 1=menu, 2=submenu, 3=file manager, 4=popups, 5=editor, 6=notifications
uint8_t savemode = 0;        // 0=bake, 1=clone, 2=new, 3=save, 4=load, 5=delete
uint8_t savecurX = 0;        // cursor position in file manager, 0-5
uint8_t savepage = 0;        // page number for external eeprom
bool confirmation = false;   // confirmation popup inside file manager
bool full = false;           // is the selected save slot full?
int8_t curpos = 0;           // cursor position in songmode, 0-7
bool StartMenuTimer = true;  // activate the go-to-menu timer
uint8_t Jittercur = 0;       // cursor position in the jitter submenu
bool cloneCur = false;       // cursor position in the clone seq submenu
bool editorPopup = false;    // normal popup or in editor popup?
uint8_t currentSeqEdit = 0;  // sequence that you are editing in editor
uint8_t colum;               // vertical position in editor
uint8_t row;                 // orizontal position in editor
uint8_t frame;               // vertical page/section in editor
uint8_t window;              // orizontal page/section in editor
uint8_t sidemenuitem = 1;    // selected item in the note editor's side-menu (0=transp. octave, 1=transp. note, 2=shift sequence)

// time keeping
bool playing = false;              // is the sequencer playing?
uint8_t clockTimeout;              // variable for keeping track of external/internal clock
bool internalClock = true;         // is the sequencer using the internal clock?
uint8_t StepSpeed = 3;             // 0=1/48, 1=1/32, 2=1/24, 3=1/16, 4=1/12, 5=1/8, 6=1/6, 7=1/4
bool FixSync = false;              // re-allign the sequence when changing step-speeds
uint8_t sendrealtime = 1;          // send midi realtime messages. 0=off, 1=on (@internalclock, only if playing), 2=on (@internalclock, always)
uint8_t tSignature = 4;            // time signature for LED indicator/metronome and beats, 1 - 8 (1*/4, 2*/4..to 8/4)
int8_t countBeat;                  // keep track of the time beats
int8_t countStep;                  // keep track of note steps in seq/song mode
int8_t IntercountStep;             // keep track of note steps while inter-recording
volatile bool stepEnable = false;  // keep track of internal timer ticks
int8_t countTicks;                 // clock ticks counter for notes timing
int8_t globalTicks;                // clock ticks counter for global timing & reference
uint8_t ticksPerStep = 6;          // how many clock ticks (countTicks) before the sequencer moves to the next step
uint8_t flip = 6;                  // part of the frameperstep's flipflop
uint8_t flop = 6;                  // part of the frameperstep's flipflop
bool flipflopEnable;               // switch for the frameperstep's flipflop
bool swing = false;                // is swing enabled?
bool snapmode = 0;                 // when play/stop the next sequence in live mode. 0=pattern, 1=beat
bool start = false;                // dirty fix for a ableton live 10 bug. becomes true once at sequence start and send a sync command
uint8_t BPM = 120;                 // beats per minute for internalclock - min 20, max 250 bpm
const uint8_t iterations = 8;      // how many BPM "samples" to averege out for the tap tempo. more = more accurate
uint8_t BPMbuffer[iterations];     // BPM "samples" buffer for tap tempo

// arpeggiator
const uint8_t holdNotes = 10;    // maximum number of notes that can be arpeggiated
uint8_t activeNotes[holdNotes];  // contains the MIDI notes the arpeggiator plays
bool ArpDirection;               // alternate up-down for arpstyle 2, 3, 4 & 5
uint8_t arpstyle = 0;            // 0=up, 1=down, 2=up-down, 3=down-up, 4=up+down, 5=down+up, 6=random
uint8_t arpcount;                // number of times the arpeggio repeats
uint8_t arprepeat = 1;           // number of times the arpeggios gets trasposed by "stepdistance"
int8_t stepdistance = 12;        // distance in semitones between each arp repeat, -12 to +12
int8_t numNotesHeld = 0;         // how many notes are currently pressed on the keyboard
uint8_t numActiveNotes = 0;      // number of notes currently stored in the holdNotes array
uint8_t trigMode = 0;            // 0=hold, 1=trigger, 2=retrigged
bool strum = false;              // strum mode enabled?

// sequencer
bool recording = false;                          // is the sequencer in the recording state?
bool keybTransp = true;                          // enable transpose by key? (only rec, song & live mode)
const uint8_t keybBaseNote = 48;                 // for transpose by key, the sequence is transposed based on the note C3 (midi note 48)
uint8_t keybTransposeNote = 48;                  // the last note number received for transposing the sequence
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
bool updatePatternCursor = false;                // update pattern cursor avoiding indirect recursive call

// notes
int8_t pitch = 0;                   // pitch transposition: -12 to +12
bool pitchmode = false;             // 0=before scale, 1=scale root
uint8_t scale = 0;                  // 0=linear, 1=penta.major, 2=penta.minor, 3=major, 4=minor, 5=harmonic minor, 6=locrian, 7=lydian, 8=dorian, 9=phrygian, 10=inverted, 11=hexatonal
int8_t posttranspose = 0;           // post-scale pitch transposition: -12 to +12
uint8_t noteLengthSelect = 3;       // set the notelength, 0=random, 1=10%, 2=30%, 3=50%, 4=70%, 5=90%, 6=110%
bool longRandomLength = true;       // allow the longest (110%) length when random notelength is selected
bool sortnotes = true;              // sort the ActiveNotes array?
bool muted = false;                 // suspend playing any notes from the sequencer (global)
bool lockmute = false;              // keep muted. only in arp, seq & song mode
bool suspended = false;             // temporarily suspend playing any notes from the sequencer (only rec & live mode)
bool scheduleSuspend = false;       // toggle the "suspended" state in sync according to the snapmode, in live mode
bool enableSustain = true;          // enable sustain pedal functionalities
bool sustain = false;               // hold notes also if trigmode > 0
const uint8_t queueLength = 8;      // the number of notes that can overlap if notelength is 120%
int8_t cronNote[queueLength];       // stores the notes playing
int8_t cronLength[queueLength];     // stores the amount of time a note has left to play
uint8_t jitrange = 0;               // jitter range 0-24
uint8_t jitprob = 0;                // jitter probability 0-10 (0-100%)
uint8_t jitmiss = 0;                // probability of a note to be not played (0-90%)
uint8_t shiftSeq[numberSequences];  // keep track of how much each sequence has been shifted

// drum mode
bool drumMode = false;            // is drum mode enabled?
bool drumModeSelect = false;      // pre-selection for drum mode
const uint8_t drumSlots = 8;      // number of drum slots available. must be 8 (8bit)
uint8_t DrumNotes[drumSlots];     // midi notes associated to each drum slot
bool ActiveDrumNotes[drumSlots];  // keep track of the currently pressed midi drum notes
bool DrumNotesMixer[drumSlots];   // enable/disable drum slot
bool drumroll = false;            // repeat note while key is pressed?
uint8_t drumKeybMode = 1;         // 0=free, 1=sync, 2=roll, 3=mixer

// general
uint8_t modeselect = 0;     // 0=arp.mode, 1=rec.mode, 2=song mode, 3=live mode
uint8_t premodeselect = 0;  // pre-selection of the mode in submenu

void BootAnimation() {  // play the boot animation

  oled.setSize(3);  // set font size to 3x

  for (uint8_t i = 0; i <= 22; i++) {  // scroll "arpmini"
    oled.setCursorXY(2, i);
    oled.println(F("ARPMINI"));
    delayMicroseconds(12500);
  }

  oled.setCursorXY(10, 0);
  oled.print(F(":STORM"));

  oled.setSize(2);
  oled.setCursorXY(34, 48);
  oled.printF(version);

  Bip(2);  // startup Sound
}

void setup() {  // initialization setup

  DDRD &= ~(1 << PD5);  // disable LED_BUILTIN_TX (on-board LED) (set as input)

  EEPROM2.init();  // EEPROM initialization

  // oled screen initialization
  oled.init();
  oled.clear();

  Serial.begin(115200);  // start serial USB at 115200 baud

  // initialize timer1 used for internal clock
  noInterrupts();
  TCCR1A = 0;               // set TCCR1A register to 0
  TCCR1B = 0;               // same for TCCR1B
  TCNT1 = 0;                // initialize counter value to 0
  OCR1A = 5208;             // initial tempo 120bpm
  TCCR1B |= (1 << WGM12);   // turn ON CTC mode
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

  tone8.init();  // speaker initialization

  // yellowbutton + bluebutton or not valid memory check, initialize the EEPROM with default values
  if ((!(PIND & (1 << PD7) && PINB & (1 << PB5))) || (EEPROM2.read(15) != 65)) ResetEEPROM();

  // load default settings from EEPROM
  midiChannel = EEPROM2.read(0);
  sendrealtime = EEPROM2.read(1);
  soundmode = EEPROM2.read(2);
  syncport = EEPROM2.read(3);

  for (uint8_t i = 0; i < 8; i++) {
    if (i < 4) buttonCC[i] = EEPROM2.read(4 + i);  // read from eeprom cc-mapping
    DrumNotes[i] = i + 48;                         // initialize the drum notes
    DrumNotesMixer[i] = true;                      // initialize the drum notes mixer
    BPMbuffer[i] = BPM;                            // initialize the tap tempo array
  }

  AllLedsOff();  // turn OFF all LEDs

  SetSyncPort(syncport);  // apply ports settings

  // initialize sequencer
  SetBPM(BPM);
  ClearSeqPatternArray();
  SynthReset();
  Startposition();

  BootAnimation();  // boot animation

  delay(2000);
  PrintMainScreen();  // go to main screen
}

void safedigitalWrite(uint8_t pin, bool state) {  // avoid digitalwrite and i2c at the same time (external eeprom)

  if (!EEPROM2.busy) digitalWrite(pin, state);
}

void SystemReset() {  // restart system

  asm volatile("clr r27");  // clear register 27 to avoid entering bootloader
  wdt_enable(WDTO_15MS);
  while (1) {}  // wait reset
}

ISR(TIMER1_COMPA_vect) {  // hardware Timer1 for internal clock

  stepEnable = true;  // internal clock, enable next "clock tick"
}

void SetBPM(uint8_t tempo) {  // change Timer1 speed to match BPM (20-250)

  OCR1A = ((250000UL * 5) - tempo) / (2 * tempo);
}

void MIDIUSBRead() {  // parse incoming MIDI data from USB

  if (MidiUSB.RX_available()) {  // if MIDI USB is available

    midiEventPacket_t rx;

    do {
      rx = MidiUSB.read();

      if (rx.header == 0x3) {  // song position
        if (rx.byte1 == 0xF2 && syncport == 3) HandleSongPosition((rx.byte3 << 7) | rx.byte2);
      }

      else if (rx.header == 0x8) HandleNoteOff((rx.byte1 & 0xF) + 1, rx.byte2, rx.byte3);  // note off

      else if (rx.header == 0x9) {  // note on

        if (rx.byte3 > 0) HandleNoteOn((rx.byte1 & 0xF) + 1, rx.byte2, rx.byte3);  // if velocity is more than 0, it's a note on
        else HandleNoteOff((rx.byte1 & 0xF) + 1, rx.byte2, rx.byte3);              // otherwise it's a note off
      }

      else if (rx.header == 0xB) HandleCC((rx.byte1 & 0xF) + 1, rx.byte2, rx.byte3);  // midi CC

      else if (rx.header == 0xF) {  // realtime messages
        if (syncport == 3) {
          if (rx.byte1 == 0xFA) HandleStart();               // start message
          else if (rx.byte1 == 0xFB) HandleContinue();       // continue message
          else if (rx.byte1 == 0xFC) HandleStop();           // stop message
          else if (rx.byte1 == 0xF8) HandleExternalClock();  // clock tick
        }
      }

    } while (rx.header != 0);
  }
}

void TCTask() {  // time critical task - MIDI

  MIDI.read();    // read incoming MIDI data from port 1
  MIDI2.read();   // read incoming MIDI data from port 2
  MIDIUSBRead();  // read incoming MIDI data from port 3 (USB)
}

void TCTask2() {  // time critical task 2 - clock

  if (stepEnable) {  // check if is time to run the internal clock
    stepEnable = false;
    HandleInternalClock();
  }
}

void PrintLiveObjects() {  // update main screen objects

  if (updatePatternCursor) {  // song or live mode, update cursor in main screen
    updatePatternCursor = false;
    PrintPatternSequenceCursor();
  }

  AlertTimeout();  // check if is time to close the notification
}

void loop() {  // run repeatedly

  TCTask();   // read incoming MIDI data
  TCTask2();  // check if is time to run the internal clock

  if (Serial.available()) {  // if incoming serial data from USB
    PcSync(Serial.read());   // read incoming serial data
  }

  DebounceButtons();   // debounce buttons
  ScreenOnTimer();     // check if is time to dim or turn OFF the display
  GoToMenuTimer();     // main screen, check if green button is pressed for long enough to enter the menu
  PrintLiveObjects();  // update main screen objects
}

void ResetEEPROM() {  // initialize the EEPROM with default values

  oled.printlnF(printcleaning);

  for (uint8_t i = 0; i < memorySlots; i++) {

    if (i < 4) {
      EEPROM2.update(i, 1);           // midiChannel, sendrealtime, soundmode, syncport
      EEPROM2.update(4 + i, i + 30);  // buttons cc
    }

    for (uint16_t j = 0; j < 288; j++) {
      EEPROM2.update(eepromaddress(32 + j, i), 0);  // clean songs slots
    }

    oled.printF(minus);                     // print progress bar
    if ((i + 1) % 20 == 0) oled.println();  // go to the next line after 20 "-"
  }

  EEPROM2.update(15, 65);  // set check byte
  tone8.tone(880, 250);
  oled.printF(printdone);
  delay(1000);
  oled.clear();
}

uint16_t eepromaddress(uint16_t address, uint8_t slot) {  // calculate eeprom song addresses

  const uint16_t slotsize = (numberSequences * maxSeqLength) + 16 + 16;
  return (slotsize * slot) + address;
}

void PcSync(uint8_t data) {  // exchange data with PC (Arpmini Editor)

  static uint8_t enable = 0;  // 0=none, 1=command recieved , 2=song address recieved
  static uint8_t mode = 0;
  static uint8_t slot = 0;
  bool stop = false;
  static uint16_t writeIndex = 32;
  const uint16_t SIZE_SLOT = 320 - 1;  // 32 bytes system + 288 song data -1

  // recieved commands
  const uint8_t CMD_CHECK_SLOT = 0xfc;
  const uint8_t CMD_READ_SLOT = 0xfd;
  const uint8_t CMD_WRITE_SLOT = 0xfe;
  const uint8_t CMD_STOP = 0xff;

  if (enable == 0) {  // select mode
    if (data >= CMD_CHECK_SLOT && data < CMD_STOP) {
      enable = 1;
      mode = data;
      return;
    } else stop = true;
  }

  else if (enable == 1) {  // select slot
    if (data > 0 && data <= memorySlots) {
      slot = data;
      enable = 2;
      return;
    } else stop = true;
  }

  else if (enable == 2) {  // check/read/write

    if (mode == CMD_CHECK_SLOT) {  // check slot
      Serial.write(EEPROM2.read(eepromaddress(32, slot - 1)));
      Serial.write(CMD_CHECK_SLOT);  // check confirmation
      enable = false;
    }

    else if (mode == CMD_READ_SLOT) {  // read entire slot
      for (uint16_t i = 32; i <= SIZE_SLOT; i++) {
        uint8_t _data = EEPROM2.read(eepromaddress(i, slot - 1));
        if (_data != 0xff) Serial.write(_data);
        else Serial.write((uint8_t)0x00);
      }
      Serial.write(CMD_STOP);  // read confirmation
      enable = false;
    }

    else if (mode == CMD_WRITE_SLOT) {  // write
      if (data != 0xff) {
        EEPROM2.update((eepromaddress(writeIndex, slot - 1)), data);
        Serial.write(CMD_WRITE_SLOT);  // write confirmation
        if (writeIndex < SIZE_SLOT) writeIndex++;
        else stop = true;
      } else stop = true;
    }
  }

  if (stop) {  // stop and reset
    stop = false;
    enable = false;
    mode = 0;
    slot = 0;
    writeIndex = 32;
    Serial.write(CMD_STOP);                            // stop confirmation
    if (menunumber == 3) PrintLoadSaveMenu(savemode);  // update screen
  }
}

void AllLedsOn() {  // turn ON all LEDs

  for (uint8_t i = 0; i <= 3; i++) {
    safedigitalWrite(LEDs[i], HIGH);
  }
}

void AllLedsOff() {  // turn OFF all LEDs

  for (uint8_t i = 0; i <= 3; i++) {
    safedigitalWrite(LEDs[i], LOW);
  }
}

void GoToMenuTimer() {  // greenbutton longpress, enter the menu

  static unsigned long SampleMenuTime;  // time in ms at which the timer was set
  static bool MenuTimerState;           // is the timer still running?

  if (StartMenuTimer) {  // start the timer
    StartMenuTimer = false;
    MenuTimerState = true;
    SampleMenuTime = millis();
  }

  if (MenuTimerState) {
    if (greentristate == 2) {                  // if green button is pressed
      if (millis() - SampleMenuTime > 1000) {  // if 1 second is passed
        MenuTimerState = false;
        greentristate = 1;
        if (recording) {  // disable recording
          recording = false;
          ManageRecording();
        }
        if (modeselect == 3) menuitem = 2;
        PrintMenu(menuitem);  // enter the menu
        Bip(2);               // confirmation sound
      }
    }
  }
}

void AlertTimeout() {  // automatically turn OFF notifications

  static unsigned long SampleMenuTime;  // time in ms at which the timer was set
  static bool MenuTimerState;           // is the timer still running?

  if (alertnotification) {  // start the timer
    alertnotification = false;
    MenuTimerState = true;
    SampleMenuTime = millis();
  }

  if (MenuTimerState) {
    if (millis() - SampleMenuTime > 1000) {
      MenuTimerState = false;
      if (menunumber == 0) PrintBPMBar();
    }
  }
}

void ScreenOnTimer() {  // automatically dim or turn OFF display

  static unsigned long SampleTime = 0;  // time at which the timer was set
  static bool TimerState = false;       // is the timer still running?

  if (StartScreenTimer) {  // start the timer
    StartScreenTimer = false;
    TimerState = true;
    SampleTime = millis();
    oled.sendCommand(OLED_DISPLAY_ON);
    oled.setContrast(255);
  }

  if (TimerState && millis() - SampleTime > 4000) {  // if the timer is running, check if it has expired
    TimerState = false;

    if (menunumber == 0 && !recording && !playing && internalClock) {  // check if the screen should be turned off or dimmed
      oled.sendCommand(OLED_DISPLAY_OFF);
    } else {
      oled.setContrast(1);
    }
  }
}

void SynthReset() {  // clear note caches

  for (uint8_t i = 0; i < queueLength; i++) {  // clear note queue
    cronNote[i] = -1;
    cronLength[i] = -1;
  }

  ClearArpBuffer();
  AllNotesOff();
}

void ClearArpBuffer() {  // clear arpeggiator buffer

  numActiveNotes = 0;
  for (uint8_t i = 0; i < holdNotes; i++) {  // clear the activeNotes buffer
    activeNotes[i] = 0;
  }
  countStep = -1;  // rewind step count
  ArpDirection = true;
  arpcount = 0;  // ??
}

void AllNotesOff() {  // send allnoteoff control change

  SendCC(123, 0, midiChannel);
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

void SetSyncPort(uint8_t port) {  // set port for external sync, 0: no port, 1: port1, 2: port2

  for (uint8_t i = 1; i <= 3; i++) {
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
  oled.invertText(true);
}

void PrintBottomText() {  // bottom small text printing setup

  oled.setSize(2);
  oled.setCursorXY(0, 48);
}

void Bip(uint8_t type) {  // bip sounds

  if (soundmode != 3) {                          // all sounds off
    if (soundmode == 1) {                        // include ui sounds
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

  if (tapInterval < 1500 && tapInterval > 240) {  // if the time between taps is not too long or too short (40-250bpm)

    BPMbuffer[bufferIndex] = 60200 / tapInterval;  // store iteration
    bufferIndex = (bufferIndex + 1) % iterations;  // go to the next BPMbuffer slot

    uint16_t BPMsum = 0;  // sum all iterations
    for (uint8_t i = 0; i < iterations; i++) {
      BPMsum += BPMbuffer[i];
    }

    if ((bufferIndex % 2) == 0) {     // every 2 taps
      BPM = BPMsum / iterations;      // calculate averege
      SetBPM(BPM);                    // apply new BPM
      SubmenuSettings(menuitem, -1);  // refresh submenu page
    }

  } else bufferIndex = 0;

  lastTapTime = currentTapTime;  // store last interval
}

void HandleInternalClock() {  // internal clock

  if (internalClock) {                                                                     // normal operation, run clock
    if ((sendrealtime == 1 && playing) || (sendrealtime == 2)) SendRealtime(midi::Clock);  // send midi clock
    RunClock();                                                                            // run clock
  }

  else {  // if external clock fails, stop the sequencer
    clockTimeout++;
    if (clockTimeout >= 50) {
      clockTimeout = 0;
      PrintAlertNotifi(10);  // clock lost!
      HandleStop();
    }
  }
}

void HandleExternalClock() {  // external clock

  if (!internalClock) {
    clockTimeout = 0;           // reset the clock timeout
    SendRealtime(midi::Clock);  // send midi clock
    RunClock();                 // run clock
  }
}

void RunClock() {  // main clock

  bool clockenable = !internalClock || playing;  // external clock, run the clock always. internal clock, run the clock only while playing

  if (modeselect < 2) {  // arp & rec mode, trigmodes logics

    if (trigMode == 0 || !enableSustain) sustain = false;  // if trigmode is set to "hold" disable sustain
    if (trigMode > 0) {                                    // if trigmode is set to "trig" or "retrig"
      if (!sustain && numNotesHeld == 0) {                 // if no key pressed & sustain pedal not engaged
        if (modeselect == 0) ClearArpBuffer();             // arp mode, reset buffer
        else suspended = !recording;                       // rec mode, suspend playing unless recording
      }
    } else suspended = false;                     // if trigmode is set to "hold" disable "suspended"
  } else if (modeselect != 3) suspended = false;  // "suspended" not compatible with song mode. disabled

  if (clockenable) {  // clock counters

    countTicks = (countTicks + 1) % ticksPerStep;  // main clock counter
    globalTicks = (globalTicks + 1) % 12;          // global clock counter. for leds blinking and sync
  }

  if (globalTicks == 0) {

    if (modeselect == 3 || clockenable) {

      countBeat = (countBeat + 1) % (tSignature * 2);

      if (playing && countBeat % 2 == 0) {

        if (FixSync) {
          FixSync = false;
          flipflopEnable = false;
          countTicks = 0;
        }

        if ((menunumber == 0 && modeselect != 3) || menunumber >= 4) safedigitalWrite(yellowLED, HIGH);  // turn ON yellowled, time indicator

        if (recording || metro) Metronome();  // play metronome
      }

      if (modeselect == 3) {  // live mode

        if ((snapmode == 0 && countStep == (seqLength - 1)) || (snapmode == 1 && countBeat % 2 == 0)) {  // snapmodes timings

          if (scheduleSuspend) {
            scheduleSuspend = false;
            suspended = !suspended;
          }

          if (currentSeq != newcurrentSeq) {
            currentSeq = newcurrentSeq;
            suspended = false;
            if (menunumber == 0) updatePatternCursor = true;  // update screen
          }
        }

        if (menunumber == 0) {  // live mode blinking

          if (countBeat % 2 == 0 || scheduleSuspend) {  // blink to the beat or double while transitioning
            if (playing) {
              if (!suspended || scheduleSuspend) {
                if (currentSeq < 4) safedigitalWrite((LEDs[currentSeq]), HIGH);  // turn ON a single LED for each sequence: 1=red, 2=yellow, 3=blue, 4=green

                else if (currentSeq == 4) {  // sequence 5, turn ON yellow & red
                  safedigitalWrite(yellowLED, HIGH);
                  safedigitalWrite(redLED, HIGH);
                } else if (currentSeq == 5) {  // sequence 6, turn ON blue & green
                  safedigitalWrite(blueLED, HIGH);
                  safedigitalWrite(greenLED, HIGH);
                } else if (currentSeq == 6) {  // sequence 7, turn ON blue & red
                  safedigitalWrite(blueLED, HIGH);
                  safedigitalWrite(redLED, HIGH);
                } else {  // sequence 8, turn ON yellow & green
                  safedigitalWrite(yellowLED, HIGH);
                  safedigitalWrite(greenLED, HIGH);
                }
              } else AllLedsOn();  // if no sequence playing, turn ON all LEDs
            }
          }
        }
      }
    }
  }

  else if (globalTicks == 2 && playing) {  // turn OFF LEDs every two globalTicks - tempo indicator

    if (menunumber >= 4) safedigitalWrite(yellowLED, LOW);  // while popups or inside editor, turn OFF yellowled
    else if (menunumber == 0) {                             // in main screen
      if (modeselect == 3) AllLedsOff();                    // live mode, turn OFF all LEDs
      else safedigitalWrite(yellowLED, LOW);                // other modes, turn OFF yellowled
    }
  }

  if (countTicks == 0) {  // trigger new step
    if (start) {
      start = false;
      HandleSongPosition(0);  // make ableton live happy, without would be slightly out of sync
    }
    if (modeselect == 3 || clockenable) HandleStep();  // trigger new step
  }

  for (uint8_t i = 0; i < queueLength; i++) {  // decrement queued notes

    if (cronNote[i] > -1) {  // if note is queued, subtract time remaining
      cronLength[i] = cronLength[i] - SetNoteLength();
    }

    if (cronLength[i] < -1) {  // if no time remaining, remove it
      SendNoteOff(cronNote[i], 0, midiChannel);
      cronLength[i] = -1;
      cronNote[i] = -1;
    }
  }
}

void SetTicksPerStep() {  // set ticksPerStep values

  static const uint8_t noSwingValues[] PROGMEM = { 2, 3, 4, 6, 8, 12, 16, 24 };
  static const uint8_t flipSwing[] PROGMEM = { 2, 4, 4, 8, 8, 16, 16, 32 };

  if (swing) {
    flip = pgm_read_byte(&flipSwing[StepSpeed]);  // read from PROGMEM
    flop = (StepSpeed % 2 == 0) ? flip : flip / 2;
  } else {
    flip = pgm_read_byte(&noSwingValues[StepSpeed]);  // read from PROGMEM
    flop = flip;
  }
}

void FlipFlopFPS() {  // alternates ticksPerStep values (for swing)

  flipflopEnable = !flipflopEnable;
  if (flipflopEnable) {
    if (!FixSync) SetTicksPerStep();
    ticksPerStep = flip;
  } else ticksPerStep = flop;
}

uint8_t SetNoteLength() {  // set the note duration

  static uint8_t noteLength = 30;
  static const uint8_t noteLengths[] PROGMEM = { 55, 35, 30, 25, 20, 15 };  // base noteLengths

  if (FixSync) return (noteLength);

  if (countTicks == 0) {
    if (noteLengthSelect == 0) {  // random
      noteLength = pgm_read_byte(&noteLengths[Random.get(0, longRandomLength + 4)]);
    } else noteLength = pgm_read_byte(&noteLengths[noteLengthSelect - 1]);
  } else return (noteLength);

  if (swing && !(StepSpeed % 2 == 0)) {  // adapt note length to swing
    if (flipflopEnable) noteLength = noteLength - 4;
    else noteLength = noteLength + 10;
  }

  if (StepSpeed == 0) noteLength = noteLength * 3;             // 1/48
  else if (StepSpeed == 1) noteLength = noteLength * 2;        // 1/32
  else if (StepSpeed == 2) noteLength = noteLength + 10;       // 1/24
  else if (StepSpeed == 3) return (noteLength);                // 1/16
  else if (StepSpeed == 4) noteLength = noteLength - 5;        // 1/12
  else if (StepSpeed == 5) noteLength = noteLength / 2;        // 1/8
  else if (StepSpeed == 6) noteLength = (noteLength / 3) + 1;  // 1/6
  else noteLength = noteLength / 4;                            // 1/4

  return (noteLength);
}

void HandleStep() {  // step sequencer

  FlipFlopFPS();

  if (modeselect == 0) {  // arp mode
    if (playing) {
      SetArpStyle(arpstyle);  // set arp sequence
      if (!muted && numActiveNotes > 0)
        QueueNote(activeNotes[countStep]);  // enqueue and play
    }
  }

  else {  // rec, song & live mode

    countStep = (countStep + 1) % seqLength;  // seq. sequencer

    if (/*!muted &&*/ playing && drumMode && (drumKeybMode == 1 || drumKeybMode == 2)) {

      for (uint8_t i = 0; i < drumSlots; i++) {  // play drums in sync
        if (ActiveDrumNotes[i]) {
          if (drumKeybMode == 1) ActiveDrumNotes[i] = false;
          if (recording) {
            if (drumKeybMode == 2) bitSet8(noteSeq[currentSeq][countStep], i);  // record roll
          } else QueueNote(DrumNotes[i]);
        }
      }
    }

    if (modeselect < 3) {  // rec & song mode
      if (recording) {
        if (playing) {
          if (bluestate) {
            noteSeq[currentSeq][countStep] = 0;  // if recording, bluebutton put a rest/clear step
            PrintAlertNotifi(7);                 // print clear step notification // could cause screen glitch...
          }
        }

        else if (internalClock) {  // if recording and not playing, yellowLED indicate steps
          if (countStep % 4 == 0) {
            safedigitalWrite(yellowLED, HIGH);
          } else safedigitalWrite(yellowLED, LOW);
        }
      }

      if (modeselect == 2) {  // call pattern sequencer
        if (countStep == 0) {
          if (!recording || (chainrec && !nopattern)) {
            HandlePattern(1);  // pattern go forwards
          } else nopattern = false;
        }
      }
    }

    if ((playing && !muted && !suspended) || (!playing && recording)) {
      uint8_t note = noteSeq[currentSeq][countStep];
      if (note > 0) {
        if (!drumMode) {
          if (note <= 127) {
            int8_t offset = recording ? 0 : (keybTransposeNote - keybBaseNote);
            QueueNote(note + offset);
          }
        } else {
          for (uint8_t i = 0; i < drumSlots; i++) {
            if (bitRead(note, i)) {
              if (DrumNotesMixer[i]) QueueNote(DrumNotes[i]);  // if instrument slot not disabled, play drum note
            }
          }
        }
      }
    }
  }
}

void HandlePattern(bool dir) {  // pattern sequencer in songmode

  if (!lockpattern) {
    int8_t step = dir ? 1 : -1;  // decide direction: +1 forward, -1 backward

    do {
      pattern = (pattern + step + patternLength) % patternLength;  // move to the next pattern, wrap around with modulo
    } while (songPattern[pattern] == 0);                           // skip empty patterns

    if (songPattern[pattern]) currentSeq = songPattern[pattern] - 1;  // if current pattern is active set the sequence
  }
  if (menunumber == 0) updatePatternCursor = true;  // update screen
}

void Metronome() {  // manage the metronome

  if (countBeat == 0) Bip(3);
  else Bip(4);
}

void SendRealtime(midi::MidiType type) {  // send MIDI realtime messages

  if (sendrealtime) {
    MIDI.sendRealTime(type);
    midiEventPacket_t data = { 0x0F, type, 0, 0 };
    MidiUSB.sendMIDI(data);
  }
}

void HandleStart() {  // start message - re-start the sequence

  SendRealtime(midi::Start);  // pass through start messages

  internalClock = false;
  playing = true;
  Startposition();
  UpdateScreenBPM();
}

void HandleContinue() {  // continue message - start the sequence

  SendRealtime(midi::Continue);  // pass through continue messages

  internalClock = false;
  globalTicks = 0;
  countTicks = 0;
  playing = true;
  UpdateScreenBPM();
}

void HandleStop() {  // stop the sequence and switch over to internal clock

  SendRealtime(midi::Stop);  // pass through stop messages

  if (recording) {
    recording = false;
    ManageRecording();
  }

  playing = false;
  internalClock = true;
  numNotesHeld = 0;
  UpdateScreenBPM();

  if (menunumber == 0) {
    if (modeselect != 3) safedigitalWrite(yellowLED, LOW);  // turn OFF yellowLED
    else AllLedsOff();
  }
}

void StartAndStop() {  // manage starts and stops

  playing = !playing;
  if (internalClock) {
    if (playing) {
      Startposition();
      SendRealtime(midi::Start);
      start = true;  // to make ableton live 10 happy
    } else SendRealtime(midi::Stop);
  } else {  // not internal clock
    AllNotesOff();
    if (recording) {
      recording = false;
      ManageRecording();
    }
  }
}

void HandleSongPosition(uint16_t position) {  // handle song position messages

  if (sendrealtime) {
    MIDI.sendSongPosition(position);  // pass through song position messages
    uint8_t low = position & 0x7F;
    uint8_t high = (position >> 7) & 0x7F;
    midiEventPacket_t SPosition = { 0x03, 0xF2, low, high };
    MidiUSB.sendMIDI(SPosition);
  }
}

void Startposition() {  // called every time the sequencing starts or stops

  SetTicksPerStep();

  if (!FixSync) {
    countTicks = -1;
    globalTicks = -1;
  }

  AllNotesOff();

  countBeat = -1;
  arpcount = 0;
  countStep = -1;
  flipflopEnable = false;
  ArpDirection = true;

  if (modeselect == 2) {
    if (recording) nopattern = true;
    else if (playing && !lockpattern) pattern = -1;
  }
}

void UpdateScreenBPM() {  // refresh BPM indicator

  StartScreenTimer = true;  // turn ON display and restart screen-on timer

  if (alertnotification) return;

  if (menunumber == 0) PrintBPMBar();                                        // refresh BPM bar
  else if (menunumber == 2 && menuitem == 4) SubmenuSettings(menuitem, -1);  // refresh submenu page
}

void SendCC(uint8_t cc, uint8_t value, uint8_t channel) {

  MIDI.sendControlChange(cc, value, channel);
  uint8_t status = 0xB0 | (channel - 1);
  midiEventPacket_t CCevent = { 0x0B, status, cc, value };
  MidiUSB.sendMIDI(CCevent);
}

void SendNoteOn(uint8_t note, uint8_t velocity, uint8_t channel) {

  MIDI.sendNoteOn(note, velocity, channel);
  uint8_t status = 0x90 | (channel - 1);
  midiEventPacket_t noteOn = { 0x09, status, note, velocity };
  MidiUSB.sendMIDI(noteOn);
  //tone8.tone(tone8.midiToFreq(note), 50);  // only for testing
}

void SendNoteOff(uint8_t note, uint8_t velocity, uint8_t channel) {

  MIDI.sendNoteOff(note, velocity, channel);
  uint8_t status = 0x80 | (channel - 1);
  midiEventPacket_t noteOff = { 0x08, status, note, velocity };
  MidiUSB.sendMIDI(noteOff);
}

void HandleCC(uint8_t channel, uint8_t cc, uint8_t value) {  // handle CC messages

  if (channel == midiChannel) {       // if same channel
    if ((cc != 64) && (cc != 123)) {  // if not sustain pedal or panic cc

      if (menunumber == 2 && menuitem == 17) {  // if inside cc mapping menu

        EnableButtons = true;  // just in case, re enable on-board buttons

        if (value > 0) {                   // if cc is entered and not released
          buttonCC[mapButtonSelect] = cc;  // store cc
          SubmenuSettings(menuitem, -1);   // refresh submenu page
          StartScreenTimer = true;         // just in case, lit up the display
          Bip(2);                          // confirm sound
        }
      }

      else {  // external control

        for (uint8_t i = 0; i < 4; i++) {           // check if incoming cc is mapped to a button
          if (cc == buttonCC[i]) {                  // if incoming cc is mapped to a button
            bool* state = nullptr;                  // reset button pointer
            if (i == 0) state = &redstate;          // point to red button state
            else if (i == 1) state = &yellowstate;  // point to yellow button state
            else if (i == 2) state = &bluestate;    // point to blue button state
            else if (i == 3) state = &greenstate;   // point to green button state

            if (state) {                                 // if pointer points to a button
              *state = value;                            // update button state (HIGH or LOW)
              numbuttonspressedCC += (*state ? 1 : -1);  // decrease or increase number buttons pressed count
              EnableButtons = !numbuttonspressedCC;      // while external control, temporarely disable on-board buttons
              ButtonsCommands(*state);                   // trigger button action
            }
            break;
          }
        }
      }
    }

    else if (cc == 64) {  // if sustain pedal cc

      if (modeselect < 2 && enableSustain && trigMode > 0) {
        sustain = value;                  // update sustain state
        PrintAlertNotifi(8);              // print sustain state notification
      } else SendCC(cc, value, channel);  // pass trough if sustain disabled
    }

  } else SendCC(cc, value, channel);  // pass trough if different channel
}

void HandleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {  // handle a note-on events

  if (channel == midiChannel) {  // if same channel

    numNotesHeld++;  // increase notes held count

    if (drumMode) {  // drum mode

      if (menunumber == 2 && menuitem == 6) {  // inside map drum notes menu
        DrumNotes[curpos] = note;              // store note
        SubmenuSettings(menuitem, -1);         // refresh submenu page
        StartScreenTimer = true;               // lit up display
        Bip(2);                                // confirmation sound
      }

      for (uint8_t i = 0; i < drumSlots; i++) {  // check if incoming note is in the drum notes range

        if (note == DrumNotes[i]) {  // if in range

          if (drumKeybMode != 3) {                                 // if not in mixer mode
            if (DrumNotesMixer[i]) {                               // if instrument slot is enabled
              ActiveDrumNotes[i] = true;                           // update array
            } else if (playing || recording) PrintAlertNotifi(3);  // if instrument slot is disabled, show alert notification
          }

          else {                                     // if in mixer mode
            DrumNotesMixer[i] = !DrumNotesMixer[i];  // toggle mixer slot
            if (menunumber == 5) PrintEditor();      // if in beat editor, update screen
            else PrintAlertNotifi(0);                // print mixer state notification
          }
        }
      }
    }

    if (!playing) {

      if (menunumber == 5 && colum) {  // if inside editor, store note in the sequence
        if (!drumMode) {
          noteSeq[currentSeqEdit][colum - 1] = note;  // store note
          PrintEditor();                              // update screen
        } else {
          for (uint8_t i = 0; i < drumSlots; i++) {               // check currently held drum notes
            if (note == DrumNotes[i]) {                           // if in range
              bitToggle8(noteSeq[currentSeqEdit][colum - 1], i);  // store note
              PrintEditor();                                      // update screen
            }
          }
        }
      }

      if (!recording) SendNoteOn(TransposeAndScale(note), velocity, channel);  // bypass notes

      else if (modeselect != 0) {  // recording & !playing & not arp mode

        if (countStep == -1) HandleStep();  // trigger the first step

        if (!drumMode) {
          noteSeq[currentSeq][countStep] = note;  // store the note
          QueueNote(TransposeAndScale(note));     // play the note
        }

        else {
          for (uint8_t i = 0; i < drumSlots; i++) {             // check currently held drum notes
            if (ActiveDrumNotes[i]) {                           // if note is held
              if (DrumNotesMixer[i]) {                          // if instrument slot not disabled
                QueueNote(DrumNotes[i]);                        // play note
                bitToggle8(noteSeq[currentSeq][countStep], i);  // store/remove note
                ActiveDrumNotes[i] = false;                     // remove note from the buffer
              }
            }
          }
        }
      }

    }

    else {  // if playing

      if (modeselect < 2 && numNotesHeld == 1) {

        suspended = false;  // disable suspended state if a key is pressed (has effect only in rec mode)
        if (modeselect == 0) ClearArpBuffer();
        if (trigMode == 2 && !recording) {  // trigmode 2 (retrig) restart the sequencer
          Startposition();
          if (internalClock) HandleSongPosition(0);
        }
      }

      if (modeselect == 0) {  // arp mode

        if (numActiveNotes < holdNotes) {  // store the note only if the activeNotes array is not full
          activeNotes[numActiveNotes] = note;
          numActiveNotes++;  // increase notes held in buffer count
          SortArray();       // sort the activeNotes array
        }

      }

      else {  // not arp mode & playing

        if (recording) {                                                                                        // if playing and recording
          uint8_t stepIndex = (countTicks + 2 > (ticksPerStep / 2)) ? (countStep + 1) % seqLength : countStep;  // storing destination

          if (!drumMode) {
            noteSeq[currentSeq][stepIndex] = note;        // store note
            if (stepIndex == countStep) QueueNote(note);  // play note
          }

          else {                                                        // if drum mode
            for (uint8_t i = 0; i < drumSlots; i++) {                   // check currently held drum notes
              if (ActiveDrumNotes[i]) {                                 // if note is held
                if (DrumNotesMixer[i]) {                                // if instrument slot not disabled
                  bitSet8(noteSeq[currentSeq][stepIndex], i);           // store note
                  if (drumKeybMode != 2) ActiveDrumNotes[i] = false;    // if not in drum-roll mode, remove note from the buffer
                  if (stepIndex == countStep) QueueNote(DrumNotes[i]);  // play note
                }
              }
            }
          }
        }

        else {  // if playing and not recording
          if (!drumMode) {
            if (keybTransp) {
              keybTransposeNote = note;  // set by-keyboard-transpose
              PrintAlertNotifi(9);       // print keyboard transposition state
            } else PrintAlertNotifi(4);  // print "keyboard disabled" notification
          }

          else if (drumKeybMode == 0) {                // drum mode, if free play keybmode, play note immediately
            for (uint8_t i = 0; i < drumSlots; i++) {  // check currently held drum notes
              if (ActiveDrumNotes[i]) {                // if note is held
                if (DrumNotesMixer[i]) {               // if instrument slot not disabled
                  QueueNote(DrumNotes[i]);             // play note
                }
              }
            }
          }
        }
      }
    }

  } else SendNoteOn(note, velocity, channel);  // bypass if different channel
}

void HandleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {  // handle a note-off events

  if (channel == midiChannel) {  // if same channel

    if (numNotesHeld > 0) numNotesHeld--;  // decrease notes held count

    if (drumMode) {
      for (uint8_t i = 0; i < drumSlots; i++) {  // check if incoming note is in the drum notes range
        if (note == DrumNotes[i]) {
          if (drumKeybMode != 3 && DrumNotesMixer[i]) ActiveDrumNotes[i] = false;  // update array if not in mixer mode
        }
      }
    }

    if (!playing) {
      if (!recording) SendNoteOff(TransposeAndScale(note), velocity, channel);  // pass trough note off messages
    }

    else {  // else if playing

      if (modeselect == 0 && (trigMode > 0 || strum) && !sustain) {  // arp mode and playing, remove note from buffer

        for (uint8_t i = 0; i < holdNotes; i++) {  // search the released note
          if (activeNotes[i] == note) {
            activeNotes[i] = 0;  // remove it from the buffer
            numActiveNotes--;
            ShiftZeros();  // sort the activeNotes array
          }
        }
      }
    }

  } else SendNoteOff(note, velocity, channel);  // bypass if different channel
}

void SortArray() {  // sort activeNotes[]

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

  for (uint8_t i = 0; i < holdNotes - 1; i++) {  // remove any duplicates
    if ((activeNotes[i] > 0) && (activeNotes[i] == activeNotes[i + 1])) {
      activeNotes[i + 1] = 0;
      numActiveNotes--;
    }
  }

  ShiftZeros();  // push all zeros to the end of activeNotes[]
}

void ShiftZeros() {  // push all zeros to the end of activeNotes[]

  for (uint8_t i = 0; i < holdNotes - 1; i++) {
    if (activeNotes[i] == 0) {
      int8_t temp = activeNotes[i + 1];
      activeNotes[i + 1] = activeNotes[i];
      activeNotes[i] = temp;
    }
  }
}

int16_t SetScale(int16_t note, uint8_t scale) {  // adapt note to fit in to a scale

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
    { 0, 0, -1, 0, 0, 0, -1, 0, 0, -1, 0, -1 },         // Scale 10: Phrygian
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

  uint8_t chance = Random.get(1, 10);
  if (chance <= missProb) return false;
  else return true;
}

uint8_t TransposeAndScale(int16_t note) {  // apply transposition and scale

  if (drumMode) return note;  // no transpositions or scales in drum mode

  if ((!pitchmode) || scale == 0) note = SetScale(note + pitch, scale);
  else note = SetScale(note - pitch, scale) + pitch;

  note = note + posttranspose;  // apply post-transposition

  // if note gets transposed out of range, raise or lower an octave
  if (note < 0) note += 12;
  if (note > 127) note -= 12;

  return note;
}

void QueueNote(int8_t note) {  // play notes

  const uint8_t defaultVelocity = 64;

  if (modeselect == 0) {  // only in arp mode, arp step transpose
    if (arpcount) note = note + ((arpcount - 1) * stepdistance);
  }

  note = note + Jitter(jitrange, jitprob);  // apply jitter
  note = TransposeAndScale(note);           // apply scale & transpositions

  if (modeselect == 0 && recording) {  // arp mode inter-recording
    IntercountStep++;
    noteSeq[currentSeq][IntercountStep] = note;
    if (IntercountStep == seqLength) {  // automatically stop recording an the end
      IntercountStep = 0;
      recording = false;
      ManageRecording();
    }
  }

  if (!ProbabilityMiss(jitmiss)) return;  // queue note or skip?

  for (uint8_t i = 0; i < queueLength; i++) {  // if note is already stored in the queue, remove it
    if (cronNote[i] == note && cronNote[i] >= 0) {
      SendNoteOff(cronNote[i], 0, midiChannel);
      cronNote[i] = -1;
      cronLength[i] = -1;
      break;
    }
  }

  for (uint8_t i = 0; i < queueLength; i++) {  // place note in a free slot and play it
    if (cronLength[i] < 0) {
      SendNoteOn(note, defaultVelocity, midiChannel);
      cronNote[i] = note;
      cronLength[i] = 100;
      break;
    }
  }
}

void QueueNoteEditor(uint8_t note) {  // play notes inside editor

  if (!playing && note <= 127 && note > 0) QueueNote(note);
}

void ManageRecording() {  // recording ending setup

  keybTransposeNote = keybBaseNote;

  safedigitalWrite(redLED, recording);  // turn ON/OFF redLED

  if (/*internalClock &&*/ !playing) {
    Startposition();
  }
}

void SetArpStyle(uint8_t style) {  // arpeggiator algorithms

  if (!numActiveNotes) return;

  if (style % 2 != 0) {  // if down-starting arp style, start from last note
    if (countStep == -1) countStep = numActiveNotes;
  }

  uint8_t arpup = (countStep + 1) % numActiveNotes;
  uint8_t arpdown = (numActiveNotes + (countStep - 1)) % numActiveNotes;
  int8_t activenotes = numActiveNotes - 1;
  static uint8_t countrandnotes = 0;

  if (countStep == -1) countrandnotes = 0;

  if (style == 6) {  // random

    countStep = Random.get(0, activenotes);
    countrandnotes++;
    arpcount = Random.get(1, arprepeat);
    if (countrandnotes > (numActiveNotes * arprepeat)) {
      countrandnotes = 0;
      if (strum && !sustain) ClearArpBuffer();
    }
  }

  else {  // others

    switch (style) {
      case 0:  // up
        countStep = arpup;
        if (countStep == 0) arpcount++;  // end of cycle
        break;

      case 1:  // down
        countStep = arpdown;
        if (countStep == activenotes) arpcount++;  // end of cycle
        break;

      case 2:  // up-down
        if (ArpDirection) {
          countStep = arpup;               // arp up
          if (countStep == 0) arpcount++;  // only one activenote logic
          else if (countStep == activenotes) ArpDirection = false;
        } else {  // arp down
          countStep = arpdown;
          if (countStep == 0) {
            ArpDirection = true;
            arpcount++;  // end of cycle
          }
        }
        break;

      case 3:  // down-up
        if (ArpDirection) {
          countStep = arpdown;                       // arp down
          if (countStep == activenotes) arpcount++;  // only one activenote logic
          else if (countStep == 0) ArpDirection = false;
        } else {  // arp up
          countStep = arpup;
          if (countStep == activenotes) {
            ArpDirection = true;
            arpcount++;  // end of cycle
          }
        }
        break;

      case 4:  // up+down
        if (ArpDirection) {
          countStep++;
          if (countStep > activenotes) {
            countStep = activenotes;  // repeat last note
            ArpDirection = false;
          }
        } else {
          countStep--;
          if (countStep < 0) {
            countStep = 0;  // repeat first note
            ArpDirection = true;
            arpcount++;  // end of cycle
          }
        }
        break;

      case 5:  // down+up
        if (ArpDirection) {
          countStep--;
          if (countStep < 0) {
            countStep = 0;  // repeat first note
            ArpDirection = false;
          }
        } else {
          countStep++;
          if (countStep > activenotes) {
            countStep = activenotes;  // repeat last note
            ArpDirection = true;
            arpcount++;  // end of cycle
          }
        }
        break;
    }
    if (arpcount > arprepeat) {
      arpcount = 1;
      if (strum && !sustain) ClearArpBuffer();
    } else if (!arpcount) arpcount = 1;
  }
}

void ClearSeqPatternArray() {  // clear all pattern and sequences arrays

  for (uint8_t i = 0; i < patternLength; i++) {  // clean SongPattern array
    if (i < 4) songPattern[i] = (i + 1);         // set default 1-2-3-4-1-2-3-4
    else songPattern[i] = (i - 3);
  }

  for (uint8_t j = 0; j < numberSequences; j++) {  // clean seq arrays
    for (uint8_t i = 0; i < maxSeqLength; i++) {
      noteSeq[j][i] = 0;
    }
    shiftSeq[j] = 0;  // clean shift array
  }
}

void PrintSeqSelectPopup() {  // print sequence selection popups - menu 4

  menunumber = 4;

  oled.setSize(2);
  if (!editorPopup || !drumMode) {
    oled.invertText(true);
    oled.drawRect(28, 32, 100, 40);
  } else oled.clear(28, 32, 100, 40);

  oled.setCursorXY(36, 32);
  oled.printF(seq);
  oled.print(newcurrentSeq + 1);

  oled.invertText(false);
}

void PrintFileManagerPopup() {  // print file manager popups

  PrintTitle();
  oled.setCursorXY(28, 32);

  if (savemode < 2) {  // bake, clone
    oled.setCursorXY(10, 32);
    oled.print(F(" PROCEED?"));
  }

  else if (savemode == 2) {  // new
    if (!drumModeSelect) oled.printF(printnext);
    else oled.printF(space);
    oled.printF(printnote);
    oled.setCursorXY(28, 48);
    if (drumModeSelect) oled.printF(printnext);
    else oled.printF(space);
    oled.printF(printbeat);
  }

  else if (savemode == 3) {  // save
    if (!full) oled.print(F(" SAVE?"));
    else {
      oled.setCursorXY(6, 32);
      oled.print(F(" REWRITE?"));
    }
  }

  else {  // load & delete
    if (!full) {
      oled.setCursorXY(22, 32);
      oled.print(F(" EMPTY!"));  // nothing to load or delete!
    } else {
      if (savemode == 4) oled.print(F(" LOAD?"));  // load
      else if (savemode == 5) {
        oled.setCursorXY(16, 32);
        oled.print(F(" DELETE?"));  // delete
      }
    }
  }

  oled.invertText(false);
}

void PrintAlertNotifi(uint8_t txt) {  // print notifications

  if (menunumber > 0) return;  // notifications are allowed only in the main screen

  alertnotification = true;
  StartScreenTimer = true;

  oled.drawRect(0, 48, 127, 63);
  PrintBottomText();
  oled.invertText(true);

  switch (txt) {

    case 0:  // mixer state
      for (uint8_t i = 0; i < drumSlots; i++) {
        oled.setCursorX((i * 16) + 2);
        char letter = 'A' + i;
        if (DrumNotesMixer[i]) oled.print(letter);  // if instrument slot not disabled
        else oled.printF(printx);                   // else, print "X"
      }
      break;

    case 1:  // recording while drumKeybMode=mixer not allowed!
      oled.setCursorX(8);
      oled.print(F("DIS.MIXER!"));
      break;

    case 2:  // recording while externalsync and not playing not allowed!
      oled.setCursorX(18);
      oled.print(F("NOT"));
      oled.shiftX();
      oled.print(F("PLAY!"));
      break;

    case 3:  // current drum slot/sequencer is disabled/muted!
      oled.setCursorX(34);
      oled.print(F("MUTED!"));
      break;

    case 4:  // keyboard disabled!
      oled.setCursorX(8);
      oled.print(F("ENAB.KEYB!"));
      break;

    case 5:  // step forward
    case 6:  // step backward
    case 7:  // step cleared
      if (seqLength < 9) oled.setCursorX(17);
      else if (countStep < 9) oled.setCursorX(11);
      else oled.setCursorX(5);

      if (txt != 7) {
        oled.printF(printstep);
        if (txt == 5) oled.printF(printnext);  // step forward
        else oled.printF(printback);           // step backward
      }

      oled.print(countStep + 1);
      oled.print(F("/"));
      oled.print(seqLength);

      if (txt == 7) oled.print(F(" REST"));  // step cleared
      break;

    case 8:  // sustain state
      oled.setCursorX(sustain ? 20 : 14);
      oled.printF(printpedal);
      oled.shiftX();
      oled.printPtr(offonalways, sustain);
      break;

    case 9:
      {  // keyboard transpose state
        int8_t transp = keybTransposeNote - keybBaseNote;
        int8_t abstransp = transp;
        if (transp < 0) abstransp = -abstransp;

        if (abstransp == 0) oled.setCursorX(18);
        else if (abstransp > 9) oled.setCursorX(5);
        else oled.setCursorX(11);

        oled.printF(printtransp);
        if (transp > 0) oled.printF(plus);
        oled.print(transp);
        break;
      }

    case 10:  // clock lost!
      oled.setCursorX(9);
      oled.print(F("CLOCK"));
      oled.shiftX();
      oled.print(F("LOST"));
      break;
  }

  oled.invertText(false);
}

void PrintBPMBar() {  // print BPM status bar to the screen

  oled.clear(0, 48, 127, 63);
  PrintBottomText();

  if (!internalClock || BPM > 99) {
    oled.setCursorX(4);
  } else oled.setCursorX(11);

  if (internalClock) oled.print(BPM);  // print the BPM
  else oled.printF(ext);

  oled.shiftX();
  oled.printF(printbpm);  // print "BPM"
  oled.shiftX();
  oled.print(tSignature);  // print time signature
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

    if (pattern >= 0) {  // if outside the range don't print cursor
      oled.clear(0, 32, 127, 40);
      oled.setCursorXY((pattern * 16) + 2, 32);

      if (!lockpattern) oled.printF(upcursor);
      else oled.printF(emptyshifticon);
    }

    else {  // If pattern is -1, it was likely not found in the song chain, so display info.
      oled.setCursorXY(25, 32);
      if (!lockpattern) oled.printF(printnext);
      else oled.printF(emptyshifticon);
      oled.printF(seq);
      oled.print(currentSeq + 1);
    }
  }

  else {  // live mode

    static const uint8_t cursorX[8] PROGMEM = { 18, 18, 114, 114, 42, 90, 66, 66 };
    static const uint8_t cursorY[8] PROGMEM = { 16, 32, 16, 32, 16, 16, 16, 32 };

    static uint8_t lastseq = 0;  // store the previous sequence number

    for (uint8_t i = 0; i < 2; i++) {                // clear previous cursor and set new position
      uint8_t x = pgm_read_byte(&cursorX[lastseq]);  // read from PROGMEM
      uint8_t y = pgm_read_byte(&cursorY[lastseq]);  // read from PROGMEM
      oled.setCursorXY(x, y);
      if (i) break;
      oled.printF(space);
      lastseq = currentSeq;
    }

    oled.printF(printback);
  }
}

void PrintMainScreen() {  // print main screen - menu 0

  menunumber = 0;

  oled.clear();

  // print titles
  oled.drawRect(0, 0, 127, 8);
  PrintTitle();
  if (modeselect < 2) oled.shiftX();
  oled.shiftX();
  oled.printPtr(modes, modeselect);
  oled.shiftX();
  if (drumMode) oled.printF(drumicon);
  else oled.printF(noteicon);
  oled.shiftX();
  oled.printlnF(printmode);
  oled.invertText(false);

  if (lockmute) safedigitalWrite(blueLED, HIGH);  // the sequencer is muted!

  if (modeselect == 0) {  // arp mode screen
    oled.setSize(3);

    uint8_t pos = 2;
    if (arpstyle == 0) pos = 48;
    else if (arpstyle == 1) pos = 28;
    else if (arpstyle == 6) pos = 12;
    oled.setCursorXY(pos, 20);

    oled.printF(arpmodes[arpstyle]);
    oled.setSize(2);
  }

  else if (modeselect == 1) {  // rec mode screen
    oled.setCursorXY(36, 16);
    oled.printF(seq);
    oled.print(currentSeq + 1);
    oled.setCursorXY(((seqLength > 9) ? 14 : 20), 32);
    oled.printF(length);
    oled.shiftX();
    oled.print(seqLength);
  }

  else if (modeselect == 2) {  // song mode screen
    PrintPatternSequence();
    PrintPatternSequenceCursor();
  }

  else {  // live mode screen
    oled.shiftX();
    oled.println(F("1 5 7 6 3"));
    oled.shiftX();
    oled.print(F("2   8   4"));
    PrintPatternSequenceCursor();
  }

  PrintBPMBar();
}

void PrintMenu(uint8_t item) {  // print main menu - menu 1

  menunumber = 1;

  oled.clear();
  oled.setSize(3);
  oled.printF(printnext);

  switch (item) {

    case 0:  // file
      oled.printF(file);
      break;

    case 1:  // mode select
      oled.printlnF(printmode);
      oled.printF(printselect);
      break;

    case 2:  // arp style / seq.select / edit chain / start-stop live

      if (modeselect == 0) {  // arp mode
        oled.printlnF(arp);
        oled.printF(printstyle);
      }

      else if (modeselect == 1) {  // rec mode
        oled.printlnF(seq);
        oled.printF(printselect);
      }

      else if (modeselect == 2) {  // song mode
        oled.printlnF(chain);
        oled.printF(printeditor);
      }

      else {  // live mode
        if (playing) oled.printlnF(printstop);
        else oled.printlnF(printstart);
        oled.printF(printlive);
      }
      break;

    case 3:  // editor
      if (drumMode) oled.printlnF(printbeat);
      else oled.printlnF(printnote);
      oled.printF(printeditor);
      break;

    case 4:  // tempo
      oled.printF(printtempo);
      break;

    case 5:  // pitch / mixer
      if (!drumMode) oled.printF(printpitch);
      else oled.printF(printmixer);
      break;

    case 6:  // scale / drum notes
      if (!drumMode) oled.printF(printscale);
      else oled.printF(printdrumnotes);
      break;

    case 7:  // random
      oled.printF(printrandom);
      break;

    case 8:  // note length
      oled.printlnF(printnote);
      oled.printF(length);
      break;

    case 9:  // seq.length / arp steps
      if (modeselect == 0) {
        oled.printlnF(arp);
        oled.printF(steps);
      } else {
        oled.printlnF(seq);
        oled.printF(length);
      }
      break;

    case 10:  // speed & swing
      oled.printF(speed);
      break;

    case 11:  // trig.mode / chain rec / snap time
      if (modeselect == 2) {
        oled.printlnF(chain);
        oled.printF(rec);
      } else if (modeselect == 3) {
        oled.printlnF(printsnap);
        oled.printF(printmode);
      } else {
        oled.printlnF(printtrig);
        oled.printF(printstyle);
      }
      break;

    case 12:  // keyboard control
      oled.printlnF(printkeyb);
      oled.printF(printmode);
      break;

    case 13:  // metronome
      oled.printF(printmetro);
      break;

    case 14:  // midi channel
      oled.printF(printmidi);
      break;

    case 15:  // send sync
      oled.printlnF(printsend);
      oled.printF(sync);
      break;

    case 16:  // get sync
      oled.printlnF(printget);
      oled.printF(sync);
      break;

    case 17:  // map buttons
      oled.printF(printmap);
      break;

    case 18:  // sound
      oled.printF(printsound);
      break;

    case 19:  // restart
      oled.printF(printreboot);
      break;
  }
}

void SubmenuSettings(uint8_t item, int8_t dir) {  // change & print settings in submenu - menu 2

  bool keyEnable = dir + 1;
  bool Direction = dir;

  menunumber = 2;

  oled.clear();
  oled.setSize(3);

  if (item <= 1) oled.printF(printnext);
  else if (item != 5 && item != 7 && item != 19) oled.printF(printback);

  switch (item) {

    case 0:  // file

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!Direction) {
          if (savemode < 5) savemode++;
        } else {
          if (savemode > 0) savemode--;
        }
      }

      //-----SCREEN COMMANDS----//
      if (modeselect == 0 && savemode < 2) savemode = 2;  // no bake or clone in arp mode
      oled.printlnF(savemodes[savemode]);
      if (savemode == 1 || (savemode == 0 && modeselect == 1) || (savemode < 2 && modeselect == 2 && lockpattern)) oled.printF(seq);
      else oled.printF(song);
      break;

    case 1:  // mode select

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!Direction) {
          if (premodeselect < 3) premodeselect++;
        } else {
          if (premodeselect > 0) premodeselect--;
        }
      }

      if (drumMode && premodeselect == 0) premodeselect = 1;  // arp mode not allowed in drum mode

      //-----SCREEN COMMANDS----//
      oled.printlnF(printmode);
      oled.printPtr(modes, premodeselect);
      break;

    case 2:  // arp style / seq.select / edit chain / start-stop live

      if (modeselect == 0) {  // arp style

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (Direction) {
              if (arpstyle > 0) arpstyle--;
            } else {
              if (arpstyle < 6) arpstyle++;
            }
          } else if (arpstyle < 6) sortnotes = Direction;
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(printstyle);
        oled.printF(arpmodes[arpstyle]);

        if (arpstyle < 6) {
          PrintBottomText();
          oled.printF(sortmodes[sortnotes]);
        }
      }

      else if (modeselect == 1) {  // seq.select

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!Direction) {
            if (currentSeq > 0) currentSeq--;
          } else {
            if (currentSeq < (numberSequences - 1)) currentSeq++;
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(seq);
        oled.print(currentSeq + 1);
      }

      else if (modeselect == 2) {  // edit chain

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {

          int8_t count = -1;

          for (uint8_t i = 0; i < patternLength; i++) {  // count number of active song-patterns (minus one)
            if (songPattern[i] > 0) count++;
          }

          if (!greenstate) {
            if (Direction) {
              if (curpos < 7) curpos++;
            } else {
              if (curpos > 0) curpos--;
            }
          } else {
            if (Direction) {
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

      else {  // start-stop live
        StartAndStop();
        suspended = playing;
        confirmsound = true;
        greentristate = 1;
        PrintMainScreen();
      }
      break;

    case 3:  // editor

      currentSeqEdit = currentSeq;
      row = 1;
      colum = 0;
      window = 0;
      frame = 0;
      PrintEditor();
      break;

    case 4:  // tempo

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {

        int8_t step = 1 ^ (greenstate << 2);  // if green button, step becomes 5. otherwise is 1
        if (!Direction) step = -step;
        BPM += step;

        BPM = constrain(BPM, 20, 250);
        SetBPM(BPM);
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(printbpm);
      oled.print(BPM);
      if (!internalClock) oled.printF(printcircle);  // BPM value is referred to the internal clock!
      break;

    case 5:  // pitch / mixer

      if (drumMode) {  // map drum notes

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (Direction) {
              if (curpos < 7) curpos++;
            } else {
              if (curpos > 0) curpos--;
            }
          } else DrumNotesMixer[curpos] = Direction;
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(printmixer);
        oled.setSize(2);
        for (uint8_t i = 0; i < drumSlots; i++) {
          oled.setCursorXY(i * 16, 30);
          char letter = 'A' + i;
          if (DrumNotesMixer[i]) oled.print(letter);  // if instrument slot not disabled, print slot letter
          else oled.printF(printx);                   // else, print "X"
        }
        oled.setCursorXY((curpos * 16), 48);
        oled.printF(upcursor);
        break;
      }

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (Direction) {
            if (pitch < 12) pitch++;
          } else {
            if (pitch > -12) pitch--;
          }
        } else pitchmode = Direction;
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
      oled.printF(pitchmodes[pitchmode]);
      oled.printF(printscale);
      break;

    case 6:  // scale / drum notes

      if (!drumMode) {  // scale select
        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (Direction) {
              if (scale < 12) scale++;
            } else {
              if (scale > 0) scale--;
            }
          } else {
            if (Direction) {
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
        oled.printF(printtransp);
        if (posttranspose > 0) oled.printF(plus);
        oled.print(posttranspose);

      }

      else {  // map drum notes
        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (Direction) {
              if (curpos < 7) curpos++;
            } else {
              if (curpos > 0) curpos--;
            }
          } else {
            if (Direction) {
              if (DrumNotes[curpos] < 127) DrumNotes[curpos]++;
            } else {
              if (DrumNotes[curpos] > 0) DrumNotes[curpos]--;
            }
          }
          QueueNoteEditor(DrumNotes[curpos]);
        }

        //-----SCREEN COMMANDS----//
        uint8_t DrumNote = DrumNotes[curpos];
        uint8_t Octave = DrumNote / 12;
        uint8_t normalizedTranspose = (DrumNote % 12 + 12) % 12;  // normalize the transpose value

        oled.printF(noteicon);
        oled.shiftX();
        oled.printF(noteNames[normalizedTranspose]);
        if (Octave) oled.print(Octave - 1);
        else oled.printF(minusoneicon);

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

    case 7:  // random

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (!Direction) {
            if (Jittercur < 2) Jittercur++;
          } else {
            if (Jittercur > 0) Jittercur--;
          }
        } else {
          if (Direction) {
            if (Jittercur == 0) {
              if (jitrange < 24) jitrange++;
            } else if (Jittercur == 1) {
              if (jitprob < 10) jitprob++;
            } else {
              if (jitmiss < 9) jitmiss++;
            }
          } else {
            if (Jittercur == 0) {
              if (jitrange > 0) jitrange--;
            } else if (Jittercur == 1) {
              if (jitprob > 0) jitprob--;
            } else {
              if (jitmiss > 0) jitmiss--;
            }
          }
        }
      }

      //-----SCREEN COMMANDS----//
      PrintTitle();
      oled.println(F("  RANDOM  "));
      oled.invertText(false);

      for (uint8_t i = 0; i < 3; i++) {

        if (i == Jittercur) oled.printF(printnext);
        else oled.printF(space);

        oled.printF(jitmodes[i]);

        if (i == 0) {
          oled.println(jitrange);
        } else if (i == 1) {
          oled.print(jitprob * 10);
          oled.printlnF(percent);
        } else {
          oled.print(jitmiss * 10);
          oled.printlnF(percent);
        }
      }
      break;

    case 8:  // note length

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (Direction) {
            if (noteLengthSelect < 6) noteLengthSelect++;
          } else {
            if (noteLengthSelect > 0) noteLengthSelect--;
          }
        } else if (noteLengthSelect == 0) longRandomLength = Direction;
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(length);
      if (noteLengthSelect == 0) {
        oled.printF(printrandom);
        PrintBottomText();
        oled.printF(maxrandomlength[longRandomLength]);
      } else {
        oled.print((noteLengthSelect * 20) - 10);
        oled.printF(percent);
      }
      break;

    case 9:  // seq.length / arp steps

      if (modeselect != 0) {

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (Direction) {
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
            if (Direction) {
              if (arprepeat < 5) arprepeat++;
            } else {
              if (arprepeat > 1) arprepeat--;
            }
          } else {
            if (Direction) {
              if (stepdistance < 12) stepdistance++;
            } else {
              if (stepdistance > -12) stepdistance--;
            }
          }
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(steps);
        oled.print(arprepeat);
        PrintBottomText();
        oled.print(F("DIST."));
        if (stepdistance > 0) oled.printF(plus);
        oled.print(stepdistance);
      }
      break;

    case 10:  // speed & swing

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (Direction) {
            if (StepSpeed > 0) StepSpeed--;
          } else {
            if (StepSpeed < 7) StepSpeed++;
          }
          if (playing) FixSync = true;
        } else swing = Direction;
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(speed);
      oled.print(F("1/"));
      oled.printF(speeds[StepSpeed]);

      PrintBottomText();
      oled.printF(printswing);
      oled.shiftX();
      oled.printPtr(offonalways, swing);
      break;

    case 11:  // trig.mode / chain rec / snap time

      if (modeselect < 2) {  // arp & rec mode

        //-----BUTTONS COMMANDS----//
        if (keyEnable) {
          if (!greenstate) {
            if (Direction) {
              if (trigMode > 0) trigMode--;
            } else {
              if (trigMode < 2) trigMode++;
            }
          } else if (trigMode > 0) enableSustain = Direction;
        }

        //-----SCREEN COMMANDS----//
        oled.printlnF(printtrig);
        oled.printF(trigmodes[trigMode]);

        if (trigMode > 0) {
          PrintBottomText();
          oled.printF(printpedal);
          oled.shiftX();
          oled.printPtr(offonalways, enableSustain);
        }
      }

      else if (modeselect == 2) {  // song mode

        //-----BUTTONS COMMANDS----//
        if (keyEnable) chainrec = Direction;

        //-----SCREEN COMMANDS----//
        oled.printlnF(chain);
        oled.printPtr(offonalways, chainrec);
      }

      else {  // live mode

        //-----BUTTONS COMMANDS----//
        if (keyEnable) snapmode = Direction;

        //-----SCREEN COMMANDS----//
        oled.printlnF(printsnap);
        oled.printF(snapmodes[snapmode]);
      }
      break;

    case 12:  // keyboard control

      oled.printlnF(printkeyb);

      if (modeselect > 0) {  // not arp mode
        if (!drumMode) {     // not drum mode
          //-----BUTTONS COMMANDS----//
          if (keyEnable) keybTransp = Direction;

          //-----SCREEN COMMANDS----//
          if (keybTransp) oled.printF(printtransp);
          else oled.printF(off);
        }

        else {  // drum mode
          //-----BUTTONS COMMANDS----//
          if (keyEnable) {
            if (!Direction) {
              if (drumKeybMode < 3) drumKeybMode++;
            } else {
              if (drumKeybMode > 0) drumKeybMode--;
            }
          }

          //-----SCREEN COMMANDS----//
          oled.printPtr(drumkeybmodes, drumKeybMode);
        }
      }

      else {  // arp mode
        //-----BUTTONS COMMANDS----//
        if (keyEnable) strum = Direction;

        //-----SCREEN COMMANDS----//
        if (strum) oled.printF(printstrum);
        else oled.printF(printfree);
      }
      break;

    case 13:  // metronome

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) metro = Direction;
        else {
          if (Direction) {
            if (tSignature < 8) tSignature++;
          } else {
            if (tSignature > 1) tSignature--;
          }
        }
      }

      //-----SCREEN COMMANDS----//
      oled.printlnF(printmetro);
      oled.printPtr(offonalways, metro);
      PrintBottomText();
      oled.print(F("SIGN."));
      oled.print(tSignature);
      oled.printF(fourth);
      break;

    case 14:  // midi channel

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          AllNotesOff();
          if (Direction) {
            if (midiChannel < 16) midiChannel++;
          } else {
            if (midiChannel > 1) midiChannel--;
          }
        } else {
          EEPROM2.update(0, midiChannel);
          ScreenBlink();
        }
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("CH"));
      oled.print(midiChannel);
      if (EEPROM2.read(0) == midiChannel) oled.printF(printcircle);
      break;

    case 15:  // send sync

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (Direction) {
            if (sendrealtime < 2) sendrealtime++;
          } else {
            if (sendrealtime > 0) sendrealtime--;
          }
        } else {
          EEPROM2.update(1, sendrealtime);
          ScreenBlink();
        }
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("SEND"));
      oled.printPtr(offonalways, sendrealtime);
      if (EEPROM2.read(1) == sendrealtime) oled.printF(printcircle);
      break;

    case 16:  // get sync

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (internalClock) {
          if (!greenstate) {
            if (Direction) {
              if (syncport < 3) syncport++;
            } else {
              if (syncport > 0) syncport--;
            }
          } else {
            EEPROM2.update(3, syncport);
            ScreenBlink();
          }
        }
        SetSyncPort(syncport);
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("PORT"));
      oled.printF(inports[syncport]);
      if (EEPROM2.read(3) == syncport) oled.printF(printcircle);
      break;

    case 17:  // map buttons

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (Direction) {
            if (mapButtonSelect > 0) mapButtonSelect--;
          } else {
            if (mapButtonSelect < 3) mapButtonSelect++;
          }
        } else {
          for (uint8_t i = 0; i < 4; i++) {
            EEPROM2.update(4 + i, buttonCC[i]);
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

    case 18:  // sound

      //-----BUTTONS COMMANDS----//
      if (keyEnable) {
        if (!greenstate) {
          if (Direction) {
            if (soundmode > 1) soundmode--;
          } else {
            if (soundmode < 3) soundmode++;
          }
        } else {
          EEPROM2.update(2, soundmode);
          ScreenBlink();
        }
      }

      //-----SCREEN COMMANDS----//
      oled.println(F("SOUND"));
      oled.printPtr(soundmodes, soundmode - 1);
      if (EEPROM2.read(2) == soundmode) oled.printF(printcircle);
      break;

    case 19:  // restart
      SystemReset();
      break;
  }
}

void PrintEditor() {  // print note/beat editor - menu 5

  menunumber = 5;

  oled.setCursorXY(0, 0);
  oled.setSize(2);

  if (lockmute) safedigitalWrite(blueLED, HIGH);  // the sequencer is muted!

  if (!row && !colum) oled.printF(printnext);  // seq. select
  else oled.printF(printcircle);

  oled.print(F("SEQ"));
  oled.print(currentSeqEdit + 1);

  oled.printF(space);

  for (uint8_t i = 0; i < 4; i++) {                     // title window view
    if (i >= (seqLength + 7) / 8) oled.printF(printx);  // inactive windows
    else if (i == window) oled.printF(fullbox);         // current window
    else oled.printF(emptybox);                         // other active windows
  }

  oled.println();  // go down

  if (drumMode) {  // print beat editor

    for (uint8_t i = (frame * 3); i < ((frame + 1) * 3); i++) {

      if (i > 7) {  // clear bottom empty space
        oled.clear(0, 54, 127, 63);
        break;
      }

      char letter = 'A' + i;
      if (!colum && row == (i + 1)) oled.invertText(true);  // print mixer's letters
      if (DrumNotesMixer[i]) oled.print(letter);            // if instrument slot not disabled, print slot letter
      else oled.printF(printx);                             // else, print "X"

      oled.invertText(true);
      oled.printF(verticallines);  // vertical lines

      for (uint8_t j = (window * 8); j < ((window + 1) * 8); j++) {  // print matrix
        if (j > (seqLength - 1)) {
          oled.clear(((seqLength - (window * 8)) + 2) * 12, 16, 127, 63);  // clear empty space
          break;
        }

        bool pos = (colum == (j + 1) && row == (i + 1));  // print beat pattern
        if ((bitRead(noteSeq[currentSeqEdit][j], i)) == 0) {
          if (pos) oled.printF(emptyselectedbox);
          else oled.printF(emptybox);
        } else {
          if (pos) oled.printF(fullselectedbox);
          else oled.printF(fullbox);
        }
      }
      oled.invertText(false);
      oled.println();  // go down
    }
  }

  else {  // print note editor

    for (uint8_t i = 0; i < 3; i++) {

      if (!colum && row == (i + 1)) oled.invertText(true);

      // print side menu
      if (i == 0) oled.printF(octaveicon);
      else if (i == 1) oled.printF(noteicon);
      else {
        if (shiftSeq[currentSeqEdit] == 0) oled.printF(fullshifticon);
        else oled.printF(emptyshifticon);
      }

      oled.invertText(false);

      // print side menu separator
      if (sidemenuitem == i) oled.printF(verticallinetri);  // vertical line with triangle
      else oled.printF(verticalline);                       // vertical line

      for (uint8_t j = (window * 8); j < ((window + 1) * 8); j++) {  // print notes

        if (j > (seqLength - 1)) {
          oled.clear(((seqLength - (window * 8)) + 2) * 12, 16, 127, 63);  // clear empty space
          break;
        }

        uint8_t currentnote = noteSeq[currentSeqEdit][j];
        uint8_t normalizedTranspose = ((currentnote) % 12 + 12) % 12;  // normalize the transpose value
        bool pos = currentnote > 0 && currentnote <= 127;

        if (colum == (j + 1)) oled.invertText(true);  // if current note

        if (i == 0) {  // print first row (note)
          if (pos) oled.printF(noteNames2[normalizedTranspose]);
          else oled.printF(space);
        } else if (i == 1) {  // print second row (none/sharp)
          if (pos) oled.printF(noteNames3[normalizedTranspose]);
          else oled.printF(minus);
        } else {  // print third row (octave)
          if (pos) {
            uint8_t octave = currentnote / 12;
            if (octave) oled.print(octave - 1);  // print octave number
            else oled.printF(minusoneicon);
          } else oled.printF(space);
        }

        oled.invertText(false);  // reset invert state
      }
      oled.println();  // go down
    }
  }
}

void PrintLoadSaveMenu(uint8_t mode) {  // print file manager - menu 3

  menunumber = 3;

  if (mode == 0) {  // bake song
    confirmation = true;
    PrintFileManagerPopup();
  }

  else {  // clone, new, save, load, delete

    PrintTitle();

    if (mode == 2) oled.setCursorXY(4, 0);  // new song title start position
    else oled.setCursorXY(0, 0);            // other modes title start position

    oled.printF(space);
    oled.printF(savemodes[mode]);  // print mode name

    if (mode != 5) oled.printF(space);  // if not delete, add a space

    if (mode == 1) {  // clone seq.

      oled.print(F("SEQ "));  // last bit of title
      oled.invertText(false);

      oled.clear(0, 32, 127, 40);  // clean popup-area

      oled.setCursorXY(23, 16);  // first row, source
      if (!cloneCur) oled.printF(printnext);
      else oled.printF(space);
      oled.printF(seq);
      oled.print(currentSeqSource + 1);

      oled.setCursorXY(58, 32);  // down arrow
      oled.printF(downarrow);

      oled.setCursorXY(23, 48);  // second row, destination
      if (cloneCur) oled.printF(printnext);
      else oled.printF(space);
      oled.printF(seq);
      oled.print(currentSeqDestination + 1);
    }

    else if (mode == 2) {  // new song

      oled.print(F("SONG "));  // last bit of title
      oled.invertText(false);

      oled.clear(25, 32, 100, 63);  // clean popup-area

      oled.setCursorXY(5, 16);
      oled.printF(seq);
      oled.printF(length);
      oled.setSize(3);
      if (NewSeqLength > 9) oled.setCursorXY(45, 32);
      else oled.setCursorXY(56, 32);
      oled.print(NewSeqLength);
    }

    else {  // save, load, delete song

      // last bit of the title
      oled.print(F("@>"));  // title separator
      char letter = 'A' + savepage;
      oled.print(letter);  // memory page
      oled.printF(space);
      oled.invertText(false);

      oled.clear(60, 32, 65, 42);  // clean popup-area

      for (uint8_t i = 0; i < 6; i++) {
        uint8_t loc = i + (6 * savepage);
        if (i < 3) oled.setCursorXY(0, (i + 1) * 16);
        else oled.setCursorXY(65, (i - 2) * 16);

        uint8_t drumsong = (EEPROM2.read(eepromaddress(32, loc)));

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

        if (loc == savecurX) oled.printF(printback);
        else oled.printF(space);
      }
    }
  }
}

void LoadSave(uint8_t mode, uint8_t slot) {  // (bake/clone/new/save/load/delete, slot number 0-59)

  if (mode == 0) {  // bake seq/song
    BakeSequence();
    ScreenBlink();
  }

  else if (mode == 1) {  // clone seq.
    if (currentSeqSource != currentSeqDestination) {
      CloneSequence(currentSeqSource, currentSeqDestination);
      if (!drumMode) shiftSeq[currentSeqDestination] = shiftSeq[currentSeqSource];
    }
    ScreenBlink();
  }

  else if (mode == 2) {  // new song

    playing = false;
    StepSpeed = 3;
    trigMode = 0;
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
  }

  else if (mode == 3) {  // save song

    uint8_t slot1 = (pitchmode << 4) | (swing << 0);
    EEPROM2.update((eepromaddress(32, slot)), (drumMode + 1));
    EEPROM2.update((eepromaddress(33, slot)), slot1);
    EEPROM2.update((eepromaddress(34, slot)), (pitch + 12));
    EEPROM2.update((eepromaddress(35, slot)), scale);
    EEPROM2.update((eepromaddress(36, slot)), (posttranspose + 12));
    EEPROM2.update((eepromaddress(37, slot)), seqLength);
    EEPROM2.update((eepromaddress(38, slot)), BPM);
    EEPROM2.update((eepromaddress(39, slot)), StepSpeed);

    for (uint8_t i = 0; i < numberSequences; i++) {
      EEPROM2.update((eepromaddress(i + 40, slot)), songPattern[i]);  // save SongPattern array
      for (uint8_t j = 0; j < maxSeqLength; j++) {
        EEPROM2.update((eepromaddress(j + 48 + (32 * i), slot)), noteSeq[i][j]);  // save all seq's array
      }
      if (drumMode) EEPROM2.update((eepromaddress(i + 304, slot)), DrumNotes[i]);  // save drumNotes array
      else EEPROM2.update((eepromaddress(i + 304, slot)), shiftSeq[i]);            // save shiftSeq array
    }

    ScreenBlink();
  }

  else if (mode == 4) {  // load song

    playing = false;
    lockpattern = false;
    trigMode = 0;
    pattern = 0;
    ClearSeqPatternArray();

    drumMode = (EEPROM2.read(eepromaddress(32, slot)) - 1);
    uint8_t slot1 = EEPROM2.read(eepromaddress(33, slot));
    pitchmode = (slot1 >> 4) & 1;
    swing = (slot1 >> 0) & 1;
    pitch = (EEPROM2.read(eepromaddress(34, slot)) - 12);
    scale = EEPROM2.read(eepromaddress(35, slot));
    posttranspose = (EEPROM2.read(eepromaddress(36, slot)) - 12);
    seqLength = EEPROM2.read(eepromaddress(37, slot));
    BPM = EEPROM2.read(eepromaddress(38, slot));
    StepSpeed = EEPROM2.read(eepromaddress(39, slot));

    for (uint8_t i = 0; i < numberSequences; i++) {
      songPattern[i] = EEPROM2.read(eepromaddress(i + 40, slot));  // load SongPattern array
      for (uint8_t j = 0; j < maxSeqLength; j++) {
        noteSeq[i][j] = EEPROM2.read(eepromaddress(j + 48 + (32 * i), slot));  // load all seq's array
      }
      if (drumMode) DrumNotes[i] = EEPROM2.read(eepromaddress(i + 304, slot));  // load drumNotes array
      else shiftSeq[i] = EEPROM2.read(eepromaddress(i + 304, slot));            // load shiftSeq array
    }

    SetBPM(BPM);
    modeselect = 2;
    premodeselect = modeselect;
    currentSeq = 0;
    ManageRecording();
    ScreenBlink();
  }

  else {  // delete song

    EEPROM2.update((eepromaddress(32, slot)), 0);
    ScreenBlink();
  }

  if (mode > 2) PrintLoadSaveMenu(savemode);  // if save/load/delete, refresh file manager screen
  else PrintMainScreen();                     // else, go back to main screen
}

void BakeSequence() {  // normalize transpose & scale in to current seq / all seqs

  if (modeselect == 1 || (modeselect == 2 && lockpattern)) {  // bake current seq array
    for (uint8_t i = 0; i < maxSeqLength; i++) {
      if (noteSeq[currentSeq][i] > 0) noteSeq[currentSeq][i] = TransposeAndScale(noteSeq[currentSeq][i]);
    }
  }

  else {
    for (uint8_t j = 0; j < numberSequences; j++) {  // bake all seq arrays
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

  const uint8_t DEBOUNCEDELAY = 10;  // time in ms
  static uint8_t lastReading = 0;
  static unsigned long lastDebounceTime = 0;

  uint8_t reading = 0;  // reset buttons' readings

  // store all four readings in one byte (uint8_t reading)
  if (!(PINB & (1 << PB4))) reading |= 0x01;  // greenbutton
  if (!(PIND & (1 << PD7))) reading |= 0x02;  // yellowbutton
  if (!(PINE & (1 << PE6))) reading |= 0x04;  // redbutton
  if (!(PINB & (1 << PB5))) reading |= 0x08;  // bluebutton

  unsigned long now = millis();

  if (reading != lastReading) {  // if current reading is not the same as the last reading (a button has been pressed or released)
    lastDebounceTime = now;      // store current time
    if (!EnableButtons) {        // if buttons are disabled because currently externally controlled by MIDI CC
      EnableButtons = true;      // temporarely disable external control
      numbuttonspressedCC = 0;   // reset active buttons controlled by CC count
    }
  }

  if ((now - lastDebounceTime) > DEBOUNCEDELAY && EnableButtons) {  // if at least 10 ms has passed
    if ((reading & 0x01) != greenstate) {                           // if button's state is changed
      greenstate = (reading & 0x01);                                // update green button flag
      ButtonsCommands(greenstate);                                  // trigger button's commands
    } else if (((reading >> 1) & 0x01) != yellowstate) {            // if button's state is changed
      yellowstate = (reading >> 1) & 0x01;                          // update yellow button flag
      ButtonsCommands(yellowstate);                                 // trigger button's commands
    } else if (((reading >> 2) & 0x01) != redstate) {               // if button's state is changed
      redstate = (reading >> 2) & 0x01;                             // update red button flag
      ButtonsCommands(redstate);                                    // trigger button's commands
    } else if (((reading >> 3) & 0x01) != bluestate) {              // if button's state is changed
      bluestate = (reading >> 3) & 0x01;                            // update blue button flag
      ButtonsCommands(bluestate);                                   // trigger button's commands
    }
  }

  lastReading = reading;  // store current buttons' readings
}

void ButtonsCommands(bool anypressed) {  // manage buttons' commands

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

  bool allowrec = ((playing || internalClock) && (!drumMode || drumKeybMode != 3));

  if (!redispressed) newredstate = redstate;           // take a redstate snapshot
  if (!yellowispressed) newyellowstate = yellowstate;  // take a yellowstate snapshot
  if (!blueispressed) newbluestate = bluestate;        // take a bluestate snapshot
  if (!greenispressed) newgreenstate = greenstate;     // take a greenstate snapshot

  if (!redtristate) redtristate = 1;        // set redtristate to null
  if (!yellowtristate) yellowtristate = 1;  // set yellowtristate to null
  if (!greentristate) greentristate = 1;    // set greentristate to null
  if (!bluetristate) bluetristate = 1;      // set bluetristate to null

  StartScreenTimer = true;  // if any button is pressed or released, start the screen timer

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

    if (bluetristate == 2 && !newbluestate) {
      bluetristate = 0;
    }

    if (greentristate == 2 && !newgreenstate) {
      greentristate = 0;
    }

    if (modeselect != 3) {  // not live mode

      if (blueispressed) {  // lock mute / midi panic

        if (newgreenstate) SynthReset();  // reset synth
        else if (newredstate) {           // lock mute
          if (!recording) lockmute = !lockmute;
        }
      }

      if (greenispressed) {  // if green is pressed

        if (newredstate) {  // rec or song mode, open seq. selection popup
          if (modeselect > 0 && !recording) PrintSeqSelectPopup();
        }

        else if (newbluestate) {  // step-recording, go backwards
          if (!playing && internalClock && recording && modeselect > 0) {

            if (countStep == 0) {
              countStep = seqLength - 2;
              if (chainrec && modeselect == 2) {  // step & pattern backwards
                HandlePattern(0);
              }
            } else countStep -= 2;

            nopattern = true;
            HandleStep();

            PrintAlertNotifi(6);  // print going backwards notification
          }

          if (modeselect == 2 && !recording) {  // lock pattern logic
            lockpattern = !lockpattern;
            PrintPatternSequenceCursor();
          }

        }

        else if (newyellowstate) {  // go to start position or continue

          if (internalClock) {
            if (playing) {
              Startposition();
              SendRealtime(midi::Start);
              start = true;
            } else {
              SendRealtime(midi::Continue);
              playing = true;
            }
          }
        }
      }

      else {  // if green is not pressed

        if (!playing && internalClock && recording && modeselect > 0) {  // step-recording, go forwards or clear step

          if (newgreenstate) {  // step-recording, go forwards
            if (countStep > 0) nopattern = false;
            HandleStep();
            PrintAlertNotifi(5);  // print going forwards notification
          }

          else if (newbluestate) {  // step-recording, clear step
            if (countStep > -1) {   // don't clear start position step
              noteSeq[currentSeq][countStep] = 0;
              PrintAlertNotifi(7);  // print clear step notification
            }
          }
        }

        if (newyellowstate) {  // play/stop
          StartAndStop();
        }

        if (!blueispressed && newredstate) {  // start/stop recording

          if (!lockmute) {  // recording while muted not allowed

            if (modeselect == 0) {  // arp mode, inter-recording
              if (!recording) {
                if (allowrec) PrintSeqSelectPopup();  // open seq. selection popup
                else PrintAlertNotifi(2);             // print not playing notification
              }

              else {
                recording = false;
                ManageRecording();
              }
            }

            else {  // rec & song modes
              if (allowrec) {
                recording = !recording;
                ManageRecording();
              } else {
                if (drumMode && drumKeybMode == 3) PrintAlertNotifi(1);  // print disable mixer notification
                else PrintAlertNotifi(2);                                // print not playing notification
              }
            }
          } else PrintAlertNotifi(3);  // print muted! notification
        }

        if (blueispressed || newbluestate) {  // toggle mute logic
          muted = lockmute ? !newbluestate : newbluestate;
          safedigitalWrite(blueLED, muted);
        }
      }
    }

    else {  // live mode

      bool redandblue = false;
      bool redandyellow = false;
      bool yellowandgreen = false;
      bool greenandblue = false;

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

      if (!redtristate || !yellowtristate || !greentristate || !bluetristate || redandblue || redandyellow || yellowandgreen || greenandblue) {  // if any key combo

        if (!redtristate) newcurrentSeq = 0;
        else if (!yellowtristate) newcurrentSeq = 1;
        else if (!bluetristate) newcurrentSeq = 2;
        else if (!greentristate) newcurrentSeq = 3;
        else if (redandyellow) newcurrentSeq = 4;
        else if (greenandblue) newcurrentSeq = 5;
        else if (redandblue) newcurrentSeq = 6;
        else newcurrentSeq = 7;

        if (!playing) {  // if not playing, start the sequencer from the sequence selected from the key combo
          currentSeq = newcurrentSeq;
          StartAndStop();
        }

        else {
          scheduleSuspend = true;  // resume/disable playing, when is the right time (snap mode)

          if (suspended) currentSeq = newcurrentSeq;  // if playing suspended, start the sequencer from the sequence selected from the key combo
        }

        PrintPatternSequenceCursor();
      }
    }
  }

  else if (menunumber == 1) {  // main menu - menu 1

    if (newgreenstate) {  // go to submenu
      curpos = 0;
      SubmenuSettings(menuitem, -1);  // refresh submenu page
    }

    else if (newbluestate) {  // go back to main screen
      PrintMainScreen();
    }

    else if (newyellowstate) {  // go down in the menu
      if (menuitem < 19) {
        menuitem++;
        PrintMenu(menuitem);
      }
    }

    else if (newredstate) {  // go up in the menu
      if (menuitem > 0) {
        menuitem--;
        PrintMenu(menuitem);
      }
    }
  }

  else if (menunumber == 2) {  // submenu - menu 2

    if (newbluestate) PrintMenu(menuitem);  // go back to the menu

    else if (newgreenstate) {  // do things

      if (menuitem == 0) {  // enter file manager
        cloneCur = false;
        currentSeqSource = currentSeq;
        if (savemode > 0) oled.clear();
        PrintLoadSaveMenu(savemode);
        newgreenstate = false;
      }

      else if (menuitem == 1) {  // select mode

        if (premodeselect != modeselect) {  // change mode
          newcurrentSeq = currentSeq;

          if (premodeselect == 3) {
            lockmute = muted = false;  // lockmute & muted disabled, not compatible with live mode
          }

          modeselect = premodeselect;
        }
        ScreenBlink();
        PrintMainScreen();
      }

      else if (menuitem == 4) TapTempo();  // trigger tap tempo
    }

    else if (newyellowstate) SubmenuSettings(menuitem, 0);  // down direction
    else if (newredstate) SubmenuSettings(menuitem, 1);     // up direction
  }

  else if (menunumber == 3) {  // file manager - menu 3

    if (!confirmation) {

      if (savemode == 1) {  // clone seq.

        if (newgreenstate && !cloneCur) {
          cloneCur = true;
          newgreenstate = false;
        } else if (newredstate || newyellowstate) {
          int8_t dir = 1;
          if (newyellowstate) dir = -dir;
          uint8_t val = cloneCur ? currentSeqDestination : currentSeqSource;

          val += dir;
          if (val < numberSequences) {
            if (cloneCur) currentSeqDestination = val;
            else currentSeqSource = val;
          }
        }
      }

      else if (savemode == 2) {  // new song, set sequence length

        if (newyellowstate) {  // decrease length
          if (NewSeqLength > 1) {
            NewSeqLength--;
          }
        }

        else if (newredstate) {  // increase length
          if (NewSeqLength < maxSeqLength) {
            NewSeqLength++;
          }
        }
      }

      else {  // save/load/delete, move inside file manager

        if (newredstate) {
          if (savecurX > 0) savecurX--;
        }

        else if (newyellowstate) {
          if (savecurX < (memorySlots - 1)) savecurX++;
        }

        savepage = (savecurX / 6);  // calculate page number
        full = (EEPROM2.read(eepromaddress(32, savecurX)));
      }

      if (newgreenstate) {  // press green to trigger confirmation popup
        newgreenstate = false;
        confirmation = true;
        drumModeSelect = false;
        PrintFileManagerPopup();
      } else if (anypressed) PrintLoadSaveMenu(savemode);  // any button pressed, update screen
    }

    if (confirmation) {  // inside popup

      if (savemode == 2) {                          // select song type
        if (newredstate) drumModeSelect = false;    // note mode
        if (newyellowstate) drumModeSelect = true;  // drum mode
        PrintFileManagerPopup();
      }

      if (newgreenstate) {  // press green to accept and proceed
        confirmation = false;
        if (savemode < 4 || full) LoadSave(savemode, savecurX);
        else PrintLoadSaveMenu(savemode);
      }

      else if (newbluestate) {  // press blue to close popup without proceeding
        confirmation = false;
        if (savemode > 0) PrintLoadSaveMenu(savemode);
        else SubmenuSettings(menuitem, -1);  // refresh submenu page
      }
    }

    else {  // if not inside popup

      if (newbluestate) {
        if (savemode != 1 || !cloneCur) {  // press blue to go back to the menu
          SubmenuSettings(menuitem, -1);   // refresh submenu page
        } else {
          cloneCur = false;
          PrintLoadSaveMenu(savemode);  // clone seq, press blue to reset cursor position
        }
      }
    }
  }

  else if (menunumber == 4) {  // recording popup - menu 4

    if (newbluestate) {  // exit popup, do nothing
      if (!editorPopup) PrintMainScreen();
      else {  // if inside editor
        editorPopup = false;
        PrintEditor();
      }
    }

    else if (newgreenstate) {  // set selected sequence and exit popup

      currentSeq = newcurrentSeq;
      currentSeqEdit = newcurrentSeq;

      if (!editorPopup) {  // if not inside editor

        if (modeselect == 0) {  // arp mode, jump directly in to recording
          IntercountStep = -1;  // reset count step for inter-recording (for arp mode)
          recording = true;
          ManageRecording();
        }

        else if (modeselect == 2) {  // song mode, move to the selected sequence

          pattern = -1;  // in case selected seq is not present in the pattern sequence

          for (uint8_t i = 0; i < patternLength; i++) {  // search the selected sequence in the pattern chain
            if ((currentSeq + 1) == songPattern[i]) {
              pattern = i;  // sequence found!
              break;
            }
          }
        }

        PrintMainScreen();  // exit popup
      }

      else {  // else if inside editor, close popup
        editorPopup = false;
        colum = 0;
        row = 1;
        PrintEditor();  // exit popup
      }
      confirmsound = true;  // play confirmation sound
    }

    else if (newyellowstate) {  // seq. selection down
      if (newcurrentSeq > 0) newcurrentSeq--;
      PrintSeqSelectPopup();  // refresh seq. selection popup
    }

    else if (newredstate) {  // seq. selection up
      if (newcurrentSeq < (numberSequences - 1)) newcurrentSeq++;
      PrintSeqSelectPopup();  // refresh seq. selection popup
    }
  }

  else {  // editor - menu 5

    static uint8_t temporarymode = 0;  // allows playing sequences inside the editor by temporarily disabling any conflicting settings

    if (newgreenstate) {
      greentristate = 2;
    }

    if (!greenispressed) {  // move horizontally
      if (newredstate || newyellowstate) {

        if (newredstate) colum = (colum < seqLength) ? colum + 1 : 1;
        else if (colum) colum--;

        if (colum) {  // if inside sequencer/matrix
          uint8_t note = noteSeq[currentSeqEdit][colum - 1];
          if (!drumMode) QueueNoteEditor(note);  // play note
          else {                                 // play beat
            for (uint8_t i = 0; i < drumSlots; i++) {
              if (bitRead(note, i)) {
                if (DrumNotesMixer[i]) QueueNoteEditor(DrumNotes[i]);  // if instrument slot not disabled, play drum note
              }
            }
          }
        }
      }
    }

    else {  // greenispressed

      if (newredstate || newyellowstate) {  // edit sequence
        if (!drumMode) {                    // note editor, transpose/enable/disable note & shift sequence
          if (colum) {                      // if inside sequence
            if (sidemenuitem != 2) {        // transpose/enable/disable note

              uint8_t* note = &noteSeq[currentSeqEdit][colum - 1];  // pointer to current note

              if (*note > 0 && *note <= 127) {               // if in active range
                int8_t step = (sidemenuitem == 0) ? 12 : 1;  // increment/decrement by note or octave?
                if (newyellowstate) step = -step;            // if yellowbutton, decrement note
                int16_t temp = *note + step;                 // increment / decrement note
                temp = constrain(temp, 1, 127);
                *note = (uint8_t)temp;
                QueueNoteEditor(*note);  // play note
              }

              else if (*note > 127) {    // if note disabled
                *note -= 127;            // enable note
                QueueNoteEditor(*note);  // play note
              }

              else {                     // if note not set
                *note = 60;              // set default value (C4)
                QueueNoteEditor(*note);  // play note
              }
            }

            else {  // shift sequence

              if (newredstate) {  // shift right

                uint8_t last = noteSeq[currentSeqEdit][seqLength - 1];                                                    // save last
                for (uint8_t i = seqLength - 1; i > 0; i--) noteSeq[currentSeqEdit][i] = noteSeq[currentSeqEdit][i - 1];  // shift right
                noteSeq[currentSeqEdit][0] = last;                                                                        // first note is the previous last note
                shiftSeq[currentSeqEdit] = (shiftSeq[currentSeqEdit] + (seqLength - 1)) % seqLength;
              }

              else {  // shift left

                uint8_t first = noteSeq[currentSeqEdit][0];                                                               // save first
                for (uint8_t i = 0; i < seqLength - 1; i++) noteSeq[currentSeqEdit][i] = noteSeq[currentSeqEdit][i + 1];  // shift left
                noteSeq[currentSeqEdit][seqLength - 1] = first;                                                           // last note is the previous first note
                shiftSeq[currentSeqEdit] = (shiftSeq[currentSeqEdit] + 1) % seqLength;
              }
            }
          }
        }

        if (drumMode || !colum) {  // move up & down

          uint8_t max = (drumMode) ? 8 : 3;  // if drumMode max 8 rows, else max 3 rows

          if (newyellowstate) {
            if (row < max) row++;  //  move up
          } else {
            if (row > 0) row--;  // move down;
          }
        }
        greentristate = 0;
      }

      else if (newbluestate) {  // green + blue play/stop

        StartAndStop();

        if (!temporarymode && playing) {  // if you restarted the sequencer inside the editor, switch to rec mode temporarely
          temporarymode = trigMode + 1;   // store trigMode setting
          trigMode = 0;                   // set trigMode to "hold"
          premodeselect = modeselect;     // store current mode
          modeselect = 1;                 // switch to rec mode
          currentSeq = currentSeqEdit;    // assign the sequence shown at editor startup as the active sequence
        }

        greentristate = 0;
      }
    }

    if (greentristate == 2 && !newgreenstate) {  // only green, enable/disable note/dot

      if (row) {         // if not in side-menu
        if (drumMode) {  // beat editor
          if (colum) {   // if inside matrix

            uint8_t* note = &noteSeq[currentSeqEdit][colum - 1];  // pointer to current note

            bitToggle8(*note, row - 1);     // set/remove dot
            if (bitRead(*note, row - 1)) {  // if dot set, play drum note
              if (DrumNotesMixer[row - 1]) QueueNoteEditor(DrumNotes[row - 1]);
            }
          } else DrumNotesMixer[row - 1] = !DrumNotesMixer[row - 1];  // enable/disable instrument slot
        }

        else {  // note editor

          if (colum) {  // enable/disable note inside sequence

            uint8_t* note = &noteSeq[currentSeqEdit][colum - 1];  // pointer to current note

            if (*note > 0 && *note <= 127) {  // if note is active
              *note += 127;                   // disable note
            } else if (*note > 127) {         // if note is disabled
              *note -= 127;                   // enable note
              QueueNoteEditor(*note);         // play note
            } else {                          // enable and create new note
              *note = 60;                     // set default value (C4)
              QueueNoteEditor(*note);         // play note
            }
          } else sidemenuitem = row - 1;  // select sidemenu item
        }
      }

      else if (!colum) {  // if side-menu, in seq selector
        editorPopup = true;
        PrintSeqSelectPopup();  // open seq. selection popup
      }

      greentristate = 0;
    }

    if (colum && !row) row = 1;  // exit sidemenu
    window = (colum - 1) / 8;    // calculate window number
    frame = (row - 1) / 3;       // calculate frame number

    if (!editorPopup) PrintEditor();  // refresh editor page

    if (newbluestate && !greenispressed) {  // if only blue is pressed, go back to the menu

      if (temporarymode) {             // before exiting, if necessary, restore previous settings
        trigMode = temporarymode - 1;  // restore trigMode setting
        temporarymode = 0;             // disable flag
        modeselect = premodeselect;    // restore previous mode
      }

      PrintMenu(menuitem);  // go back to the menu
    }
  }

  if (!(menunumber == 2 && menuitem == 17)) {  // turn ON button LED when pressed (except in CC mapping menu)

    if (!recording) {
      safedigitalWrite(redLED, redstate);
      safedigitalWrite(yellowLED, yellowstate);
    }

    safedigitalWrite(greenLED, greenstate);

    if (menunumber > 0 && menunumber < 4 || !lockmute) safedigitalWrite(blueLED, newbluestate);
  }

  if (anypressed) {  // play click or confirmation sound
    Bip(confirmsound + 1);
    confirmsound = false;
  }

  greenispressed = greenstate;
  blueispressed = bluestate;
  redispressed = redstate;
  yellowispressed = yellowstate;
}
