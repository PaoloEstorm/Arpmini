
// vocabulary
const char space[] PROGMEM = " ";
const char on[] PROGMEM = "ON";
const char off[] PROGMEM = "OFF";
const char seq[] PROGMEM = "SEQ.";
const char rec[] PROGMEM = "REC";
const char song[] PROGMEM = "SONG";
const char arp[] PROGMEM = "ARP";
const char printmode[] PROGMEM = "MODE";
const char printlive[] PROGMEM = "LIVE";
const char length[] PROGMEM = "LENGTH";
const char printselect[] PROGMEM = "SELECT";
const char editor[] PROGMEM = "EDIT";
const char sync[] PROGMEM = "SYNC";
const char speed[] PROGMEM = "SPEED";
const char printbpm[] PROGMEM = "BPM";
const char printswing[] PROGMEM = "SWING";
const char printmetro[] PROGMEM = "METRO";
const char file[] PROGMEM = "FILE";
const char printpitch[] PROGMEM = "PITCH";
const char printnote[] PROGMEM = "NOTE";
const char printscale[] PROGMEM = "SCALE";
const char printstyle[] PROGMEM = "STYLE";
const char steps[] PROGMEM = "STEPS";
const char ext[] PROGMEM = "EXT";
const char chain[] PROGMEM = "CHAIN";
const char printCC[] PROGMEM = "CC ";
const char fourth[] PROGMEM = "/4";
const char printnext[] PROGMEM = ">";
const char printback[] PROGMEM = "<";
const char upcursor[] PROGMEM = "^";
const char downcursor[] PROGMEM = "_";
const char plus[] PROGMEM = "+";
const char printx[] PROGMEM = "X";
const char percent[] PROGMEM = "%";
const char printstop[] PROGMEM = "STOP";
const char printstart[] PROGMEM = "START";
const char printtempo[] PROGMEM = "TEMPO";
const char printrandom[] PROGMEM = "RANDOM";
const char printtrig[] PROGMEM = "TRIG";
const char printsnap[] PROGMEM = "SNAP";
const char printmidi[] PROGMEM = "MIDI*\n\rCHANNEL";
const char printsend[] PROGMEM = "SEND*";
const char printget[] PROGMEM = "GET*";
const char printmap[] PROGMEM = "MAP*\n\rKEYS";
const char printsound[] PROGMEM = "SOUND*";
const char printreboot[] PROGMEM = "REBOOT";
const char always[] PROGMEM = "ALWAYS";
const char printkeyb[] PROGMEM = "KEYB";
const char printtransp[] PROGMEM = "TRANSP.";
const char printstrum[] PROGMEM = "STRUM";
const char fullbox[] PROGMEM = "[";
const char emptybox[] PROGMEM = "]";
const char fullselectedbox[] PROGMEM = "(";
const char emptyselectedbox[] PROGMEM = ")";
const char noteicon[] PROGMEM = "'";
const char drumicon[] PROGMEM = ",";

const char noteNames[][3] PROGMEM = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

const char scaleNames[][8] PROGMEM = {
  "LINEAR", "PENTMAJ", "PENTMIN", "MAJOR", "MINOR",
  "HARMMIN", "BLUES", "LOCRIAN", "LYDIAN", "DORIAN",
  "PHRYGIA", "INVERSE", "HEXATON"
};

const char savemodes[][7] PROGMEM = {
  "BAKE", "CLONE", "NEW", "SAVE", "LOAD", "DELETE"
};

const char arpmodes[][8] PROGMEM = {
  "UP", "DOWN", "UP-DOWN", "DOWN-UP", "UP+DOWN", "DOWN+UP", "RANDOM"
};

const char speeds[][5] PROGMEM = {
  "1/32", "1/24", "1/16", "1/12", "1/8", "1/6", "1/4"     
};

const char trigmodes[][7] PROGMEM = {
  "HOLD", "GATE", "RETRIG"
};

const char snapmodes[][8] PROGMEM = {
  "PATTERN", "UP-BEAT", "BEAT"
};

const char sortmodes[][10] PROGMEM = {
  "KEY-ORDER", "ORDERED"
};

const char drumkeybmodes[][6] PROGMEM = {
  "FREE", "SYNC", "ROLL", "MIXER"
};

const char* const modes[] = { arp, rec, song, printlive };

const char* const sendmodes[] = { off, on, always };
