
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
const char printeditor[] PROGMEM = "EDITOR";
const char sync[] PROGMEM = "SYNC";
const char speed[] PROGMEM = "SPEED";
const char printbpm[] PROGMEM = "BPM";
const char printswing[] PROGMEM = "SWING";
const char printmetro[] PROGMEM = "METRO";
const char file[] PROGMEM = "FILE";
const char printpitch[] PROGMEM = "PITCH";
const char printnote[] PROGMEM = "NOTE ";
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
const char minus[] PROGMEM = "-";
const char printx[] PROGMEM = "X";
const char percent[] PROGMEM = "%";
const char printstop[] PROGMEM = "STOP";
const char printstart[] PROGMEM = "START";
const char printtempo[] PROGMEM = "TEMPO";
const char printrandom[] PROGMEM = "RANDOM";
const char printtrig[] PROGMEM = "TRIG";
const char printsnap[] PROGMEM = "SNAP";
const char printmidi[] PROGMEM = "MIDI*\nCHANNEL";
const char printsend[] PROGMEM = "SEND*";
const char printget[] PROGMEM = "GET*";
const char printmap[] PROGMEM = "MAP*\nKEYS";
const char printsound[] PROGMEM = "SOUND*";
const char printuioff[] PROGMEM = "UI-OFF";
const char printreboot[] PROGMEM = "REBOOT";
const char always[] PROGMEM = "ALWAYS";
const char printkeyb[] PROGMEM = "KEYB";
const char printtransp[] PROGMEM = "TRANSP.";
const char printstrum[] PROGMEM = "STRUM";
const char printmixer[] PROGMEM = "MIXER";
const char printbeat[] PROGMEM = "BEAT ";
const char printdrumnotes[] PROGMEM = "DRUM\nNOTES";
const char printstep[] PROGMEM = "STEP";
const char printcleaning[] PROGMEM = "CLEANING";
const char printdone[] PROGMEM = "DONE";
const char printfree[] PROGMEM = "FREE";
const char printroll[] PROGMEM = "ROLL";
const char printpedal[] PROGMEM = "PEDAL";
const char fullbox[] PROGMEM = "[";
const char emptybox[] PROGMEM = "]";
const char fullselectedbox[] PROGMEM = "(";
const char emptyselectedbox[] PROGMEM = ")";
const char noteicon[] PROGMEM = "'";
const char drumicon[] PROGMEM = ",";
const char octaveicon[] PROGMEM = "&";
const char emptyshifticon[] PROGMEM = "\"";
const char fullshifticon[] PROGMEM = "a";
const char verticalline[] PROGMEM = "@";
const char verticallines[] PROGMEM = ";";
const char verticallinetri[] PROGMEM = "`";
const char minusoneicon[] PROGMEM = "\\";
const char downarrow[] PROGMEM = "$";
const char printcircle[] PROGMEM = "*";

const char noteNames[][3] PROGMEM = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

const char noteNames2[][2] PROGMEM = {
  "C", "C", "D", "D", "E", "F", "F", "G", "G", "A", "A", "B"
};

const char noteNames3[][2] PROGMEM = {
  "-", "#", "-", "#", "-", "-", "#", "-", "#", "-", "#", "-"
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

const char speeds[][3] PROGMEM = {
  "48", "32", "24", "16", "12", "8", "6", "4"
};

const char trigmodes[][7] PROGMEM = {
  "HOLD", "GATE", "RETRIG"
};

const char snapmodes[][8] PROGMEM = {
  "SEQ-END", "ON-BEAT"
};

const char sortmodes[][10] PROGMEM = {
  "KEY-ORDER", "ORDERED"
};

const char maxrandomlength[][9] PROGMEM = {
  "MAX.90%", "MAX.110%"
};

const char pitchmodes[][6] PROGMEM = {
  "PRE-", "ROOT-"
};

const char inports[][4] PROGMEM = {
  "OFF", "1", "2", "USB"
};

const char jitmodes[][6] PROGMEM = {
  "RANG ", "PROB ", "MISS "
};

const char* const drumkeybmodes[] PROGMEM = { printfree, sync, printroll, printmixer };
const char* const modes[] PROGMEM = { arp, rec, song, printlive };
const char* const offonalways[] PROGMEM = { off, on, always };
const char* const soundmodes[] PROGMEM = { on, printuioff, off };
