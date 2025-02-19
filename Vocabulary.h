#pragma once

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
const char printup[] PROGMEM = "UP";
const char printdown[] PROGMEM = "DOWN";
const char printrandom[] PROGMEM = "RANDOM";
const char length[] PROGMEM = "LENGTH";
const char select[] PROGMEM = "SELECT";
const char edit[] PROGMEM = "EDIT";
const char sync[] PROGMEM = "SYNC";
const char speed[] PROGMEM = "SPEED";
const char printbpm[] PROGMEM = "BPM";
const char printswing[] PROGMEM = "SWING";
const char printmetro[] PROGMEM = "METRO";
const char printsound[] PROGMEM = "SOUND";
const char file[] PROGMEM = "FILE";
const char printpitch[] PROGMEM = "PITCH";
const char printnote[] PROGMEM = "NOTE";
const char printscale[] PROGMEM = "SCALE";
const char printstyle[] PROGMEM = "STYLE";
const char steps[] PROGMEM = "STEPS";
const char ext[] PROGMEM = "EXT";
const char chain[] PROGMEM = "CHAIN";
const char printmap[] PROGMEM = "MAP";
const char printCC[] PROGMEM = "CC ";
const char half[] PROGMEM = "1/8";
const char third[] PROGMEM = "1/12";
const char normal[] PROGMEM = "1/16";
const char twice[] PROGMEM = "1/32";
const char fourth[] PROGMEM = "/4";
const char printnext[] PROGMEM = ">";
const char printback[] PROGMEM = "<";
const char upcursor[] PROGMEM = "^";
const char downcursor[] PROGMEM = "_";
const char plus[] PROGMEM = "+";
const char minus[] PROGMEM = "-";
const char printx[] PROGMEM = "X";
const char percent[] PROGMEM = "%";

const char noteNames[][3] PROGMEM = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

const char scaleNames[][8] PROGMEM = {
  "LINEAR", "PENTMAJ", "PENTMIN", "MAJOR", "MINOR",
  "ARABIC", "BLUES", "LOCRIAN", "LYDIAN", "DORIAN",
  "INVERSE", "HEXATON"
};
