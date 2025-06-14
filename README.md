# Arpmini+

Arpmini+ is a miniature MIDI loopstation, offering advanced functionality as an arpeggiator, notes/drum sequencer and recorder, designed to control external MIDI instruments.
At the core of Arpmini+ is an Arduino Pro Micro with an ATmega32u4 microcontroller.
You can connect multiple Arpminis together creating a larger and more complex system for live performance or music production.

## Give that dusty old synth in your closet a fresh new life! Unlock a ton of advanced features and unleash your creativity with Arpmini+

![IMG_7545](https://github.com/user-attachments/assets/0898df72-a166-4744-bc27-c8b8d48c6f3d)

Its features include:

    Simple and intuitive UI.
    Packed with settings for total customization.
    Two independent MIDI inputs protected by optocouplers.
    Two MIDI outputs, which share the same signal to control multiple MIDI devices simultaneously.
    Four backlit buttons, mappable and controllable via MIDI CC from an external MIDI controller, for enhanced versatility and remote control.
    32KB of memory to store up to 60 songs and various global settings, marked with an asterisk in the menu.
    Built-in speaker for acoustic feedback and metronome.

Arpeggio Styles

    Up: Notes played in ascending order
    Down: Notes played in descending order
    Up-Down: Notes played in ascending order, then descending
    Down-Up: Notes played in descending order, then ascending
    Up+Down: Alternating ascending-descending sequence
    Down+Up: Alternating descending-ascending sequence
    Random: Notes played in a random order

Selectable Scales

    Major pentatonic scale
    Minor pentatonic scale
    Major scale
    Minor scale
    Harmonic Minor scale
    Blues scale
    Locrian scale
    Lydian scale
    Dorian scale
    Phrygian scale
    Inverted scale
    Hexatonic scale

Arpmini+ Modes

![Modes](https://github.com/user-attachments/assets/ca2bec88-29bf-4b8d-bb55-1927422d3a1b)

    Arp Mode: Arpeggiates the played notes based on the selected arpeggio mode.
    Rec Mode: Records and plays back up to 8 patterns of variable length, with a maximum of 32 notes each.
    Song Mode: Allows to chain up 8 recorded patterns, in any desired order.
    Live Mode: Allows the playback of one of the 8 different patterns, each assigned to one buttons/multi-button-combo.
        
## Functionalities:

Load/save/edit up to 60 songs (8 traks 32 steps max), clone sequence, live/while arpeggiating & by step recording, 7 arp styles - oredered or key-press order, record, edit and play polyphonic drum tracks (with piano roll), tempo from 20 to 250 BPM with tap-tempo, pre/post, by-root and by-keyboard traspositions, 12 music scales, bake (normalize) transpositions and scale,  random variations (range, probability and miss-note probability), up to 4 arp steps with adjustable distance, 4 arpeggiator/sequencer speeds, 3 trigger mode, swing, metronome with customizable time signature, midi channel selection, in/out sync options, external control by MIDI CC, UI sound settings, 
(I'm working on a detailed user manual)

![Wiring](https://github.com/user-attachments/assets/d4ef5a1f-47d1-4e6a-9500-db48eb0f6fc2)

A simulated demo of Arpmini+ can be explored on Wokwi via the link https://wokwi.com/projects/430279970010982401

## Schematic and PCB design: https://oshwlab.com/estorm/miniseq

Sparkfun Pro Micro: https://amzn.eu/d/if0r67j

Oled Display (blue): https://amzn.eu/d/c0RTBxJ

Oled Display (white): https://amzn.eu/d/1IbVAiL

Buttons: https://amzn.eu/d/7UyjFKx

Buzzer: https://amzn.eu/d/fEAc4UO

Pin Headers: https://amzn.eu/d/9up7Byu

