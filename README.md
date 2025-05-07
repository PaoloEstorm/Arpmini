# Arpmini+

Arpmini+ is a compact MIDI device, offering advanced functionality as an arpeggiator, notes/drum sequencer and recorder, designed to control external MIDI instruments. By connecting multiple Arpmini+ devices in daisy chain, you can expand its capabilities and control, creating a larger and more versatile system for live performance or music production.

![IMG_6585](https://github.com/user-attachments/assets/60bd7ffc-d121-4a0a-b854-e3a48006ef9f)

Its features include:

    Intuitive interface with four backlit buttons.
    All settings can be managed through the menu, which is accessed by holding the green button for 1 second from the main screen.
    Two independent MIDI inputs protected by optocouplers.
    Two MIDI outputs, which share the same signal to control multiple MIDI devices simultaneously.
    The 4 buttons of Arpmini+ can be mapped and controlled by an external MIDI controller via MIDI CC, offering greater versatility and remote control.
    You can save up to 60 songs in memory (EXTERNAL EEPROM). In addition to songs, various global settings can also be saved, and are indicated by an asterisk in the menu.

At the core of Arpmini+ is an Arduino Pro Micro with an ATmega32u4 microcontroller.

Arpeggio Styles

    Up: Notes played in ascending order.
    Down: Notes played in descending order.
    Up-Down: Notes played in ascending order, then descending.
    Down-Up: Notes played in descending order, then ascending.
    Up+Down: Alternating ascending-descending sequence.
    Down+Up: Alternating descending-ascending sequence.
    Random: Notes played in a random order.

Musical Scales

    Linear: No variation between input note and output note
    PentMaj: Major pentatonic scale.
    PentMin: Minor pentatonic scale.
    Major: Major scale.
    Minor: Minor scale.
    Arabic: Arabic scale.
    Blues: Blues scale.
    Locrian: Locrian scale.
    Lydian: Lydian scale.
    Dorian: Dorian scale.
    Inverse: Inverted scale.
    Hexaton: Hexatonic scale.

Arpmini+ Modes

![All](https://github.com/user-attachments/assets/68c1cb12-e384-424e-9e43-bb253b8aad30)


    Arp Mode: Arpeggiates the played notes based on the selected arpeggio mode.
    Rec Mode: Records and plays back up to 4 patterns of variable length, with a maximum of 32 notes each.
    Song Mode: Allows the user to chain up to 8 combinations of 4 recorded patterns, in any desired order.
    Live Mode: Allows the playback of 4 different patterns, each assigned to one of the 4 buttons on Arpmini+. When a button is pressed, the associated pattern is played, and it will change when another button is pressed; otherwise, the pattern loops.
        
## Functionalities:

Load/save/edit up to 60 songs (4 traks 32 steps max), clone sequence, live/while arpeggiating & by step recording, 7 arp styles - oredered or key-press order, record, edit and play polyphonic drum tracks (with piano roll), tempo from 20 to 250 BPM with tap-tempo, pre/post, by-root and by-keyboard traspositions, 11 music scales, bake (normalize) transpositions and scale,  jitter (range, probability and miss-note probability), up to 4 arp steps with adjustable distance, 4 arpeggiator/sequencer speeds, 3 trigger mode, swing, metronome with customizable time signature, midi channel selection, in/out sync options, external control by MIDI CC, UI sound settings, 
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

