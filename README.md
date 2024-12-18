# Arpmini+

Arpmini+ is a compact MIDI device, similar in size to a keychain, offering advanced functionality as an arpeggiator, sequencer, and note recorder, designed to control external MIDI instruments. By connecting multiple Arpmini+ devices in daisy chain, you can expand its capabilities and control, creating a larger and more versatile system for live performance or music production.

Its features include:

    Intuitive interface with four backlit buttons (red, yellow, green, blue), each assigned to a specific function for quick and easy control.
    All settings can be managed through the menu, which is accessed by holding the green button for 1 second from the main screen.
    Two independent MIDI inputs protected by optocouplers, ensuring reliability and electrical isolation.
    Two MIDI outputs, which share the same signal to control multiple MIDI devices simultaneously.
    Audio control through an integrated buzzer for interface sounds, confirmations, and metronome.
    The 4 buttons of Arpmini+ can be mapped and controlled by an external MIDI controller via MIDI CC, offering greater versatility and remote control.
    You can save up to 6 songs in memory (EEPROM), recalling and playing them at any time. In addition to songs, various global settings can also be saved, including:
    the MIDI channel, audio alert settings, MIDI input and output port settings and MIDI CC mapping for the buttons.

At the core of Arpmini+ is an Arduino Pro Micro with an ATmega32u4 microcontroller, which provides USB MIDI compatibility and flexibility for future expansions.

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
        
Additional Functions

Various other functions and settings are available through the menu. (I'm working to write a detailed user manual)

A simulated demo of Arpmini+ can be explored on Wokwi via the link https://wokwi.com/projects/415695537200967681

Schematic and PCB design: https://oshwlab.com/estorm/miniseq

Note: Arpmini+ uses a modified version of the GyverOLED library by @AlexGyver. Be sure to use the attached files for proper character rendering.
