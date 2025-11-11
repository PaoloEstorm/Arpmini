
#ifndef MIDIUSB_h
#define MIDIUSB_h

#include <stdint.h>
#include <Arduino.h>

#include <stdint.h>
#include "PluggableUSB.h"

#define EPTYPE_DESCRIPTOR_SIZE uint8_t
#define EP_TYPE_BULK_IN_MIDI EP_TYPE_BULK_IN
#define EP_TYPE_BULK_OUT_MIDI EP_TYPE_BULK_OUT
//#define MIDI_BUFFER_SIZE USB_EP_SIZE // 64
#define MIDI_BUFFER_SIZE 16

#define MIDI_AUDIO 0x01
#define MIDI_AUDIO_CONTROL 0x01
#define MIDI_CS_INTERFACE 0x24
#define MIDI_CS_ENDPOINT 0x25
#define MIDI_STREAMING 0x3
#define MIDI_JACK_EMD 0x01
#define MIDI_JACK_EXT 0X02

typedef struct
{
	uint8_t header;
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
} midiEventPacket_t;

_Pragma("pack(1)")
  /// Midi Audio Control Interface Descriptor
  typedef struct
{
	uint8_t len;    // 9
	uint8_t dtype;  // 4
	uint8_t dsubType;
	uint16_t bcdADc;
	uint16_t wTotalLength;
	uint8_t bInCollection;
	uint8_t interfaceNumbers;
} MIDI_ACInterfaceDescriptor;

typedef struct
{
	uint8_t len;    // 9
	uint8_t dtype;  // 4
	uint8_t dsubType;
	uint8_t jackType;
	uint8_t jackID;
	uint8_t jackStrIndex;
} MIDIJackinDescriptor;

typedef struct
{
	uint8_t len;    // 9
	uint8_t dtype;  // 4
	uint8_t dsubType;
	uint8_t jackType;
	uint8_t jackID;
	uint8_t nPins;
	uint8_t srcJackID;
	uint8_t srcPinID;
	uint8_t jackStrIndex;
} MIDIJackOutDescriptor;

/// MIDI Jack EndPoint Descriptor, common to midi in and out jacks.
typedef struct
{
	EndpointDescriptor len;  // 9
	uint8_t refresh;         // 4
	uint8_t sync;
} MIDI_EPDescriptor;

/// MIDI Jack  EndPoint AudioControl Descriptor, common to midi in and out ac jacks.
typedef struct
{
	uint8_t len;    // 5
	uint8_t dtype;  // 0x24
	uint8_t subtype;
	uint8_t embJacks;
	uint8_t jackID;
} MIDI_EP_ACDescriptor;

/// MIDI Audio Stream Descriptor Interface
typedef struct
{
	uint8_t len;    // 9
	uint8_t dtype;  // 4
	uint8_t dsubType;
	uint16_t bcdADc;
	uint16_t wTotalLength;
} MIDI_ASInterfaceDescriptor;

/// Top Level MIDI Descriptor used to create a Midi Interface instace \see MIDI_::getInterface()
typedef struct
{
	//	IAD
	IADDescriptor iad;
	// MIDI Audio Control Interface
	InterfaceDescriptor Audio_ControlInterface;
	MIDI_ACInterfaceDescriptor Audio_ControlInterface_SPC;

	// MIDI Audio Streaming Interface
	InterfaceDescriptor Audio_StreamInterface;
	MIDI_ASInterfaceDescriptor Audio_StreamInterface_SPC;

	MIDIJackinDescriptor MIDI_In_Jack_Emb;
	MIDIJackinDescriptor MIDI_In_Jack_Ext;
	MIDIJackOutDescriptor MIDI_Out_Jack_Emb;
	MIDIJackOutDescriptor MIDI_Out_Jack_Ext;

	MIDI_EPDescriptor MIDI_In_Jack_Endpoint;
	MIDI_EP_ACDescriptor MIDI_In_Jack_Endpoint_SPC;
	MIDI_EPDescriptor MIDI_Out_Jack_Endpoint;
	MIDI_EP_ACDescriptor MIDI_Out_Jack_Endpoint_SPC;
} MIDIDescriptor;

#define D_AC_INTERFACE(_streamingInterfaces, _MIDIInterface) \
	{ 9, MIDI_CS_INTERFACE, 0x1, 0x0100, 0x0009, _streamingInterfaces, (uint8_t)(_MIDIInterface) }

#define D_AS_INTERFACE \
	{ 0x7, MIDI_CS_INTERFACE, 0x01, 0x0100, 0x0041 }

#define D_MIDI_INJACK(jackProp, _jackID) \
	{ 0x06, MIDI_CS_INTERFACE, 0x02, jackProp, _jackID, 0 }

#define D_MIDI_OUTJACK(jackProp, _jackID, _nPins, _srcID, _srcPin) \
	{ 0x09, MIDI_CS_INTERFACE, 0x3, jackProp, _jackID, _nPins, _srcID, _srcPin, 0 }

#define D_MIDI_JACK_EP(_addr, _attr, _packetSize) \
	{ 9, 5, _addr, _attr, _packetSize, 0, 0, 0 }

#define D_MIDI_AC_JACK_EP(_nMIDI, _iDMIDI) \
	{ 5, MIDI_CS_ENDPOINT, 0x1, _nMIDI, _iDMIDI }

#define D_CDCCS(_subtype, _d0, _d1) \
	{ 5, 0x24, _subtype, _d0, _d1 }
#define D_CDCCS4(_subtype, _d0) \
	{ 4, 0x24, _subtype, _d0 }

#ifndef DOXYGEN_ARD
// the following would confuse doxygen documentation tool, so skip in that case for autodoc build
_Pragma("pack()")

#define WEAK __attribute__((weak))

#endif

  /**
 	 Concrete MIDI implementation of a PluggableUSBModule
 	 By default, will define one midi in and one midi out enpoints.
 */
  class MIDI_ : public PluggableUSBModule {

private:
	void accept(void);                 ///< Accepts a midi packet, \see MIDI_::read()
	EPTYPE_DESCRIPTOR_SIZE epType[2];  ///< Container that defines the two bulk MIDI IN/OUT endpoints types

protected:
	// Implementation of the PUSBListNode

	/// Creates a MIDIDescriptor midi interface and sollicit USBDevice to send control to it.
	///   \see USBDevice::SendControl()
	int getInterface(uint8_t* interfaceNum);
	/// Current implementation just returns 0
	int getDescriptor(USBSetup& setup);
	/// Optional interface usb setup callback, current implementation returns false
	bool setup(USBSetup& setup);
  /// MIDI Device short name, just returns 0
  uint8_t getShortName(char* name);

public:
	/// Creates a MIDI USB device with 2 endpoints
	MIDI_(void);
	/// Returns the number of bytes currently available from RX
	uint8_t available(void);
	/// Reads a new MIDI message from USB
	midiEventPacket_t read(void);
	/// Flushes TX midi channel
	void flush(void);
	/// Sends a MIDI message to USB
	void sendMIDI(midiEventPacket_t event);
	/// Sends a MIDI buffer of length size to USB
	size_t write(const uint8_t* buffer, size_t size);
	bool TX_available(void);
	bool RX_available(void);
};
extern MIDI_ MidiUSB;

#endif /* MIDIUSB_h */
