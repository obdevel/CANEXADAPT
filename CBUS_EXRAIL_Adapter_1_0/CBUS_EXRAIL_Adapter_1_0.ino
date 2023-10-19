
/*
  Copyright (C) Duncan Greenwood M5767 (duncan_greenwood@hotmail.com), 2023

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*

  EXADAPT - a CBUS module to interface with EXRAIL running on a DCC-EX command station

*/

/*

  Arduino libraries required (for Uno/Nano):

  Arduino CBUS libraries:
  -- CBUS
  -- CBUS2515
  -- CBUSLED
  -- CBUSswitch
  -- CBUSconfig

  3rd party libraries:
  -- ACAN2515   -- MCP2515 interface library
  -- Streaming  -- serial formatting

*/

/*

  NV map:
  1   -- how to consume messages
              - 0 = send all events to EXRAIL, using some filter list
              - 1 = only send previously taught events to EXRAIL, with or without a filter list

  EV map:
      -- not currently used

*/

// CBUS library headers
#include <CBUS2515.h>                   // CAN controller and CBUS class -- uses CBUS.h and ACAN2515.h
#include <CBUSswitch.h>                 // CBUS pushbutton switch class
#include <CBUSLED.h>                    // CBUS LED class
#include <CBUSconfig.h>                 // module configuration
#include <CBUSParams.h>                 // CBUS parameter class
#include <cbusdefs.h>                   // CBUS constants

// 3rd party library headers
#include <Streaming.h>                  // serial output formatting

// definitions
#define EXRAIL_SERIAL_PORT Serial       // the serial port to communicate with EXRAIL on the DCC-EX command station

// constants
const byte VER_MAJ = 1;                 // code major version
const char VER_MIN = 'a';               // code minor version
const byte VER_BETA = 0;                // code beta sub-version
const byte MODULE_ID = 132;             // CBUS module type

const byte LED_GRN_PIN = 4;             // CBUS green SLiM LED pin
const byte LED_YLW_PIN = 5;             // CBUS yellow FLiM LED pin
const byte CBUS_SWITCH_PIN = 6;         // CBUS push button switch pin
const byte CAN_INT_PIN = 2;             // MCP2515 interrupt pin
const byte CAN_CS_PIN  = 10;            // MCP2515 chip select pin
const byte NUM_RX_BUFS = 8;             // number of CAN receive buffers
const byte NUM_TX_BUFS = 2;             // number of CAN transmit buffers

const byte EXRAIL_BUFFER_SIZE = 32;     // size of EXRAIL serial input buffer

// CBUS objects
CBUSConfig module_config;               // module configuration object
CBUS2515 CBUS(&module_config);          // CBUS object
CBUSLED ledGrn, ledYlw;                 // two LED objects
CBUSSwitch cbus_switch;                 // CBUS switch object

// module name, must be 7 characters, space padded
unsigned char mname[7] = { 'E', 'X', 'A', 'D', 'A', 'P', 'T' };

// forward function declarations
void event_handler(byte, CANFrame *);
void frame_handler(CANFrame *);
bool is_CBUS_event(byte);
void send_message_to_EXRAIL(CANFrame *);
void read_data_from_EXRAIL(void);
void process_message_from_EXRAIL(const char *);
void process_serial_input(void);
void print_config(void);

//
/// setup CBUS - runs once at power-on from setup()
//

void setupCBUS(void) {

  // set config layout parameters
  module_config.EE_NVS_START = 10;
  module_config.EE_NUM_NVS = 5;
  module_config.EE_EVENTS_START = 20;
  module_config.EE_MAX_EVENTS = 128;
  module_config.EE_NUM_EVS = 2;
  module_config.EE_BYTES_PER_EVENT = (module_config.EE_NUM_EVS + 4);

  // 6 * 128 = 768 bytes

  // initialise and load module configuration
  module_config.setEEPROMtype(EEPROM_INTERNAL);
  module_config.begin();

  Serial << F("> mode = ") << ((module_config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID;
  Serial << F(", NN = ") << module_config.nodeNum << endl;

  // show code version and copyright notice
  print_config();

  // set module parameters
  CBUSParams params(module_config);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);

  // set CBUS LED pins and assign to CBUS
  ledGrn.setPin(LED_GRN_PIN);
  ledYlw.setPin(LED_YLW_PIN);
  CBUS.setLEDs(ledGrn, ledYlw);

  // initialise CBUS switch and assign to CBUS
  cbus_switch.setPin(CBUS_SWITCH_PIN, LOW);
  cbus_switch.run();
  CBUS.setSwitch(cbus_switch);

  // module reset - if switch is depressed at startup and module is in SLiM mode
  if (cbus_switch.isPressed() && !module_config.FLiM) {
    Serial << F("> switch was pressed at startup in SLiM mode") << endl;
    module_config.resetModule(ledGrn, ledYlw, cbus_switch);
    // module reboots
    /*NOTREACHED*/
  }

  // opportunity to set one-time module configuration items after module reset, e.g. events, NVs
  // reset will clear the event table and set all NVs to zero
  if (module_config.isResetFlagSet()) {
    module_config.writeNV(1, 0);                                    // use some other method to determine whuch events to forward to EXRAIL
    module_config.clearResetFlag();
  }

  // register our CAN frame handler, to receive every CAN frame
  CBUS.setFrameHandler(frame_handler);

  // register our CBUS event handler, to receive only previously taught events
  CBUS.setEventHandler(event_handler);

  // set CBUS LEDs to indicate the current mode
  CBUS.indicateMode(module_config.FLiM);

  // configure and start CAN bus and CBUS message processing
  CBUS.setPins(CAN_CS_PIN, CAN_INT_PIN);                            // MCP2515 chip-select and interrupt pins
  CBUS.setOscFreq(16000000UL);                                      // select the crystal frequency of the CAN module
  CBUS.setNumBuffers(NUM_RX_BUFS, NUM_TX_BUFS);                     // CAN send and receive buffers -- more buffers = more memory used

  if (!CBUS.begin()) {
    Serial << F("> error starting CBUS") << endl;
  } else {
    Serial << F("> CBUS started ok") << endl;
  }

  return;
}

//
/// setup - runs once at power on
//

void setup(void) {

  uint32_t t1 = 0;

  // main serial interface
  Serial.begin(115200);
  while (!Serial && millis() - t1 < 5000);
  Serial << endl << endl << F("> ** CBUS EXRAIL Adapter module 1 0 ** ") << __FILE__ << endl;

  // serial interface to EXRAIL, if separate
  // EXRAIL_SERIAL_PORT.begin(115200);

  // configure the CBUS library and CAN bus interface
  setupCBUS();

  // end of setup
  Serial << F("> ready") << endl << endl;

  return;
}

//
/// loop - runs forever
//

void loop(void) {

  //
  /// do CBUS message, switch and LED processing
  //

  CBUS.process();

  //
  /// check for messages from EXRAIL
  //

  read_data_from_EXRAIL();

  //
  /// process console commands
  //

  // process_serial_input();

  //
  /// check CAN message buffers
  //

  // if (CBUS.canp->receiveBufferPeakCount() > CBUS.canp->receiveBufferSize()) {
  // Serial << F("> receive buffer overflow") << endl;
  // }

  // if (CBUS.canp->transmitBufferPeakCount(0) > CBUS.canp->transmitBufferSize(0)) {
  // Serial << F("> transmit buffer overflow") << endl;
  // }

  //
  /// check CAN bus state
  //

  // byte s = CBUS.canp->errorFlagRegister();

  // if (s != 0) {
  // Serial << F("> error flag register is non-zero = ") << s << endl;
  // }

  //
  /// bottom of loop()
  //
}

//
/// user-defined frame processing callback function
/// called from the CBUS library for every CAN frame received
/// this comprises all CBUS messages
/// it receives a pointer to the received CAN frame
/// don't handle taught events here, use the event_handler function instead
//

void frame_handler(CANFrame *msg) {

  // print the formatted message
  Serial << F("> << ") << format_CAN_message(msg) << endl;

  if (is_CBUS_event(msg->data[0])) {                              // message opcode is a CBUS event
    if (module_config.readNV(1) == 0) {                           // send all CBUS messages if NV1 = 0
      send_message_to_EXRAIL(msg);                                // may apply its own filter
    }
  }

  return;
}

//
/// user-defined event processing callback function
/// called from the CBUS library whenever a previously taught event is received
/// it receives the event table index and a pointer to the incoming CAN frame
//

void event_handler(byte index, CANFrame *msg) {

  (void)index;                                                   // unused

  if (module_config.readNV(1) == 1) {                            // send only previously taught events if NV1 = 1
    send_message_to_EXRAIL(msg);                                 // may apply its own filter
  }

  return;
}

//
/// send a CBUS message
//

void send_cbus_message(CANFrame *msg) {

  Serial << F("> >> ") << format_CAN_message(msg) << endl;

  if (CBUS.sendMessage(msg)) {
    // Serial << F("> sent CBUS message ok") << endl;
  } else {
    Serial << F("> *** error sending CBUS message") << endl;
  }
}

//
/// return a CAN message as a formatted character string for display
//

char *format_CAN_message(const CANFrame *msg) {

  static char msgstr[48], tbuff[8];

  snprintf(msgstr, sizeof(msgstr), "[%03lx] [%1d] [ ", msg->id, msg->len);

  for (byte i = 0; i < msg->len && i < 8; i++) {
    snprintf(tbuff, sizeof(tbuff), "%02x ", msg->data[i]);
    strlcat(msgstr, tbuff, sizeof(msgstr));
  }

  strlcat(msgstr, " ] ", sizeof(msgstr));
  strlcat(msgstr, msg->rtr ? "R" : "", sizeof(msgstr));
  strlcat(msgstr, msg->ext ? "X" : "", sizeof(msgstr));

  return msgstr;
}

//
/// determine whether the message opcode is a CBUS event
//

bool is_CBUS_event(byte opcode) {

  switch (opcode) {

    case OPC_ACON:
    case OPC_ACON1:
    case OPC_ACON2:
    case OPC_ACON3:

    case OPC_ACOF:
    case OPC_ACOF1:
    case OPC_ACOF2:
    case OPC_ACOF3:

    case OPC_ARON:
    case OPC_AROF:

    case OPC_ASON:
    case OPC_ASON1:
    case OPC_ASON2:
    case OPC_ASON3:

    case OPC_ASOF:
    case OPC_ASOF1:
    case OPC_ASOF2:
    case OPC_ASOF3:

      return true;
      break;

    default:
      return false;
  }
}

//
/// send a CBUS message to EXRAIL over a serial link
//

void send_message_to_EXRAIL(CANFrame *msg) {

  // *** TODO - apply a message filter, format the string and write to the EXRAIL serial port
  Serial << F("> sending CAN message to EXRAIL = ") << format_CAN_message(msg) << endl;

  return;
}

//
/// read data from EXRAIL, build up a string, process at EOL
//

void read_data_from_EXRAIL(void) {

  static bool som_rcvd = false;                                // has the start-of-message char been received ?
  static char input_buffer[EXRAIL_BUFFER_SIZE] = {'\0'};       // buffer in which to build message
  static byte input_buffer_index = 0;                          // current offset into buffer where next char will be written

  while (EXRAIL_SERIAL_PORT.available()) {

    char c = EXRAIL_SERIAL_PORT.read();

    switch (c) {
      case '<':                                                // start of new message
        input_buffer_index = 0;                                // reset the buffer index
        som_rcvd = true;
        break;

      case '>':                                                // end of message
        process_message_from_EXRAIL(input_buffer);             // process the string
        input_buffer_index = 0;                                // reset the buffer index
        som_rcvd = false;
        break;

      default:                                                 // any other alphanumeric character
        if (som_rcvd && isalnum(c)) {                          // append to string only if SOM has been received
          input_buffer[input_buffer_index] = c; 
          ++input_buffer_index;

          if (input_buffer_index >= EXRAIL_BUFFER_SIZE) {      // input too long -- discard
            input_buffer_index = 0;
            som_rcvd = false;
          }
        }

        break;
    }

    input_buffer[input_buffer_index] = '\0';                   // ensure string is always null terminated
  }

  return;
}

//
/// process a complete message from EXRAIL
//

void process_message_from_EXRAIL(const char *buffer) {

  // *** TODO
  Serial << F("> processing message from EXRAIL = ") << buffer << endl;

  return;
}


//
/// print code version, config details and copyright notice
//

void print_config(void) {

  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Duncan Greenwood (M5767), 2023") << endl;

  return;
}

//
/// command interpreter for serial console input
//

void process_serial_input(void) {

  byte uev = 0;
  char msgstr[32], dstr[32];

  if (Serial.available()) {

    char c = Serial.read();

    switch (c) {

      case 'n':

        // node config
        print_config();

        // node identity
        Serial << F("> CBUS node configuration") << endl;
        Serial << F("> mode = ") << (module_config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID << F(", node number = ") << module_config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':

        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;
        Serial << F("  max events = ") << module_config.EE_MAX_EVENTS << F(" EVs per event = ") << module_config.EE_NUM_EVS << F(" bytes per event = ") << module_config.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (module_config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * module_config.EE_BYTES_PER_EVENT) << F(" of ") << (module_config.EE_MAX_EVENTS * module_config.EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (module_config.EE_NUM_EVS); j++) {
          sprintf(dstr, "EV%03d | ", j + 1);
          Serial << dstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {

          if (module_config.getEvTableEntry(j) != 0) {
            sprintf(dstr, "  %03d  | ", j);
            Serial << dstr;

            // for each data byte of this event
            for (byte e = 0; e < (module_config.EE_NUM_EVS + 4); e++) {
              sprintf(dstr, " 0x%02hx | ", module_config.readEEPROM(module_config.EE_EVENTS_START + (j * module_config.EE_BYTES_PER_EVENT) + e));
              Serial << dstr;
            }

            sprintf(dstr, "%4d |", module_config.getEvTableEntry(j));
            Serial << dstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':

        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= module_config.EE_NUM_NVS; j++) {
          byte v = module_config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      case 'c':
        // CAN bus status
        CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        module_config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        CBUS.reset();
        break;

      case '*':
        // reboot
        module_config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << module_config.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // reset module
        module_config.resetModule();
        break;

      case '\r':
      case '\n':
        Serial << endl;
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }

  return;
}
