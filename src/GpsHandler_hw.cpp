/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                           EQUIPMENT  HANDLERS                              --
--                                                                            --
--                           GPSHandlerHW Source                              --
--                                                                            --
--            Copyright (C) 2023 Universidad Politécnica de Madrid            --
--                                                                            --
-- HERCCULES was developed by the Real-Time Systems Group at  the Universidad --
-- Politécnica de Madrid.                                                     --
--                                                                            --
-- HERCCULES is free software: you can redistribute it and/or modify it under --
-- the terms of the GNU General Public License as published by the Free Soft- --
-- ware Foundation, either version 3 of the License,  or (at your option) any --
-- later version.                                                             --
--                                                                            --
-- HERCCULES is distributed  in the hope that  it will be useful  but WITHOUT --
-- ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FIT- --
-- NESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more --
-- details. You should have received a copy of the GNU General Public License --
-- along with HERCCULES. If not, see <https://www.gnu.org/licenses/>.         --
--                                                                            --
------------------------------------------------------------------------------*/

#include "GpsHandler_hw.h"

#include <cstdio>       // fgets to read one line
#include <cstring>      // String function definitions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // write
#include <iostream>     // cout

/*****************
 * Local variables
 *****************/
namespace {
    int serialPort {-1};
    FILE * serialPortBuffered = nullptr;
}

/********************
 * Private Operations
 ********************/
namespace {

    /***************************************************************************
     * Serial Port Low Level Ops
     **************************************************************************/

    // configSerialPort
    // ================
    // Configures the communication parameters and other interface properties
    // with the help of specific termios functions and data structures.
    // The default configuration for I/O is 8N1 and 9600 bauds:
    bool configSerialPort() {

        (void) fcntl(serialPort, F_SETFL, 0);
        // undo O_NDELAY, we needed nonblocking mode only to open the device,
        // but blocking for I/O operations!

        // Retrieve default configuration:
        termios serialPortConfig {};
        if(tcgetattr(serialPort, &serialPortConfig) < 0) {
            perror("tcgetattr serialPort");
            return false;
        }

        // c_iflag: input mode
        // - - - - - - - - - -
        // Turn off input processing:
        serialPortConfig.c_iflag &= ~IGNBRK; // A break
        serialPortConfig.c_iflag &= ~BRKINT; //  reads as null byte
        serialPortConfig.c_iflag &= ~IGNPAR; // Framing errors and parity errors
        serialPortConfig.c_iflag &= ~PARMRK; //  read as null bytes
        serialPortConfig.c_iflag &= ~INPCK;  // Disable input parity checking
        serialPortConfig.c_iflag &= ~ISTRIP; // Do not strip off 8th bit
        serialPortConfig.c_iflag &= ~INLCR;  // No TL to CR translation
        serialPortConfig.c_iflag &= ~IGNCR;  // Do not ignore carriage return on input
        serialPortConfig.c_iflag &= ~ICRNL;  // No CR to NL translation
        serialPortConfig.c_iflag &= ~IXOFF;  // disable xon/xoff on input
        serialPortConfig.c_iflag &= ~IXON;   // and output
        serialPortConfig.c_iflag &= ~IXANY;  // Only START restarts the stopped output

        // c_oflag: output mode
        // - - - - - - - - - - -
        // prevent special interpretation of output bytes (e.g. newline chars)
        // no CR to NL translation, no NL to CR-NL translation,
        // no NL to CR translation, no column 0 CR suppression,
        // no Ctrl-D suppression, no fill characters, no case mapping.
        serialPortConfig.c_oflag = 0U;

        // c_cflag: control modes
        // - - - - - - - - - - - -
        // 8N1 configuration:
        serialPortConfig.c_cflag &= ~PARENB; // no parity bit
        serialPortConfig.c_cflag &= ~CSTOPB; // 1 Stop bit
        serialPortConfig.c_cflag &= ~CSIZE;  // clear previous bits per byte and
        serialPortConfig.c_cflag |= CS8;     // set 8 bits per byte
        serialPortConfig.c_cflag |= CLOCAL;  // ignore modem control lines
        serialPortConfig.c_cflag |= CREAD;   // enable receiver!

        // c_lflag: local modes
        // - - - - - - - - - - -
        // We were interested in cannonical mode since NMEA messages stop with
        // the <CR><LF> characters. This does not work for UBX messages since they
        // are binary messages.
        serialPortConfig.c_lflag |= ICANON;  // canonical mode ON
        serialPortConfig.c_lflag &= ~ECHO;   // ECHO off
        serialPortConfig.c_lflag &= ~ECHONL; // ECHO newline off
        serialPortConfig.c_lflag &= ~ECHOK;  // do not echo NL after KILL character
        serialPortConfig.c_lflag &= ~ECHOE;  // disable erasure
        serialPortConfig.c_lflag &= ~IEXTEN; // extended input processing ignored
        serialPortConfig.c_lflag &= ~ISIG;   // disable interpretation of INTR, QUIT and SUSP

        // Baud rate
        // - - - - -
        (void) cfsetspeed(&serialPortConfig, B9600);
        // Set to 9600 bauds

        // Apply configuration immediately and flush the  I/O buffers:
        if (tcsetattr(serialPort, TCSAFLUSH, &serialPortConfig) == 0) {
            printf("Success: set tty config\n");
        } else {
            perror("tcsetattr");
        }

        return true;
    }

    // setICANON
    // =========
    // flag = true  --> Canonical mode
    // flag = false --> Non canonical mode
    void setICANON(bool flag) {
        termios serialPortConfig {};
        if(tcgetattr(serialPort, &serialPortConfig) < 0) {
            perror("tcgetattr serialPort");
        }

        if (flag) {
            serialPortConfig.c_lflag |=  ICANON; // canonical mode ON
        } else {
            serialPortConfig.c_lflag &= ~ICANON; // canonical mode OFF

            // read will block until VMIN bytes are available and returns
            // from VMIN up to the number of requested bytes:
            serialPortConfig.c_cc[VMIN]  = equipment_handlers::gps_handler::hw::MIN_MESSAGE_SIZE_BYTES;
            serialPortConfig.c_cc[VTIME] = 0U;
        }

        // Apply configuration immediately and flush the I/O buffers:
        if (tcsetattr(serialPort, TCSAFLUSH, &serialPortConfig) == 0) {
            printf("Success: set tty config\n");
        } else {
            perror("tcsetattr");
        }
    }

    /***************************************************************************
     * UBX Port Low Level Ops
     **************************************************************************/

    bool serialRead(unsigned char * msg, size_t msg_length) {
        bool sync_read [] {false, false};
        const char sync_seq [] = {0xB5, 0x62}; ///< SYNC sequence

        do {
            unsigned char ch = 0x00;            
            read(serialPort, &ch, 1U);

            if (sync_read[0]) {
                sync_read[1] = ch == sync_seq[1];
                sync_read[0] = sync_read[1];                
            } else {
                sync_read[0] = ch == sync_seq[0];
            }
        } while (!sync_read[0] || !sync_read[1]);

        msg[0] = sync_seq[0];
        msg[1] = sync_seq[1];

        return serialPort > 0 &&
               ssize_t (msg_length - 2U) == read(serialPort, &msg[2], msg_length - 2U);
    }

    bool serialWrite(unsigned char * msg, size_t msg_length) {        
        return serialPort > 0 &&
               ssize_t (msg_length) == write(serialPort, msg, msg_length);
    }

    void applyChecksum(unsigned char * ubx_msg, size_t length) {
        unsigned char ck_a {0U};
        unsigned char ck_b {0U};

        for (size_t i = 2U; i < (length - 2); ++i) {
            ck_a = ck_a + ubx_msg[i];
            ck_b = ck_b + ck_a;
        }

        ubx_msg[length-2] = ck_a;
        ubx_msg[length-1] = ck_b;
    }

    bool ackWasReceived(unsigned char classId, unsigned char msgId) {
        constexpr size_t ack_msg_size {10U}; ///< Size in bytes for ACK/NACK UBX messages

        unsigned char msg [ack_msg_size];
        (void) memset(msg, 0, ack_msg_size);
       
        (void) serialRead(msg, ack_msg_size);

        unsigned char msg_expected [] = {
            0xB5, 0x62,
            0x05, 0x01,
            0x02, 0x00,
            classId, msgId,
            0x00, 0x00
        };
        applyChecksum(msg_expected, ack_msg_size);

        bool isAck {true};
        for (size_t i = 0; i < ack_msg_size && isAck; ++i) {
            isAck = isAck && (msg[i] == msg_expected[i]);
        }

        return isAck;
    }

    /***************************************************************************
     * Dynamic Platform Model Ops
     **************************************************************************/

    enum DynModel : char {
        PORTABLE = 0,
        STATIONARY = 2,
        PEDESTRIAN = 3,
        AUTOMOTIVE = 4,
        SEA = 5,
        AIRBORNE_LT_1G = 6,
        AIRBORNE_LT_2G = 7,
        AIRBORNE_LT_4G = 8,
    };

    // getDynamicPlatformModel
    // =======================
    bool getDynamicPlatformModel(DynModel &dynModel)
    {
        setICANON(true);

        /// @addtogroup Request CFG-NAV5
        /// @{
        unsigned char pollNav[] {
            0xB5, 0x62, ///< Header: SYNC CHAR 1 & SYNC CHAR 2
            0x06, 0x24, ///< CLASS & ID
            0x00, 0x00, ///< Length
            0x00, 0x00  ///< CK_A & CK_B
        };
        applyChecksum(pollNav, sizeof(pollNav));

        if (serialWrite(pollNav, sizeof(pollNav))) {
            std::cout << "[GPS-HW] Successfull dynMode poll!" << std::endl;
        } else {
            std::cout << "[GPS-HW] Could not poll dynMode" << std::endl;
        }
        /// @}

        /// @addtogroup Reads CFG-NAV5
        /// @{
        unsigned char getNav[44U];
        (void) memset(getNav, 0, sizeof(getNav));
        bool success = serialRead(getNav, sizeof(getNav));
        /// @}

        setICANON(false);

        dynModel = DynModel (getNav[8U]);
        return success;
    }

    // setDynamicPlatformModel
    // =======================
    bool setDynamicPlatformModel(DynModel dynModel)
    {
        setICANON(true);

        /// @addtogroup Preparing Navigation configuration command
        /// @{
        unsigned char setNav[44U] = {
            0xB5, 0x62, ///< Header: SYNC CHAR 1 & SYNC CHAR 2

            0x06, 0x24, ///< CLASS & ID

            0x24, 0x00, ///< LENGTH Little Endian (Bytes) = 36

            0x01, 0x00,                  ///< Bitmask: | ... | dgpsMask | staticHoldMask | timeMask | posMask | drLin | fixMode | minEl | dyn |
            static_cast<char>(dynModel), ///< dynModel
            0x03,                        ///< fixMode
            0x00, 0x00, 0x00, 0x00,      ///< fixedAlt
            0x10, 0x27, 0x00, 0x00,      ///< fidexAltVar
            0x05,                        ///< minElev
            0x00,                        ///< drLimit
            0xFA, 0x00,                  ///< pDop
            0xFA, 0x00,                  ///< tDop
            0x64, 0x00,                  ///< pAcc
            0x2C, 0x01,                  ///< tAcc
            0x00,                        ///< StaticHoldThresh
            0x3C,                        ///< dgpsTimeOut
            0x00, 0x00, 0x00, 0x00,      ///< reserved2
            0x00, 0x00, 0x00, 0x00,      ///< reserved3
            0x00, 0x00, 0x00, 0x00,      ///< reserved4

            0x16, 0xDC ///< CK_A & CK_B
        };

        applyChecksum(setNav, sizeof(setNav));
        /// @}

        if (serialWrite(setNav, sizeof(setNav))) {
            std::cout << "[GPS-HW] Successfull Configured dynMode!" << std::endl;
        } else {
            std::cout << "[GPS-HW] Could not config dynMode" << std::endl;
        }

        bool success = ackWasReceived(0x06, 0x24);

        setICANON(false);

        DynModel readDynModel;
        getDynamicPlatformModel(readDynModel);

        return success && readDynModel == dynModel;
    }
}

/**********************************
 * Public functions implementations
 **********************************/

namespace equipment_handlers::gps_handler::hw {

    bool initialize(std::string const & serial_port_name) {
            serialPort = open(serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            // |-- O_NOCTTY: prevent the serial device from becoming the controlling terminal,
            // |             so this won't be the controlling process.
            // |-- O_NDELAY: non-blocking mode for open, and I/O operations

            if (serialPort < 0) {
                return false;
            } else {
                serialPortBuffered = fdopen(serialPort, "r");
            }

            if ( ! configSerialPort() ) {
                return false;
            }

            DynModel readDynModel;
            getDynamicPlatformModel(readDynModel);
            if ( ! setDynamicPlatformModel(DynModel::AIRBORNE_LT_1G) ) {
                return false;
            }

            return true;
    }

    // Additional NMEA messages can be de/activated with the CFG-MSG UBX command
    bool readNMEAMessage(char * message, size_t size) {

        if(serialPortBuffered == nullptr) {
            return false;
        }

        (void) memset(message, 0, size);

        constexpr int maxRetries = 2;
        bool validLine = false;

        // Exit from loop when: found a valid line OR
        //                      we have exceeded the max number of retries
        for (int nRetries = 0; nRetries <= maxRetries && !validLine; nRetries++) {
            if (fgets(message, MAX_MESSAGE_SIZE_BYTES, serialPortBuffered) != nullptr) {
                char first = message[0];
                char last  = message[strlen(message) - 1U];
                validLine  = (first == '$') &&
                             (last == '\r' || last == '\n');
            }
        }

        if (validLine) {
            printf(">>> NMEA: %s <<<\n", message);
        }

        return validLine;
    }

    void cleanRxBuffer() {
        (void) tcflush(serialPort, TCIOFLUSH);
    }
}

// Notes for LEA6-S
// ================
//
// - Navigation update rate: 5 Hz
//
// - time for cold starts  : 26 secs
//            aided starts : 1 sec   <- Requires the usage of Assisted GPS (A-GPS)
//            hot starts   : 1 sec
//
// - Serial I/Fs:
//     * UART (Used in HERCCULES)
//     * USB
//     * I2C
//
// - Time pulse: 0.25 Hz to 1kHz
//
// - Message protocols:
//     * NMEA       : I/O, ASCII, 0183, 2.3 compatible with 3.0, activated at startup for I/O
//     * UBX binary : I/O, binary, u-blox proprietary, activated at startup ONLY for Input.
//     * RTCM       : I, 2.3
// - The settings stablished after boot-time, can be saved in the battery-backup
//   RAM, as long as the backup battery supply is not interrupted
//
// - Most UBX messages' format are in Little-Endian

// Interesting sites for serial programming
// ========================================
//
// [1] https://en.wikibooks.org/wiki/Serial_Programming/termios
// [2] http://www.strz.uni-giessen.de/~k3/pipe/SerialProgrammingGuide54.pdf
// [3] https://github.com/Lora-net/sx1302_hal/blob/6dff8191d5034539e43990c7b9a4d1bc3d5b6658/packet_forwarder/src/lora_pkt_fwd.c#L3389
//     https://github.com/Lora-net/sx1302_hal/blob/6dff8191d5034539e43990c7b9a4d1bc3d5b6658/libloragw/src/loragw_gps.c
// [4] Good code structure https://github.com/wdalmut/libgps/tree/master/src
// [5] Many returns from a function https://stackoverflow.com/questions/55718579/how-to-avoid-using-multiple-if-else-to-check-whether-the-returned-value-is-an-er
// [6] Read complete line (instead of chunks) from file descriptor: http://laurel.datsi.fi.upm.es/~ssoo/sockets/7_recepcion_completa/LEEME.html
// [7] setFlightMode: https://github.com/projecthorus/FlexTrack-Horus/blob/master/gps.ino#L81