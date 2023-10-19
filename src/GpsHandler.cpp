/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                           EQUIPMENT  HANDLERS                              --
--                                                                            --
--                            GPSHandler Source                               --
--                                                                            --
--            Copyright (C) 2022 Universidad Politécnica de Madrid            --
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

#include "GpsHandler.h"
#include "GpsHandler_hw.h"
#include "minmea.h"

#include <pthread.h>

namespace eh = equipment_handlers;

namespace {
    // ---------------------------
    // Local variables & constants
    // ---------------------------

    bool initialized {false};
    pthread_t dataReaderTh_handler;
    eh::gps_handler::receiveNMEADataCallback callback;

    // -----------------
    // Private functions
    // -----------------

    void nmeaparser(char nmeaMessage[eh::gps_handler::hw::MAX_MESSAGE_SIZE_BYTES]) {
        switch (minmea_sentence_id(nmeaMessage, false)) {
            case MINMEA_SENTENCE_RMC: {
                struct minmea_sentence_rmc frame;
                if (minmea_parse_rmc(&frame, nmeaMessage)) {
                    if(frame.valid){
                        printf("$RMC Latitud, longitud y velocidad: (%f,%f) %f nudos\n",
                               minmea_tocoord(&frame.latitude),
                               minmea_tocoord(&frame.longitude),
                               minmea_tofloat(&frame.speed));
                        printf("Hora: %02d:%02d:%02d", frame.time.hours, frame.time.minutes, frame.time.seconds);
                    } else {
                        printf("Los datos no son válidos.\n");
                    }
                } else {
                    printf("Mensaje RMC no parseado.\n");
                }
            } break;

            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, nmeaMessage)) {
                    printf("$GGA: fix quality: %d\n", frame.fix_quality);
                } else {
                    printf("Mensaje GGA no parseado.\n");
                }
            } break;

            case MINMEA_SENTENCE_GSV: {
                struct minmea_sentence_gsv frame;
                if (minmea_parse_gsv(&frame, nmeaMessage)) {
                    printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                    printf("$GSV: sattelites in view: %d\n", frame.total_sats);
                    for (int i = 0; i < 4; i++)
                        printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                               frame.sats[i].nr,
                               frame.sats[i].elevation,
                               frame.sats[i].azimuth,
                               frame.sats[i].snr);
                } else {
                    printf("Mensaje GSV no parseado.\n");
                }
            } break;

            case MINMEA_SENTENCE_GLL: {
                struct minmea_sentence_gll frame;
                if(minmea_parse_gll(&frame, nmeaMessage)){
                    printf("Latitud: %fº\n", minmea_tocoord(&frame.latitude));
                    printf("Longitud %fº\n", minmea_tocoord(&frame.longitude));
                    printf("Hora: %02d:%02d:%02d", frame.time.hours, frame.time.minutes, frame.time.seconds);
                }else{
                    printf("Mensaje GLL no parseado.\n");
                }
            } break;

            case MINMEA_SENTENCE_GSA: {
                struct minmea_sentence_gsa frame;
                if(minmea_parse_gsa(&frame, nmeaMessage)){
                    switch(frame.fix_type){
                        case 3:{
                            printf("Operando en modo 3D.\n");
                        }break;
                        case 2:{
                            printf("Operando en modo 2D.\n");
                        }break;
                        default:{
                            printf("No está disponible la información del modo de operacion.\n");
                        }break;
                    }
                    for(int i = 0; i<12; i++){
                        if(frame.sats[i]!=0) {
                            printf("PRN sat %d: %d dB\n", i + 1, frame.sats[i]);
                        }
                    }
                    printf("pdop: %f\n", minmea_tofloat(&frame.pdop));
                    printf("vdop: %f\n", minmea_tofloat(&frame.vdop));
                    printf("hdop: %f\n", minmea_tofloat(&frame.hdop));
                }else{
                    printf("Mensaje GSA no parseado.\n");
                }
            } break;

            case MINMEA_SENTENCE_VTG: {
                struct minmea_sentence_vtg frame;
                if (minmea_parse_vtg(&frame, nmeaMessage)){
                    printf("Ruta en grados reales: %lfº\n", minmea_tofloat(&frame.true_track_degrees));
                    printf("Velocidad: %lf km/h, %lf nudos", minmea_tofloat(&frame.speed_kph),minmea_tofloat(&frame.speed_knots));
                } else {
                    printf("Mensaje VTG no parseado.\n");
                }
            } break;

            case MINMEA_INVALID: {
                printf("Mensaje inválido, checksum malo\n");
            } break;

            default: {
                printf("\t$xxxxx sentence is not parsed\n");
            } break;
        }
    }

    /*
     * Sporadic thread that reads NMEA messages
     */
    void * dataReaderTh(void * threadId) {
        (void) threadId;

        // Setup local resources:
        static char nmeaMessage[eh::gps_handler::hw::MAX_MESSAGE_SIZE_BYTES];
        eh::gps_handler::hw::cleanRxBuffer();

        // Sporadic activity:
        while (true) {
            bool success = eh::gps_handler::hw::readNMEAMessage
                    (nmeaMessage,
                     eh::gps_handler::hw::MAX_MESSAGE_SIZE_BYTES);
            if (success) {
                nmeaparser(nmeaMessage);
                callback();
            } else {
                puts("FAILED readNMEAMessage");
            }
        }

    }
}

// --------------------------------
// Public functions implementations
// --------------------------------

namespace equipment_handlers::gps_handler {

    bool initialize(std::string const & serial_port_name) {
        if (!initialized) {
            initialized = eh::gps_handler::hw::initialize(serial_port_name);
        }
        return initialized;
    }

    void startDataAcquisition(receiveNMEADataCallback cb) {

        // save callback function in global variable
        callback = std::move(cb);

        // start the data reader thread
        int i  {0};
        int rc {0};
        rc = pthread_create(&dataReaderTh_handler, NULL, dataReaderTh, (void *) i);
        if (rc) {
            perror("pthread_create");
        }

    }

} // equipment_handlers::gps_reader


/* =======
 * = RMC =
 * =======
 * Recommended Minimum data
 *  - $GPRMC
 *  - Time of position fix [hhmmss.ss]
 *  - status: validity of data
 *  - latitude [ddmm.mmmm]
 *  - N/S
 *  - longitude [dddmm.mmmm]
 *  - E/W
 *  - Speed over gnd [knots]
 *  - course over gnd [degrees]
 *  - Date: [dd-mm-yy]
 *  - ... empty fields ...
 *  - Checksum
 *  - <CR><LF>
 *
 *
 * =======
 * = GGA =
 * =======
 * Global positioning system fix data
 *  - $GPGGA
 *  - UTC Time [hhmmss.sss]
 *  - Latitude [ddmm.mmmm]
 *  - N/S
 *  - Longitude [ddmm.mmmm]
 *  - E/W
 *  - Fix validity
 *  - Satellites used [0 -> 12]
 *  - HDOP [hdop]
 *  - Mean Sea Level altitude [meters]
 *  - M
 *  - Geoid Separation [Altref]
 *  - M
 *  - Diff. reference station
 *  - Checksum
 *  - <CR><LF>
 *
 * =======
 * = GSV =
 * =======
 * GNSS Satellites in View
 * GSV messages can be divided in various NMEA sentences!
 *  - $GPGSV
 *  - ...
 *
 * =======
 * = GLL =
 * =======
 * Latitude and longitude, with time of position fix and status
 *  - $GPGLL
 *  - Latitude [ddmm.mmmm]
 *  - N/S
 *  - Longitude [dddmm.mmmm]
 *  - E/W
 *  - UTC time [hhmmss.ss]
 *  - Validity
 *  - Position fix mode
 *  - Checksum
 *  - <CR><LF>
 *
 * =======
 * = GSA =
 * =======
 * GNSS Dilution Of Precision and Active Satellites
 *  - $GPGSA
 *  - Mode
 *  - Fix status
 *  - ...
 *
 * =======
 * = VTG =
 * =======
 * Course over ground and Ground speed
 *  - $GPVTG
 *  - Course over ground true [degrees]
 *  - Course over ground magnetic [degrees]
 *  - Speed over ground [knots]
 *  - Speed over ground [kph]
 *  - mode indicator
 *  - Checksum
 *  - <CR><LF>
 */