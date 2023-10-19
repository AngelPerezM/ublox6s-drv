/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                           EQUIPMENT  HANDLERS                              --
--                                                                            --
--                           GPSHandlerHW Header                              --
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

#ifndef OBSW_GPSHANDLER_HW_H
#define OBSW_GPSHANDLER_HW_H

#include <string>
#include <cstdint>

namespace equipment_handlers::gps_handler::hw {

    // ---------
    // Constants
    // ---------

    constexpr uint8_t MIN_MESSAGE_SIZE_BYTES =  8U;
    constexpr uint8_t MAX_MESSAGE_SIZE_BYTES = 82U;

    // ----------------
    // Public functions
    // ----------------

    // Initializes the access for the serial port device, if this operation
    // fails (returns false) then the other functions will fail, too.
    bool initialize(std::string const & serial_port_name);

    // Blocking operation that retrieves the raw NMEA reading in the message output parameter.
    // Possible NMEA messages are: GGA, GGL, GSA, GSV, RMC, VTG, TXT, or ZDA.
    // Returns true on success
    bool readNMEAMessage(char * message, size_t size);

    // clean intermediate buffer from the RX line.
    void cleanRxBuffer();
}

#endif //OBSW_GPSHANDLER_HW_H
