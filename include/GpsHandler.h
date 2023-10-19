/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                           EQUIPMENT  HANDLERS                              --
--                                                                            --
--                            GPSHandler Header                               --
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

#ifndef HAL_READGPS_H
#define HAL_READGPS_H

#include <string>
#include <functional>   // std::function for callback

/**
 * High level interface for the GPS MIKROE-1032 board which features a
 * LEA-6S high performance u-blox 6 position engine. The LEA-6S model
 * supports the GPS satellite constellation.
 */
namespace equipment_handlers::gps_handler {

    // Notation: std::function<return_type(parameter_type_1, parameter_type_2, parameter_type_3)>
    // void for now
    using receiveNMEADataCallback = std::function <void ()>;

    // Setups the necessary resources to read the GPS.
    // NOTE that this function must be called before any operation.
    bool initialize(std::string const & serial_port_name = "/dev/serial0");

    // Starts the periodic thread to look for NMEA messages, when the
    // required GPS data is found, the callback is invoked.
    void startDataAcquisition(receiveNMEADataCallback cb);

}

#endif //HAL_READGPS_H