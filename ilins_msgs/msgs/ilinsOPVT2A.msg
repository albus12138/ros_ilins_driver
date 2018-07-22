###########################################################################
#
# Author: r4phael
#
# Copyright (C) 2018 r4phael
#
# Email: albus.zly@gmail.com
#
###########################################################################
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
###########################################################################
#
# Last revision: July 22, 2018
#
# For more info and how to use this library, visit: https://github.com/albus12138/ros_ilins_driver
#
###########################################################################

uint16 heading
int16 pitch
int16 roll

int16 gyroX
int16 gyroY
int16 gyroZ

int16 accX
int16 accY
int16 accZ

int16 magX
int16 magY
int16 magZ

uint16 usw
uint16 voltage
int16 temperature

int32 latitude
int32 longitude
int32 altitude

int32 east_speed
int32 north_speed
int32 vertical_speed

int32 GNSS_latitude
int32 GNSS_longitude
int32 GNSS_altitude
int32 GNSS_horizontal_speed
int16 track_over_ground
int32 GNSS_vertical_speed
uint32 timestamp
uint8 GNSS_info1
uint8 GNSS_info2
uint8 solnSVs
uint16 V_latency
uint8 angle_position_type
uint16 GNSS_heading
int16 GNSS_pitch
uint16 GNSS_STD_heading
uint16 GNSS_STD_pitch
uint16 P_bar
int32 H_bar
uint8 New_GPS