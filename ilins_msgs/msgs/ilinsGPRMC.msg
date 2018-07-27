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
# Last revision: July 26, 2018
#
# For more info and how to use this library, visit: https://github.com/albus12138/ros_ilins_driver
#
###########################################################################

string message_id

float64 utc_seconds
string position_status

float64 lat
char lat_dir

float64 lon
char lon_dir

float32 speed
float32 track
string date
float32 mag_var
string mag_var_direction
string mode_indicator

uint32 checksum