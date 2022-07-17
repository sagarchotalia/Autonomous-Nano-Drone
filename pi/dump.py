"""simpleUI.py: Simple UI (toy one really) to test YAMSPy using a SSH connection.

Copyright (C) 2020 Ricardo de Azambuja

This file is part of YAMSPy.

YAMSPy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

YAMSPy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with YAMSPy.  If not, see <https://www.gnu.org/licenses/>.


WARNING:
This UI is not fit for flying your drone, it is only useful for testing stuff in a safe place, ok?

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.


TODO:
1) The integrator makes it too hard to control things (it winds up). Probably a non-linear thing would be helpful here...
2) Everything is far from optimal so it could be improved... but it's a simpleUI example after all ;)
"""


__author__ = "Ricardo de Azambuja"
__copyright__ = "Copyright 2020, MISTLab.ca"
__credits__ = [""]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Ricardo de Azambuja"
__email__ = "ricardo.azambuja@gmail.com"
__status__ = "Development"

