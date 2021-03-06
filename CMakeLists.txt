# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-hrp2.
# hpp-hrp2 is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-hrp2 is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-hrp2.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-hrp2)
SET(PROJECT_DESCRIPTION "Data specific to hrp2 robot for hpp-corbaserver")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON()
ADD_REQUIRED_DEPENDENCY ("hpp-corbaserver >= 3")
ADD_OPTIONAL_DEPENDENCY ("hpp-manipulation-corba")

INSTALL(FILES
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/hrp2/robot.py
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/hrp2/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/hrp2
)

IF (HPP_MANIPULATION_CORBA_FOUND)
  INSTALL (FILES
    ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/manipulation/hrp2/robot.py
    ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/manipulation/hrp2/__init__.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation/hrp2
    )
ENDIF ()
SETUP_PROJECt_FINALIZE()
