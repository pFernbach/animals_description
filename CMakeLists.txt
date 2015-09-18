# Copyright (c) 2015 CNRS
# Author: Mylene Campana
#
# This file is part of animals_description.
# animals_description is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# animals_description is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# animals_description.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME animals_description)
SET(PROJECT_DESCRIPTION "Data specific to animal robot for hpp-corbaserver")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON()
ADD_REQUIRED_DEPENDENCY ("hpp-corbaserver >= 3")

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

 # environment URDF - DAE - textures
install(FILES urdf/multiple_planes_3d.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(FILES urdf/plane_3d.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(FILES urdf/inclined_plane_3d.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(FILES urdf/environment_3d.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(FILES urdf/environment_3d_with_window.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(FILES urdf/waterLily_envir.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
  
install(FILES meshes/waterLily2.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
  
install(FILES meshes/SuirenT.bmp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes) # texture waterlily
  
install(FILES meshes/arbre.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
  
install(FILES meshes/bark_000.bmp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes) # texture arbre

install(FILES meshes/DL2X2_00.bmp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes) # texture arbre

install(FILES meshes/wood_stack.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(FILES meshes/8wood.jpg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(FILES meshes/bush.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(FILES meshes/etang.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(FILES meshes/etang2.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
  
install(FILES meshes/Stone_3872_x_2592_px_0606.jpg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes) # texture pond stones

install(FILES meshes/Stone_3000_x_2250_px_050613.jpg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes) # texture pond stones

install(FILES meshes/grass.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(FILES meshes/stones2.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
  
install(FILES meshes/stones3.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(FILES meshes/Marble_900_x_634_px_mfr_14.jpg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)  # texture stones

install(FILES meshes/1.jpg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)  # texture stones

install(FILES meshes/Stone_2240_x_1680_px_0706.jpg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)  # texture stones

install(FILES meshes/Granite_483_x_384_px_levadia.jpg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)  # texture stones


 # URDF - SRDF - DAE
install(FILES
  urdf/ant.urdf
  urdf/sphere.urdf
  urdf/ant_sphere.urdf
  urdf/box.urdf
  urdf/rabbit.urdf
  urdf/cave.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )

install(FILES
  srdf/ant.srdf
  srdf/sphere.srdf
  srdf/ant_sphere.srdf
  srdf/box.srdf
  srdf/rabbit.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
  meshes/ant.dae
  meshes/ant_black.dae
  meshes/ant_texture.jpg
  meshes/rabbit.dae
  meshes/rabbit1.dae
  meshes/cave.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  )

install (FILES
  src/hpp/corbaserver/rabbit/robot.py
  src/hpp/corbaserver/rabbit/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rabbit)

install (FILES
  src/hpp/corbaserver/ant/robot.py
  src/hpp/corbaserver/ant/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/ant)

install (FILES
  src/hpp/corbaserver/sphere/robot.py
  src/hpp/corbaserver/sphere/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/sphere)

install (FILES
  src/hpp/corbaserver/ant_sphere/robot.py
  src/hpp/corbaserver/ant_sphere/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/ant_sphere)

install (FILES
  src/hpp/corbaserver/box/robot.py
  src/hpp/corbaserver/box/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/box)
SETUP_PROJECT_FINALIZE()
