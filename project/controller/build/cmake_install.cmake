# Install script for directory: /home/dika/projects/hvac/project/controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so"
         RPATH "\$ORIGIN:\$ORIGIN/..:\$ORIGIN/../..")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller" TYPE SHARED_LIBRARY FILES "/home/dika/projects/hvac/project/controller/build/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so"
         OLD_RPATH "/home/dika/choreonoid/build/lib:"
         NEW_RPATH "\$ORIGIN:\$ORIGIN/..:\$ORIGIN/../..")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarMinimumController.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so"
         RPATH "\$ORIGIN:\$ORIGIN/..:\$ORIGIN/../..")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller" TYPE SHARED_LIBRARY FILES "/home/dika/projects/hvac/project/controller/build/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so"
         OLD_RPATH "/home/dika/choreonoid/build/lib/choreonoid-2.4:/home/dika/choreonoid/build/lib:"
         NEW_RPATH "\$ORIGIN:\$ORIGIN/..:\$ORIGIN/../..")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/RobotBesarWalkController.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/jalan_lari.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/jalan_lari.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/jalan_lari.so"
         RPATH "\$ORIGIN:\$ORIGIN/..:\$ORIGIN/../..")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller" TYPE SHARED_LIBRARY FILES "/home/dika/projects/hvac/project/controller/build/lib/choreonoid-2.4/simplecontroller/jalan_lari.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/jalan_lari.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/jalan_lari.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/jalan_lari.so"
         OLD_RPATH "/home/dika/choreonoid/build/lib:"
         NEW_RPATH "\$ORIGIN:\$ORIGIN/..:\$ORIGIN/../..")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/choreonoid-2.4/simplecontroller/jalan_lari.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/dika/projects/hvac/project/controller/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
