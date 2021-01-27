#
# General CPack configuration
#
# Version information
if(NOT DEFINED CPACK_PACKAGE_VERSION_MAJOR)
  execute_process(
    COMMAND git describe --abbrev=0
    COMMAND cut -d. -f1
    COMMAND tr -d '\n'
    OUTPUT_VARIABLE CPACK_PACKAGE_VERSION_MAJOR)
endif(NOT DEFINED CPACK_PACKAGE_VERSION_MAJOR)
if(NOT DEFINED CPACK_PACKAGE_VERSION_MINOR)
  execute_process(
    COMMAND git describe --abbrev=0
    COMMAND cut -d. -f2
    COMMAND tr -d '\n'
    OUTPUT_VARIABLE CPACK_PACKAGE_VERSION_MINOR)
endif(NOT DEFINED CPACK_PACKAGE_VERSION_MINOR)
if(NOT DEFINED CPACK_PACKAGE_VERSION_PATCH)
  execute_process(
    COMMAND git describe --abbrev=0
    COMMAND cut -d. -f3
    COMMAND cut -d- -f1
    COMMAND tr -d '\n'
    OUTPUT_VARIABLE CPACK_PACKAGE_VERSION_PATCH)
endif(NOT DEFINED CPACK_PACKAGE_VERSION_PATCH)
if(NOT DEFINED CPACK_PACKAGE_RELEASE)
execute_process(
  COMMAND git describe --abbrev=0
  COMMAND cut -d- -f2
  COMMAND tr -d '\n'
  OUTPUT_VARIABLE CPACK_PACKAGE_RELEASE)
endif(NOT DEFINED CPACK_PACKAGE_RELEASE)
# Other stuff
set(CPACK_PACKAGE_DESCRIPTION "ARGoS-Kilobot (Kilobot plugin for ARGoS)
 A plugin to support the Kilobot robot (https://www.kilobotics.com/)
 into the ARGoS multi-robot simulator (http://www.argos-sim.info/).")
set(CPACK_PACKAGE_HOMEPAGE "http://github.com/ilpincy/argos3-kilobot/")
set(CPACK_PACKAGE_MAINTAINER "Carlo Pinciroli <ilpincy@gmail.com>")
set(CPACK_PACKAGE_NAME "argos3plugins_${ARGOS_BUILD_FOR}_kilobot")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Kilobot support for ARGoS")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/../doc/ARGoS_LICENSE.txt")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/../README.md")
set(CPACK_STRIP_FILES ON)
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${ARGOS_PROCESSOR_ARCH}-${CPACK_PACKAGE_RELEASE}")

#
# Configuration for the Debian generator
#
set(CPACK_DEBIAN_PACKAGE_DEPENDS "argos3_${ARGOS_BUILD_FOR} (>= 3.0.0)")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION ${CPACK_PACKAGE_DESCRIPTION})
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE ${CPACK_PACKAGE_HOMEPAGE})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER ${CPACK_PACKAGE_MAINTAINER})
set(CPACK_DEBIAN_PACKAGE_SECTION "contrib/science")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_BINARY_DIR}/postinst;")

#
# Configuration for the RPM generator
#
set(CPACK_RPM_PACKAGE_DESCRIPTION ${CPACK_PACKAGE_DESCRIPTION})
set(CPACK_RPM_PACKAGE_URL ${CPACK_PACKAGE_HOMEPAGE})
set(CPACK_RPM_PACKAGE_GROUP "Development/Tools")
set(CPACK_RPM_PACKAGE_LICENSE "MIT")
set(CPACK_RPM_PACKAGE_REQUIRES "argos3_${ARGOS_BUILD_FOR} >= 3.0.0")
set(CPACK_RPM_PACKAGE_URL ${CPACK_PACKAGE_HOMEPAGE})
set(CPACK_RPM_PACKAGE_RELEASE ${CPACK_PACKAGE_RELEASE})

#
# Time to call CPack
#
include(CPack)
