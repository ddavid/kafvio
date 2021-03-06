# - Try to find Pylon
# Once done this will define
# PYLON_FOUND - System has Pylon
# PYLON_INCLUDE_DIRS - The Pylon include directories
# PYLON_LIBRARIES - The libraries needed to use Pylon

set(DEBUG 0)

if( CMAKE_SIZEOF_VOID_P EQUAL 8)

	set( PYLON_LIBRARY "/opt/pylon5/lib64" )

else( CMAKE_SIZEOF_VOID_P EQUAL 8 )

    set( PYLON_LIBRARY "/opt/pylon5/lib32" )

endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

FIND_PATH(  PYLON_INCLUDE_DIR pylon/PylonBase.h
        PATHS
        /opt/pylon5/include
        "$ENV{PYLON_ROOT}/include"
        )

FIND_LIBRARY( PYLONBASE_LIBRARY
        NAMES
        pylonbase PylonBase_MD_VC100
        PATHS
        ${PYLON_LIBRARY}
        "$ENV{PYLON_ROOT}/lib64"
        )

FIND_LIBRARY( PYLON_UTILITY_LIBRARY
        NAMES
        pylonutility PylonUtility_MD_VC100
        PATHS
        ${PYLON_LIBRARY}
        "$ENV{PYLON_ROOT}/lib64"
        )

FIND_LIBRARY( PYLON_BOOTSTRAPPER_LIBRARY
        NAMES
        pylonbootstrapper PylonBootstrapper
        PATHS
        ${PYLON_LIBRARY}
        )

set( XERCES-C_LIBRARY "" )

FIND_LIBRARY( XERCES-C_LIBRARY
        NAMES
        Xerces-C_gcc40_v2_7 Xerces-C_MD_VC100_v2_7_1
        PATHS
        ${PYLON_LIBRARY}
        )

FIND_LIBRARY( PYLON_GCBASE_LIBRARY
        NAMES
        CGBase GCBase_gcc_v3_0_Basler_pylon_v5_0
        PATHS
        ${PYLON_LIBRARY}
        "$ENV{PYLON_ROOT}/lib64"
        )

FIND_LIBRARY( GENAPI_LIBRARY
        NAMES
        GenApi GenApi_gcc_v3_0_Basler_pylon_v5_0
        PATHS
        ${PYLON_LIBRARY}
        "$ENV{PYLON_ROOT}/lib64"
        )

if(DEBUG)

	message(STATUS "   pylon-include-dir: ${PYLON_INCLUDE_DIR}")
	message(STATUS "   pylonbase: ${PYLONBASE_LIBRARY}")
	message(STATUS "   pylonutlity: ${PYLON_UTILITY_LIBRARY}")
	message(STATUS "   pylonbootstrapper: ${PYLON_BOOTSTRAPPER_LIBRARY}")
	message(STATUS "   xerces-c: ${XERCES-C_LIBRARY}")
	message(STATUS "   GCBase: ${PYLON_GCBASE_LIBRARY}")
	message(STATUS "   GenApi: ${GENAPI_LIBRARY}")
endif()


if( NOT XERCES-C_LIBRARY)

    set(XERCES-C_LIBRARY "")

endif(NOT XERCES-C_LIBRARY)

if( NOT PYLON_BOOTSTRAPPER_LIBRARY)

    set(PYLON_BOOTSTRAPPER_LIBRARY "")

endif(NOT PYLON_BOOTSTRAPPER_LIBRARY)

if( NOT PYLON_GCBASE_LIBRARY)

    set(PYLON_GCBASE_LIBRARY "")

endif(NOT PYLON_GCBASE_LIBRARY)

set(PYLON_LIBRARIES  ${PYLONBASE_LIBRARY} ${XERCES-C_LIBRARY} ${PYLON_UTILITY_LIBRARY} ${PYLON_BOOTSTRAPPER_LIBRARY} ${PYLON_GCBASE_LIBRARY} ${GENAPI_LIBRARY})

set(PYLON_INCLUDE_DIRS ${PYLON_INCLUDE_DIR})

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(PYLON DEFAULT_MSG
        PYLON_INCLUDE_DIR
        PYLON_LIBRARY)

message(STATUS "Pylon, found libs:")

foreach(l ${PYLON_LIBRARIES})

    message(STATUS "    ${l}")

endforeach(l)

message(STATUS "Pylon, found header: ${PYLON_INCLUDE_DIRS}")

mark_as_advanced(PYLON_INCLUDE_DIR PYLON_LIBRARIES)