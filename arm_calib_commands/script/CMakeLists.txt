cmake_minimum_required(VERSION 3.15...3.19)
project(pyCommon)
FILE( GLOB COMMON_PY [^.]*.py)
if (WIN32)
  INSTALL(FILES  ${GLOB}
    DESTINATION "c:/robnux/lib/site-packages"
  )
else()
  INSTALL(FILES  ${COMMON_PY}
    DESTINATION "/opt/robnux/lib/python3/site-packages"
  )
endif()
