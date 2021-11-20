include_directories( ${CMAKE_CURRENT_BINARY_DIR}/..
                     ${CMAKE_CURRENT_SOURCE_DIR}/.. )

add_llvm_component_library(LLVMMCS51Info
  MCS51TargetInfo.cpp

  LINK_COMPONENTS
  Support

  ADD_TO_COMPONENT
  MCS51
)

add_dependencies(LLVMMCS51Info MCS51CommonTableGen)

