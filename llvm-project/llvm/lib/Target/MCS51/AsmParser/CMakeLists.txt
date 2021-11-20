add_llvm_component_library(LLVMMCS51AsmParser
  MCS51AsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  AVRDesc
  AVRInfo
  Support

  ADD_TO_COMPONENT
  MCS51
)
