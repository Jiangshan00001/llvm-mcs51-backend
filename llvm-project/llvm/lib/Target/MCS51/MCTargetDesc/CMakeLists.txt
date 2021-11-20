add_llvm_component_library(LLVMMCS51Desc
  MCS51AsmBackend.cpp
  MCS51ELFObjectWriter.cpp
  MCS51ELFStreamer.cpp
  MCS51InstPrinter.cpp
  MCS51MCAsmInfo.cpp
  MCS51MCCodeEmitter.cpp
  MCS51MCELFStreamer.cpp
  MCS51MCExpr.cpp
  MCS51MCTargetDesc.cpp
  MCS51TargetStreamer.cpp

  LINK_COMPONENTS
  MC
  MCS51Info
  Support

  ADD_TO_COMPONENT
  MCS51
)

add_dependencies(LLVMMCS51Desc MCS51CommonTableGen)

