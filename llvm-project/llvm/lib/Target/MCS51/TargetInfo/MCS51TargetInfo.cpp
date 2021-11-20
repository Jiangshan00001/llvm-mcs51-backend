//===-- MCS51TargetInfo.cpp - MCS51 Target Implementation ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/MCS51TargetInfo.h"
#include "llvm/Support/TargetRegistry.h"
namespace llvm {
Target &getTheMCS51Target() {
  static Target TheMCS51Target;
  return TheMCS51Target;
}
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMCS51TargetInfo() {
  llvm::RegisterTarget<llvm::Triple::MCS51> X(llvm::getTheMCS51Target(), "MCS51",
                                            "Atmel MCS51 Microcontroller", "MCS51");
}

