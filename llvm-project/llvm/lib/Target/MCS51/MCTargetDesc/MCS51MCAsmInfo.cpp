//===-- MCS51MCAsmInfo.cpp - MCS51 asm properties -----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MCS51MCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "MCS51MCAsmInfo.h"

#include "llvm/ADT/Triple.h"

namespace llvm {

MCS51MCAsmInfo::MCS51MCAsmInfo(const Triple &TT, const MCTargetOptions &Options) {
  CodePointerSize = 2;
  CalleeSaveStackSlotSize = 2;
  CommentString = ";";
  PrivateGlobalPrefix = ".L";
  PrivateLabelPrefix = ".L";
  UsesELFSectionDirectiveForBSS = true;
  SupportsDebugInformation = true;
}

} // end of namespace llvm
