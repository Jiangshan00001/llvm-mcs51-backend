//===-- MCS51MCAsmInfo.h - MCS51 asm properties ---------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the MCS51MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCS51_ASM_INFO_H
#define LLVM_MCS51_ASM_INFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {

class Triple;

/// Specifies the format of MCS51 assembly files.
class MCS51MCAsmInfo : public MCAsmInfo {
public:
  explicit MCS51MCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
};

} // end namespace llvm

#endif // LLVM_MCS51_ASM_INFO_H
