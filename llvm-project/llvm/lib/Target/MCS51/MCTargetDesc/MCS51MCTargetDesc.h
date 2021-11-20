//===-- MCS51MCTargetDesc.h - MCS51 Target Descriptions -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MCS51 specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCS51_MCTARGET_DESC_H
#define LLVM_MCS51_MCTARGET_DESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {

class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class Target;

MCInstrInfo *createMCS51MCInstrInfo();

/// Creates a machine code emitter for MCS51.
MCCodeEmitter *createMCS51MCCodeEmitter(const MCInstrInfo &MCII,
                                      const MCRegisterInfo &MRI,
                                      MCContext &Ctx);

/// Creates an assembly backend for MCS51.
MCAsmBackend *createMCS51AsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO);

/// Creates an ELF object writer for MCS51.
std::unique_ptr<MCObjectTargetWriter> createMCS51ELFObjectWriter(uint8_t OSABI);

} // end namespace llvm

#define GET_REGINFO_ENUM
#include "MCS51GenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#include "MCS51GenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "MCS51GenSubtargetInfo.inc"

#endif // LLVM_MCS51_MCTARGET_DESC_H
