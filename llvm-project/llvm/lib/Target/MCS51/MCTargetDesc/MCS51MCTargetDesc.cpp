//===-- MCS51MCTargetDesc.cpp - MCS51 Target Descriptions ---------------------===//
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

#include "MCS51ELFStreamer.h"
#include "MCS51InstPrinter.h"
#include "MCS51MCAsmInfo.h"
#include "MCS51MCELFStreamer.h"
#include "MCS51MCTargetDesc.h"
#include "MCS51TargetStreamer.h"
#include "TargetInfo/MCS51TargetInfo.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "MCS51GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "MCS51GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "MCS51GenRegisterInfo.inc"

using namespace llvm;

MCInstrInfo *llvm::createMCS51MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitMCS51MCInstrInfo(X);

  return X;
}

static MCRegisterInfo *createMCS51MCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitMCS51MCRegisterInfo(X, 0);

  return X;
}

static MCSubtargetInfo *createMCS51MCSubtargetInfo(const Triple &TT,
                                                 StringRef CPU, StringRef FS) {
  return createMCS51MCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCInstPrinter *createMCS51MCInstPrinter(const Triple &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI) {
  if (SyntaxVariant == 0) {
    return new MCS51InstPrinter(MAI, MII, MRI);
  }

  return nullptr;
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  return createELFStreamer(Context, std::move(MAB), std::move(OW),
                           std::move(Emitter), RelaxAll);
}

static MCTargetStreamer *
createMCS51ObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new MCS51ELFStreamer(S, STI);
}

static MCTargetStreamer *createMCAsmTargetStreamer(MCStreamer &S,
                                                   formatted_raw_ostream &OS,
                                                   MCInstPrinter *InstPrint,
                                                   bool isVerboseAsm) {
  return new MCS51TargetAsmStreamer(S);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMCS51TargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<MCS51MCAsmInfo> X(getTheMCS51Target());

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(getTheMCS51Target(), createMCS51MCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(getTheMCS51Target(), createMCS51MCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(getTheMCS51Target(),
                                          createMCS51MCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(getTheMCS51Target(),
                                        createMCS51MCInstPrinter);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(getTheMCS51Target(), createMCS51MCCodeEmitter);

  // Register the obj streamer
  TargetRegistry::RegisterELFStreamer(getTheMCS51Target(), createMCStreamer);

  // Register the obj target streamer.
  TargetRegistry::RegisterObjectTargetStreamer(getTheMCS51Target(),
                                               createMCS51ObjectTargetStreamer);

  // Register the asm target streamer.
  TargetRegistry::RegisterAsmTargetStreamer(getTheMCS51Target(),
                                            createMCAsmTargetStreamer);

  // Register the asm backend (as little endian).
  TargetRegistry::RegisterMCAsmBackend(getTheMCS51Target(), createMCS51AsmBackend);
}

