//===-- MCS51TargetStreamer.cpp - MCS51 Target Streamer Methods ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MCS51 specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "MCS51TargetStreamer.h"

#include "llvm/MC/MCContext.h"

namespace llvm {

MCS51TargetStreamer::MCS51TargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

MCS51TargetAsmStreamer::MCS51TargetAsmStreamer(MCStreamer &S)
    : MCS51TargetStreamer(S) {}

void MCS51TargetStreamer::finish() {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();

  MCSymbol *DoCopyData = Context.getOrCreateSymbol("__do_copy_data");
  MCSymbol *DoClearBss = Context.getOrCreateSymbol("__do_clear_bss");

  // FIXME: We can disable __do_copy_data if there are no static RAM variables.

  OS.emitRawComment(" Declaring this symbol tells the CRT that it should");
  OS.emitRawComment("copy all variables from program memory to RAM on startup");
  OS.emitSymbolAttribute(DoCopyData, MCSA_Global);

  OS.emitRawComment(" Declaring this symbol tells the CRT that it should");
  OS.emitRawComment("clear the zeroed data section on startup");
  OS.emitSymbolAttribute(DoClearBss, MCSA_Global);
}

} // end namespace llvm

