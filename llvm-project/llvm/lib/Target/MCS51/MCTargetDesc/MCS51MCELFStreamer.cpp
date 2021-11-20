//===--------- MCS51MCELFStreamer.cpp - MCS51 subclass of MCELFStreamer -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a stub that parses a MCInst bundle and passes the
// instructions on to the real streamer.
//
//===----------------------------------------------------------------------===//
#include "MCTargetDesc/MCS51MCELFStreamer.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCObjectWriter.h"

#define DEBUG_TYPE "MCS51mcelfstreamer"

using namespace llvm;

void MCS51MCELFStreamer::emitValueForModiferKind(
    const MCSymbol *Sym, unsigned SizeInBytes, SMLoc Loc,
    MCS51MCExpr::VariantKind ModifierKind) {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_MCS51_NONE;
  if (ModifierKind == MCS51MCExpr::VK_MCS51_None) {
    Kind = MCSymbolRefExpr::VK_MCS51_DIFF8;
    if (SizeInBytes == SIZE_LONG)
      Kind = MCSymbolRefExpr::VK_MCS51_DIFF32;
    else if (SizeInBytes == SIZE_WORD)
      Kind = MCSymbolRefExpr::VK_MCS51_DIFF16;
  } else if (ModifierKind == MCS51MCExpr::VK_MCS51_LO8)
    Kind = MCSymbolRefExpr::VK_MCS51_LO8;
  else if (ModifierKind == MCS51MCExpr::VK_MCS51_HI8)
    Kind = MCSymbolRefExpr::VK_MCS51_HI8;
  else if (ModifierKind == MCS51MCExpr::VK_MCS51_HH8)
    Kind = MCSymbolRefExpr::VK_MCS51_HLO8;
  MCELFStreamer::emitValue(MCSymbolRefExpr::create(Sym, Kind, getContext()),
                           SizeInBytes, Loc);
}

namespace llvm {
MCStreamer *createMCS51ELFStreamer(Triple const &TT, MCContext &Context,
                                 std::unique_ptr<MCAsmBackend> MAB,
                                 std::unique_ptr<MCObjectWriter> OW,
                                 std::unique_ptr<MCCodeEmitter> CE) {
  return new MCS51MCELFStreamer(Context, std::move(MAB), std::move(OW),
                              std::move(CE));
}

} // end namespace llvm
