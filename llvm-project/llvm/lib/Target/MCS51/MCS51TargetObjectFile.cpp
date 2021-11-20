//===-- MCS51TargetObjectFile.cpp - MCS51 Object Files ------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCS51TargetObjectFile.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"

#include "MCS51.h"

namespace llvm {
void MCS51TargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM) {
  Base::Initialize(Ctx, TM);
  ProgmemDataSection =
      Ctx.getELFSection(".progmem.data", ELF::SHT_PROGBITS, ELF::SHF_ALLOC);
}

MCSection *
MCS51TargetObjectFile::SelectSectionForGlobal(const GlobalObject *GO,
                                            SectionKind Kind,
                                            const TargetMachine &TM) const {
  // Global values in flash memory are placed in the progmem.data section
  // unless they already have a user assigned section.
  if (MCS51::isProgramMemoryAddress(GO) && !GO->hasSection() && Kind.isReadOnly())
    return ProgmemDataSection;

  // Otherwise, we work the same way as ELF.
  return Base::SelectSectionForGlobal(GO, Kind, TM);
}
} // end of namespace llvm

