//===-- MCS51ELFObjectWriter.cpp - MCS51 ELF Writer ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MCS51FixupKinds.h"
#include "MCTargetDesc/MCS51MCExpr.h"
#include "MCTargetDesc/MCS51MCTargetDesc.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes MCS51 machine code into an ELF32 object file.
class MCS51ELFObjectWriter : public MCELFObjectTargetWriter {
public:
  MCS51ELFObjectWriter(uint8_t OSABI);

  virtual ~MCS51ELFObjectWriter() {}

  unsigned getRelocType(MCContext &Ctx,
                        const MCValue &Target,
                        const MCFixup &Fixup,
                        bool IsPCRel) const override;
};

MCS51ELFObjectWriter::MCS51ELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_IAMCU, true) {}

unsigned MCS51ELFObjectWriter::getRelocType(MCContext &Ctx,
                                          const MCValue &Target,
                                          const MCFixup &Fixup,
                                          bool IsPCRel) const {
    return ELF::EM_IAMCU;
#if 0
  MCSymbolRefExpr::VariantKind Modifier = Target.getAccessVariant();
  switch ((unsigned) Fixup.getKind()) {
  case FK_Data_1:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_MCS51_8;
    case MCSymbolRefExpr::VK_MCS51_DIFF8:
      return ELF::R_MCS51_DIFF8;
    case MCSymbolRefExpr::VK_MCS51_LO8:
      return ELF::R_MCS51_8_LO8;
    case MCSymbolRefExpr::VK_MCS51_HI8:
      return ELF::R_MCS51_8_HI8;
    case MCSymbolRefExpr::VK_MCS51_HLO8:
      return ELF::R_MCS51_8_HLO8;
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_MCS51_32;
    case MCSymbolRefExpr::VK_MCS51_DIFF32:
      return ELF::R_MCS51_DIFF32;
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_MCS51_16;
    case MCSymbolRefExpr::VK_MCS51_NONE:
    case MCSymbolRefExpr::VK_MCS51_PM:
      return ELF::R_MCS51_16_PM;
    case MCSymbolRefExpr::VK_MCS51_DIFF16:
      return ELF::R_MCS51_DIFF16;
    }
  case MCS51::fixup_32:
    return ELF::R_MCS51_32;
  case MCS51::fixup_7_pcrel:
    return ELF::R_MCS51_7_PCREL;
  case MCS51::fixup_13_pcrel:
    return ELF::R_MCS51_13_PCREL;
  case MCS51::fixup_16:
    return ELF::R_MCS51_16;
  case MCS51::fixup_16_pm:
    return ELF::R_MCS51_16_PM;
  case MCS51::fixup_lo8_ldi:
    return ELF::R_MCS51_LO8_LDI;
  case MCS51::fixup_hi8_ldi:
    return ELF::R_MCS51_HI8_LDI;
  case MCS51::fixup_hh8_ldi:
    return ELF::R_MCS51_HH8_LDI;
  case MCS51::fixup_lo8_ldi_neg:
    return ELF::R_MCS51_LO8_LDI_NEG;
  case MCS51::fixup_hi8_ldi_neg:
    return ELF::R_MCS51_HI8_LDI_NEG;
  case MCS51::fixup_hh8_ldi_neg:
    return ELF::R_MCS51_HH8_LDI_NEG;
  case MCS51::fixup_lo8_ldi_pm:
    return ELF::R_MCS51_LO8_LDI_PM;
  case MCS51::fixup_hi8_ldi_pm:
    return ELF::R_MCS51_HI8_LDI_PM;
  case MCS51::fixup_hh8_ldi_pm:
    return ELF::R_MCS51_HH8_LDI_PM;
  case MCS51::fixup_lo8_ldi_pm_neg:
    return ELF::R_MCS51_LO8_LDI_PM_NEG;
  case MCS51::fixup_hi8_ldi_pm_neg:
    return ELF::R_MCS51_HI8_LDI_PM_NEG;
  case MCS51::fixup_hh8_ldi_pm_neg:
    return ELF::R_MCS51_HH8_LDI_PM_NEG;
  case MCS51::fixup_call:
    return ELF::R_MCS51_CALL;
  case MCS51::fixup_ldi:
    return ELF::R_MCS51_LDI;
  case MCS51::fixup_6:
    return ELF::R_MCS51_6;
  case MCS51::fixup_6_adiw:
    return ELF::R_MCS51_6_ADIW;
  case MCS51::fixup_ms8_ldi:
    return ELF::R_MCS51_MS8_LDI;
  case MCS51::fixup_ms8_ldi_neg:
    return ELF::R_MCS51_MS8_LDI_NEG;
  case MCS51::fixup_lo8_ldi_gs:
    return ELF::R_MCS51_LO8_LDI_GS;
  case MCS51::fixup_hi8_ldi_gs:
    return ELF::R_MCS51_HI8_LDI_GS;
  case MCS51::fixup_8:
    return ELF::R_MCS51_8;
  case MCS51::fixup_8_lo8:
    return ELF::R_MCS51_8_LO8;
  case MCS51::fixup_8_hi8:
    return ELF::R_MCS51_8_HI8;
  case MCS51::fixup_8_hlo8:
    return ELF::R_MCS51_8_HLO8;
  case MCS51::fixup_diff8:
    return ELF::R_MCS51_DIFF8;
  case MCS51::fixup_diff16:
    return ELF::R_MCS51_DIFF16;
  case MCS51::fixup_diff32:
    return ELF::R_MCS51_DIFF32;
  case MCS51::fixup_lds_sts_16:
    return ELF::R_MCS51_LDS_STS_16;
  case MCS51::fixup_port6:
    return ELF::R_MCS51_PORT6;
  case MCS51::fixup_port5:
    return ELF::R_MCS51_PORT5;
  default:
    llvm_unreachable("invalid fixup kind!");
  }
#endif
}

std::unique_ptr<MCObjectTargetWriter> createMCS51ELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<MCS51ELFObjectWriter>(OSABI);
}

} // end of namespace llvm

