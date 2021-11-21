//===-- MCS51MCExpr.cpp - MCS51 specific MC expression classes ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCS51MCExpr.h"

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char * const Spelling;
  MCS51MCExpr::VariantKind VariantKind;
} ModifierNames[] = {
    {"lo8", MCS51MCExpr::VK_MCS51_LO8},       {"hi8", MCS51MCExpr::VK_MCS51_HI8},
    {"hh8", MCS51MCExpr::VK_MCS51_HH8}, // synonym with hlo8
    {"hlo8", MCS51MCExpr::VK_MCS51_HH8},      {"hhi8", MCS51MCExpr::VK_MCS51_HHI8},

    {"pm", MCS51MCExpr::VK_MCS51_PM},
    {"pm_lo8", MCS51MCExpr::VK_MCS51_PM_LO8}, {"pm_hi8", MCS51MCExpr::VK_MCS51_PM_HI8},
    {"pm_hh8", MCS51MCExpr::VK_MCS51_PM_HH8},

    {"lo8_gs", MCS51MCExpr::VK_MCS51_LO8_GS}, {"hi8_gs", MCS51MCExpr::VK_MCS51_HI8_GS},
    {"gs", MCS51MCExpr::VK_MCS51_GS},
};

} // end of anonymous namespace

const MCS51MCExpr *MCS51MCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) MCS51MCExpr(Kind, Expr, Negated);
}

void MCS51MCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_MCS51_None);

  if (isNegated())
    OS << '-';

  OS << getName() << '(';
  getSubExpr()->print(OS, MAI);
  OS << ')';
}

bool MCS51MCExpr::evaluateAsConstant(int64_t &Result) const {
  MCValue Value;

  bool isRelocatable =
      getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = evaluateAsInt64(Value.getConstant());
    return true;
  }

  return false;
}

bool MCS51MCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAsmLayout *Layout,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool isRelocatable = SubExpr->evaluateAsRelocatable(Value, Layout, Fixup);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Layout) return false;

    MCContext &Context = Layout->getAssembler().getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None)
      return false;
    if (Kind == VK_MCS51_PM) {
      Modifier = MCSymbolRefExpr::VK_X86_ABS8;
    }

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t MCS51MCExpr::evaluateAsInt64(int64_t Value) const {
  if (Negated)
    Value *= -1;

  switch (Kind) {
  case MCS51MCExpr::VK_MCS51_LO8:
    Value &= 0xff;
    break;
  case MCS51MCExpr::VK_MCS51_HI8:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case MCS51MCExpr::VK_MCS51_HH8:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case MCS51MCExpr::VK_MCS51_HHI8:
    Value &= 0xff000000;
    Value >>= 24;
    break;
  case MCS51MCExpr::VK_MCS51_PM_LO8:
  case MCS51MCExpr::VK_MCS51_LO8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff;
    break;
  case MCS51MCExpr::VK_MCS51_PM_HI8:
  case MCS51MCExpr::VK_MCS51_HI8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff00;
    Value >>= 8;
    break;
  case MCS51MCExpr::VK_MCS51_PM_HH8:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case MCS51MCExpr::VK_MCS51_PM:
  case MCS51MCExpr::VK_MCS51_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    break;

  case MCS51MCExpr::VK_MCS51_None:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value) & 0xff;
}

MCS51::Fixups MCS51MCExpr::getFixupKind() const {
  MCS51::Fixups Kind = MCS51::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_MCS51_LO8:
    Kind = isNegated() ? MCS51::fixup_lo8_ldi_neg : MCS51::fixup_lo8_ldi;
    break;
  case VK_MCS51_HI8:
    Kind = isNegated() ? MCS51::fixup_hi8_ldi_neg : MCS51::fixup_hi8_ldi;
    break;
  case VK_MCS51_HH8:
    Kind = isNegated() ? MCS51::fixup_hh8_ldi_neg : MCS51::fixup_hh8_ldi;
    break;
  case VK_MCS51_HHI8:
    Kind = isNegated() ? MCS51::fixup_ms8_ldi_neg : MCS51::fixup_ms8_ldi;
    break;

  case VK_MCS51_PM_LO8:
    Kind = isNegated() ? MCS51::fixup_lo8_ldi_pm_neg : MCS51::fixup_lo8_ldi_pm;
    break;
  case VK_MCS51_PM_HI8:
    Kind = isNegated() ? MCS51::fixup_hi8_ldi_pm_neg : MCS51::fixup_hi8_ldi_pm;
    break;
  case VK_MCS51_PM_HH8:
    Kind = isNegated() ? MCS51::fixup_hh8_ldi_pm_neg : MCS51::fixup_hh8_ldi_pm;
    break;
  case VK_MCS51_PM:
  case VK_MCS51_GS:
    Kind = MCS51::fixup_16_pm;
    break;
  case VK_MCS51_LO8_GS:
    Kind = MCS51::fixup_lo8_ldi_gs;
    break;
  case VK_MCS51_HI8_GS:
    Kind = MCS51::fixup_hi8_ldi_gs;
    break;

  case VK_MCS51_None:
    llvm_unreachable("Uninitialized expression");
  }

  return Kind;
}

void MCS51MCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

const char *MCS51MCExpr::getName() const {
  const auto &Modifier =
      llvm::find_if(ModifierNames, [this](ModifierEntry const &Mod) {
        return Mod.VariantKind == Kind;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->Spelling;
  }
  return nullptr;
}

MCS51MCExpr::VariantKind MCS51MCExpr::getKindByName(StringRef Name) {
  const auto &Modifier =
      llvm::find_if(ModifierNames, [&Name](ModifierEntry const &Mod) {
        return Mod.Spelling == Name;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }
  return VK_MCS51_None;
}

} // end of namespace llvm

