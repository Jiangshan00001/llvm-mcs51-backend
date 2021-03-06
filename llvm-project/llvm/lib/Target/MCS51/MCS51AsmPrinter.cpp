//===-- MCS51AsmPrinter.cpp - MCS51 LLVM assembly writer ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format MCS51 assembly language.
//
//===----------------------------------------------------------------------===//

#include "MCS51.h"
#include "MCS51MCInstLower.h"
#include "MCS51Subtarget.h"
#include "MCTargetDesc/MCS51InstPrinter.h"
#include "MCTargetDesc/MCS51MCExpr.h"
#include "TargetInfo/MCS51TargetInfo.h"

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mcs51-asm-printer"

namespace llvm {

/// An MCS51 assembly code printer.
class MCS51AsmPrinter : public AsmPrinter {
public:
  MCS51AsmPrinter(TargetMachine &TM,
                std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), MRI(*TM.getMCRegisterInfo()) { }

  StringRef getPassName() const override { return "MCS51 Assembly Printer"; }

  void printOperand(const MachineInstr *MI, unsigned OpNo, raw_ostream &O);

  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                       const char *ExtraCode, raw_ostream &O) override;

  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNum,
                             const char *ExtraCode, raw_ostream &O) override;

  void emitInstruction(const MachineInstr *MI) override;

  const MCExpr *lowerConstant(const Constant *CV) override;

private:
  const MCRegisterInfo &MRI;
};

void MCS51AsmPrinter::printOperand(const MachineInstr *MI, unsigned OpNo,
                                 raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(OpNo);

  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << MCS51InstPrinter::getPrettyRegisterName(MO.getReg(), MRI);
    break;
  case MachineOperand::MO_Immediate:
    O << MO.getImm();
    break;
  case MachineOperand::MO_GlobalAddress:
    O << getSymbol(MO.getGlobal());
    break;
  case MachineOperand::MO_ExternalSymbol:
    O << *GetExternalSymbolSymbol(MO.getSymbolName());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    O << *MO.getMBB()->getSymbol();
    break;
  default:
    llvm_unreachable("Not implemented yet!");
  }
}

bool MCS51AsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                                    const char *ExtraCode, raw_ostream &O) {
  // Default asm printer can only deal with some extra codes,
  // so try it first.
  bool Error = AsmPrinter::PrintAsmOperand(MI, OpNum, ExtraCode, O);

  if (Error && ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0)
      return true; // Unknown modifier.

    if (ExtraCode[0] >= 'A' && ExtraCode[0] <= 'Z') {
      const MachineOperand &RegOp = MI->getOperand(OpNum);

      assert(RegOp.isReg() && "Operand must be a register when you're"
                              "using 'A'..'Z' operand extracodes.");
      Register Reg = RegOp.getReg();

      unsigned ByteNumber = ExtraCode[0] - 'A';

      unsigned OpFlags = MI->getOperand(OpNum - 1).getImm();
      unsigned NumOpRegs = InlineAsm::getNumOperandRegisters(OpFlags);
      (void)NumOpRegs;

      const MCS51Subtarget &STI = MF->getSubtarget<MCS51Subtarget>();
      const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

      const TargetRegisterClass *RC = TRI.getMinimalPhysRegClass(Reg);
      unsigned BytesPerReg = TRI.getRegSizeInBits(*RC) / 8;
      assert(BytesPerReg <= 2 && "Only 8 and 16 bit regs are supported.");

      unsigned RegIdx = ByteNumber / BytesPerReg;
      assert(RegIdx < NumOpRegs && "Multibyte index out of range.");

      Reg = MI->getOperand(OpNum + RegIdx).getReg();

      if (BytesPerReg == 2) {
        Reg = TRI.getSubReg(Reg, ByteNumber % BytesPerReg ? MCS51::sub_hi
                                                          : MCS51::sub_lo);
      }

      O << MCS51InstPrinter::getPrettyRegisterName(Reg, MRI);
      return false;
    }
  }

  if (Error)
    printOperand(MI, OpNum, O);

  return false;
}

bool MCS51AsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                          unsigned OpNum, const char *ExtraCode,
                                          raw_ostream &O) {
  if (ExtraCode && ExtraCode[0]) {
    llvm_unreachable("This branch is not implemented yet");
  }

  const MachineOperand &MO = MI->getOperand(OpNum);
  (void)MO;
  assert(MO.isReg() && "Unexpected inline asm memory operand");

  // TODO: We should be able to look up the alternative name for
  // the register if it's given.
  // TableGen doesn't expose a way of getting retrieving names
  // for registers.
  if (MI->getOperand(OpNum).getReg() == MCS51::R31R30) {
    O << "Z";
  } else {
    assert(MI->getOperand(OpNum).getReg() == MCS51::R29R28 &&
           "Wrong register class for memory operand.");
    O << "Y";
  }

  // If NumOpRegs == 2, then we assume it is product of a FrameIndex expansion
  // and the second operand is an Imm.
  unsigned OpFlags = MI->getOperand(OpNum - 1).getImm();
  unsigned NumOpRegs = InlineAsm::getNumOperandRegisters(OpFlags);

  if (NumOpRegs == 2) {
    O << '+' << MI->getOperand(OpNum + 1).getImm();
  }

  return false;
}

void MCS51AsmPrinter::emitInstruction(const MachineInstr *MI) {
  MCS51MCInstLower MCInstLowering(OutContext, *this);

  MCInst I;
  MCInstLowering.lowerInstruction(*MI, I);
  EmitToStreamer(*OutStreamer, I);
}

const MCExpr *MCS51AsmPrinter::lowerConstant(const Constant *CV) {
  MCContext &Ctx = OutContext;

  if (const GlobalValue *GV = dyn_cast<GlobalValue>(CV)) {
    bool IsProgMem = GV->getAddressSpace() == MCS51::ProgramMemory;
    if (IsProgMem) {
      const MCExpr *Expr = MCSymbolRefExpr::create(getSymbol(GV), Ctx);
      return MCS51MCExpr::create(MCS51MCExpr::VK_MCS51_PM, Expr, false, Ctx);
    }
  }

  return AsmPrinter::lowerConstant(CV);
}

} // end of namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMCS51AsmPrinter() {
  llvm::RegisterAsmPrinter<llvm::MCS51AsmPrinter> X(llvm::getTheMCS51Target());
}

