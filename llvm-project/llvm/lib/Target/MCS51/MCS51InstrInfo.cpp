//===-- MCS51InstrInfo.cpp - MCS51 Instruction Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MCS51 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MCS51InstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#include "MCS51.h"
#include "MCS51MachineFunctionInfo.h"
#include "MCS51RegisterInfo.h"
#include "MCS51TargetMachine.h"
#include "MCTargetDesc/MCS51MCTargetDesc.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "MCS51GenInstrInfo.inc"

namespace llvm {

MCS51InstrInfo::MCS51InstrInfo()
    : MCS51GenInstrInfo(MCS51::ADJCALLSTACKDOWN, MCS51::ADJCALLSTACKUP), RI() {}

void MCS51InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc) const {
  const MCS51Subtarget &STI = MBB.getParent()->getSubtarget<MCS51Subtarget>();
  const MCS51RegisterInfo &TRI = *STI.getRegisterInfo();
  unsigned Opc;

  // Not all MCS51 devices support the 16-bit `MOVW` instruction.
  if (MCS51::DREGSRegClass.contains(DestReg, SrcReg)) {
    if (STI.hasMOVW() && MCS51::DREGSMOVWRegClass.contains(DestReg, SrcReg)) {
      BuildMI(MBB, MI, DL, get(MCS51::MOVWRdRr), DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
    } else {
      Register DestLo, DestHi, SrcLo, SrcHi;

      TRI.splitReg(DestReg, DestLo, DestHi);
      TRI.splitReg(SrcReg,  SrcLo,  SrcHi);

      // Copy each individual register with the `MOV` instruction.
      BuildMI(MBB, MI, DL, get(MCS51::MOVRdRr), DestLo)
        .addReg(SrcLo, getKillRegState(KillSrc));
      BuildMI(MBB, MI, DL, get(MCS51::MOVRdRr), DestHi)
        .addReg(SrcHi, getKillRegState(KillSrc));
    }
  } else {
    if (MCS51::GPR8RegClass.contains(DestReg, SrcReg)) {
      Opc = MCS51::MOVRdRr;
    } else if (SrcReg == MCS51::SP && MCS51::DREGSRegClass.contains(DestReg)) {
      Opc = MCS51::SPREAD;
    } else if (DestReg == MCS51::SP && MCS51::DREGSRegClass.contains(SrcReg)) {
      Opc = MCS51::SPWRITE;
    } else {
      llvm_unreachable("Impossible reg-to-reg copy");
    }

    BuildMI(MBB, MI, DL, get(Opc), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
  }
}

unsigned MCS51InstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case MCS51::LDDRdPtrQ:
  case MCS51::LDDWRdYQ: { //:FIXME: remove this once PR13375 gets fixed
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

unsigned MCS51InstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case MCS51::STDPtrQRr:
  case MCS51::STDWPtrQRr: {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

void MCS51InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       Register SrcReg, bool isKill,
                                       int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
  MachineFunction &MF = *MBB.getParent();
  MCS51MachineFunctionInfo *AFI = MF.getInfo<MCS51MachineFunctionInfo>();

  AFI->setHasSpills(true);

  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = MCS51::STDPtrQRr;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = MCS51::STDWPtrQRr;
  } else {
    llvm_unreachable("Cannot store this register into a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode))
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addReg(SrcReg, getKillRegState(isKill))
      .addMemOperand(MMO);
}

void MCS51InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  MachineFunction &MF = *MBB.getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = MCS51::LDDRdPtrQ;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    // Opcode = MCS51::LDDWRdPtrQ;
    //:FIXME: remove this once PR13375 gets fixed
    Opcode = MCS51::LDDWRdYQ;
  } else {
    llvm_unreachable("Cannot load this register from a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode), DestReg)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);
}

const MCInstrDesc &MCS51InstrInfo::getBrCond(MCS51CC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case MCS51CC::COND_EQ:
    return get(MCS51::BREQk);
  case MCS51CC::COND_NE:
    return get(MCS51::BRNEk);
  case MCS51CC::COND_GE:
    return get(MCS51::BRGEk);
  case MCS51CC::COND_LT:
    return get(MCS51::BRLTk);
  case MCS51CC::COND_SH:
    return get(MCS51::BRSHk);
  case MCS51CC::COND_LO:
    return get(MCS51::BRLOk);
  case MCS51CC::COND_MI:
    return get(MCS51::BRMIk);
  case MCS51CC::COND_PL:
    return get(MCS51::BRPLk);
  }
}

MCS51CC::CondCodes MCS51InstrInfo::getCondFromBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:
    return MCS51CC::COND_INVALID;
  case MCS51::BREQk:
    return MCS51CC::COND_EQ;
  case MCS51::BRNEk:
    return MCS51CC::COND_NE;
  case MCS51::BRSHk:
    return MCS51CC::COND_SH;
  case MCS51::BRLOk:
    return MCS51CC::COND_LO;
  case MCS51::BRMIk:
    return MCS51CC::COND_MI;
  case MCS51::BRPLk:
    return MCS51CC::COND_PL;
  case MCS51::BRGEk:
    return MCS51CC::COND_GE;
  case MCS51::BRLTk:
    return MCS51CC::COND_LT;
  }
}

MCS51CC::CondCodes MCS51InstrInfo::getOppositeCondition(MCS51CC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Invalid condition!");
  case MCS51CC::COND_EQ:
    return MCS51CC::COND_NE;
  case MCS51CC::COND_NE:
    return MCS51CC::COND_EQ;
  case MCS51CC::COND_SH:
    return MCS51CC::COND_LO;
  case MCS51CC::COND_LO:
    return MCS51CC::COND_SH;
  case MCS51CC::COND_GE:
    return MCS51CC::COND_LT;
  case MCS51CC::COND_LT:
    return MCS51CC::COND_GE;
  case MCS51CC::COND_MI:
    return MCS51CC::COND_PL;
  case MCS51CC::COND_PL:
    return MCS51CC::COND_MI;
  }
}

bool MCS51InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end();
  MachineBasicBlock::iterator UnCondBrIter = MBB.end();

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }

    // Working from the bottom, when we see a non-terminator
    // instruction, we're done.
    if (!isUnpredicatedTerminator(*I)) {
      break;
    }

    // A terminator that isn't a branch can't easily be handled
    // by this analysis.
    if (!I->getDesc().isBranch()) {
      return true;
    }

    // Handle unconditional branches.
    //:TODO: add here jmp
    if (I->getOpcode() == MCS51::RJMPk) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a JMP, delete them.
      while (std::next(I) != MBB.end()) {
        std::next(I)->eraseFromParent();
      }

      Cond.clear();
      FBB = 0;

      // Delete the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = 0;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = MBB.end();
        continue;
      }

      // TBB is used to indicate the unconditinal destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    MCS51CC::CondCodes BranchCode = getCondFromBranchOpc(I->getOpcode());
    if (BranchCode == MCS51CC::COND_INVALID) {
      return true; // Can't handle indirect branch.
    }

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(0).getMBB();
      if (AllowModify && UnCondBrIter != MBB.end() &&
          MBB.isLayoutSuccessor(TargetBB)) {
        // If we can modify the code and it ends in something like:
        //
        //     jCC L1
        //     jmp L2
        //   L1:
        //     ...
        //   L2:
        //
        // Then we can change this to:
        //
        //     jnCC L2
        //   L1:
        //     ...
        //   L2:
        //
        // Which is a bit more efficient.
        // We conditionally jump to the fall-through block.
        BranchCode = getOppositeCondition(BranchCode);
        unsigned JNCC = getBrCond(BranchCode).getOpcode();
        MachineBasicBlock::iterator OldInst = I;

        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(JNCC))
            .addMBB(UnCondBrIter->getOperand(0).getMBB());
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(MCS51::RJMPk))
            .addMBB(TargetBB);

        OldInst->eraseFromParent();
        UnCondBrIter->eraseFromParent();

        // Restart the analysis.
        UnCondBrIter = MBB.end();
        I = MBB.end();
        continue;
      }

      FBB = TBB;
      TBB = I->getOperand(0).getMBB();
      Cond.push_back(MachineOperand::CreateImm(BranchCode));
      continue;
    }

    // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination.
    assert(Cond.size() == 1);
    assert(TBB);

    // Only handle the case where all conditional branches branch to
    // the same destination.
    if (TBB != I->getOperand(0).getMBB()) {
      return true;
    }

    MCS51CC::CondCodes OldBranchCode = (MCS51CC::CondCodes)Cond[0].getImm();
    // If the conditions are the same, we can leave them alone.
    if (OldBranchCode == BranchCode) {
      continue;
    }

    return true;
  }

  return false;
}

unsigned MCS51InstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL,
                                    int *BytesAdded) const {
  if (BytesAdded) *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "MCS51 branch conditions have one component!");

  if (Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    auto &MI = *BuildMI(&MBB, DL, get(MCS51::RJMPk)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Conditional branch.
  unsigned Count = 0;
  MCS51CC::CondCodes CC = (MCS51CC::CondCodes)Cond[0].getImm();
  auto &CondMI = *BuildMI(&MBB, DL, getBrCond(CC)).addMBB(TBB);

  if (BytesAdded) *BytesAdded += getInstSizeInBytes(CondMI);
  ++Count;

  if (FBB) {
    // Two-way Conditional branch. Insert the second branch.
    auto &MI = *BuildMI(&MBB, DL, get(MCS51::RJMPk)).addMBB(FBB);
    if (BytesAdded) *BytesAdded += getInstSizeInBytes(MI);
    ++Count;
  }

  return Count;
}

unsigned MCS51InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  if (BytesRemoved) *BytesRemoved = 0;

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }
    //:TODO: add here the missing jmp instructions once they are implemented
    // like jmp, {e}ijmp, and other cond branches, ...
    if (I->getOpcode() != MCS51::RJMPk &&
        getCondFromBranchOpc(I->getOpcode()) == MCS51CC::COND_INVALID) {
      break;
    }

    // Remove the branch.
    if (BytesRemoved) *BytesRemoved += getInstSizeInBytes(*I);
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

bool MCS51InstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid MCS51 branch condition!");

  MCS51CC::CondCodes CC = static_cast<MCS51CC::CondCodes>(Cond[0].getImm());
  Cond[0].setImm(getOppositeCondition(CC));

  return false;
}

unsigned MCS51InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
  // A regular instruction
  default: {
    const MCInstrDesc &Desc = get(Opcode);
    return Desc.getSize();
  }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const MCS51TargetMachine &TM = static_cast<const MCS51TargetMachine&>(MF.getTarget());
    const MCS51Subtarget &STI = MF.getSubtarget<MCS51Subtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();

    return TII.getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                                  *TM.getMCAsmInfo());
  }
  }
}

MachineBasicBlock *
MCS51InstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("unexpected opcode!");
  case MCS51::JMPk:
  case MCS51::CALLk:
  case MCS51::RCALLk:
  case MCS51::RJMPk:
  case MCS51::BREQk:
  case MCS51::BRNEk:
  case MCS51::BRSHk:
  case MCS51::BRLOk:
  case MCS51::BRMIk:
  case MCS51::BRPLk:
  case MCS51::BRGEk:
  case MCS51::BRLTk:
    return MI.getOperand(0).getMBB();
  case MCS51::BRBSsk:
  case MCS51::BRBCsk:
    return MI.getOperand(1).getMBB();
  case MCS51::SBRCRrB:
  case MCS51::SBRSRrB:
  case MCS51::SBICAb:
  case MCS51::SBISAb:
    llvm_unreachable("unimplemented branch instructions");
  }
}

bool MCS51InstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                         int64_t BrOffset) const {

  switch (BranchOp) {
  default:
    llvm_unreachable("unexpected opcode!");
  case MCS51::JMPk:
  case MCS51::CALLk:
    return true;
  case MCS51::RCALLk:
  case MCS51::RJMPk:
    return isIntN(13, BrOffset);
  case MCS51::BRBSsk:
  case MCS51::BRBCsk:
  case MCS51::BREQk:
  case MCS51::BRNEk:
  case MCS51::BRSHk:
  case MCS51::BRLOk:
  case MCS51::BRMIk:
  case MCS51::BRPLk:
  case MCS51::BRGEk:
  case MCS51::BRLTk:
    return isIntN(7, BrOffset);
  }
}

unsigned MCS51InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                            MachineBasicBlock &NewDestBB,
                                            const DebugLoc &DL,
                                            int64_t BrOffset,
                                            RegScavenger *RS) const {
    // This method inserts a *direct* branch (JMP), despite its name.
    // LLVM calls this method to fixup unconditional branches; it never calls
    // insertBranch or some hypothetical "insertDirectBranch".
    // See lib/CodeGen/RegisterRelaxation.cpp for details.
    // We end up here when a jump is too long for a RJMP instruction.
    auto &MI = *BuildMI(&MBB, DL, get(MCS51::JMPk)).addMBB(&NewDestBB);

    return getInstSizeInBytes(MI);
}

} // end of namespace llvm

