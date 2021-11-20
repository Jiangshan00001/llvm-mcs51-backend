//===-- MCS51ExpandPseudoInsts.cpp - Expand pseudo instructions -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "MCS51.h"
#include "MCS51InstrInfo.h"
#include "MCS51TargetMachine.h"
#include "MCTargetDesc/MCS51MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define MCS51_EXPAND_PSEUDO_NAME "MCS51 pseudo instruction expansion pass"

namespace {

/// Expands "placeholder" instructions marked as pseudo into
/// actual MCS51 instructions.
class MCS51ExpandPseudo : public MachineFunctionPass {
public:
  static char ID;

  MCS51ExpandPseudo() : MachineFunctionPass(ID) {
    initializeMCS51ExpandPseudoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return MCS51_EXPAND_PSEUDO_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const MCS51RegisterInfo *TRI;
  const TargetInstrInfo *TII;

  /// The register to be used for temporary storage.
  const Register SCRATCH_REGISTER = MCS51::R0;
  /// The register that will always contain zero.
  const Register ZERO_REGISTER = MCS51::R1;
  /// The IO address of the status register.
  const unsigned SREG_ADDR = 0x3f;

  bool expandMBB(Block &MBB);
  bool expandMI(Block &MBB, BlockIt MBBI);
  template <unsigned OP> bool expand(Block &MBB, BlockIt MBBI);

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode));
  }

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode,
                              Register DstReg) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode), DstReg);
  }

  MachineRegisterInfo &getRegInfo(Block &MBB) { return MBB.getParent()->getRegInfo(); }

  bool expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI);
  bool expandLogic(unsigned Op, Block &MBB, BlockIt MBBI);
  bool expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI);
  bool isLogicImmOpRedundant(unsigned Op, unsigned ImmVal) const;

  template<typename Func>
  bool expandAtomic(Block &MBB, BlockIt MBBI, Func f);

  template<typename Func>
  bool expandAtomicBinaryOp(unsigned Opcode, Block &MBB, BlockIt MBBI, Func f);

  bool expandAtomicBinaryOp(unsigned Opcode, Block &MBB, BlockIt MBBI);

  bool expandAtomicArithmeticOp(unsigned MemOpcode,
                                unsigned ArithOpcode,
                                Block &MBB,
                                BlockIt MBBI);

  /// Specific shift implementation.
  bool expandLSLB7Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRB7Rd(Block &MBB, BlockIt MBBI);
  bool expandASRB7Rd(Block &MBB, BlockIt MBBI);
  bool expandLSLW4Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRW4Rd(Block &MBB, BlockIt MBBI);
  bool expandLSLW8Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRW8Rd(Block &MBB, BlockIt MBBI);
  bool expandASRW8Rd(Block &MBB, BlockIt MBBI);
  bool expandLSLW12Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRW12Rd(Block &MBB, BlockIt MBBI);

  /// Scavenges a free GPR8 register for use.
  Register scavengeGPR8(MachineInstr &MI);
};

char MCS51ExpandPseudo::ID = 0;

bool MCS51ExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool MCS51ExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const MCS51Subtarget &STI = MF.getSubtarget<MCS51Subtarget>();
  TRI = STI.getRegisterInfo();
  TII = STI.getInstrInfo();

  // We need to track liveness in order to use register scavenging.
  MF.getProperties().set(MachineFunctionProperties::Property::TracksLiveness);

  for (Block &MBB : MF) {
    bool ContinueExpanding = true;
    unsigned ExpandCount = 0;

    // Continue expanding the block until all pseudos are expanded.
    do {
      assert(ExpandCount < 10 && "pseudo expand limit reached");

      bool BlockModified = expandMBB(MBB);
      Modified |= BlockModified;
      ExpandCount++;

      ContinueExpanding = BlockModified;
    } while (ContinueExpanding);
  }

  return Modified;
}

bool MCS51ExpandPseudo::
expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::
expandLogic(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, Op)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(3).setIsDead();

  auto MIBHI = buildMI(MBB, MBBI, Op)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::
  isLogicImmOpRedundant(unsigned Op, unsigned ImmVal) const {

  // ANDI Rd, 0xff is redundant.
  if (Op == MCS51::ANDIRdK && ImmVal == 0xff)
    return true;

  // ORI Rd, 0x0 is redundant.
  if (Op == MCS51::ORIRdK && ImmVal == 0x0)
    return true;

  return false;
}

bool MCS51ExpandPseudo::
expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  unsigned Imm = MI.getOperand(2).getImm();
  unsigned Lo8 = Imm & 0xff;
  unsigned Hi8 = (Imm >> 8) & 0xff;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (!isLogicImmOpRedundant(Op, Lo8)) {
    auto MIBLO = buildMI(MBB, MBBI, Op)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(SrcIsKill))
      .addImm(Lo8);

    // SREG is always implicitly dead
    MIBLO->getOperand(3).setIsDead();
  }

  if (!isLogicImmOpRedundant(Op, Hi8)) {
    auto MIBHI = buildMI(MBB, MBBI, Op)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(SrcIsKill))
      .addImm(Hi8);

    if (ImpIsDead)
      MIBHI->getOperand(3).setIsDead();
  }

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ADDWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(MCS51::ADDRdRr, MCS51::ADCRdRr, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ADCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(MCS51::ADCRdRr, MCS51::ADCRdRr, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::SUBWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(MCS51::SUBRdRr, MCS51::SBCRdRr, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::SUBIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, MCS51::SUBIRdK)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, MCS51::SBCIRdK)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(SrcIsKill));

  switch (MI.getOperand(2).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(2).getGlobal();
    int64_t Offs = MI.getOperand(2).getOffset();
    unsigned TF = MI.getOperand(2).getTargetFlags();
    MIBLO.addGlobalAddress(GV, Offs, TF | MCS51II::MO_NEG | MCS51II::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | MCS51II::MO_NEG | MCS51II::MO_HI);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(2).getImm();
    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::SBCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(MCS51::SBCRdRr, MCS51::SBCRdRr, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::SBCIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  unsigned Imm = MI.getOperand(2).getImm();
  unsigned Lo8 = Imm & 0xff;
  unsigned Hi8 = (Imm >> 8) & 0xff;
  unsigned OpLo = MCS51::SBCIRdK;
  unsigned OpHi = MCS51::SBCIRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(SrcIsKill))
    .addImm(Lo8);

  // SREG is always implicitly killed
  MIBLO->getOperand(4).setIsKill();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(SrcIsKill))
    .addImm(Hi8);

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ANDWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(MCS51::ANDRdRr, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ANDIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(MCS51::ANDIRdK, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(MCS51::ORRdRr, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ORIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(MCS51::ORIRdK, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::EORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(MCS51::EORRdRr, MBB, MBBI);
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::COMWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = MCS51::COMRd;
  unsigned OpHi = MCS51::COMRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(2).setIsDead();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::NEGWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Do NEG on the upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, MCS51::NEGRd)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  // SREG is always implicitly dead
  MIBHI->getOperand(2).setIsDead();

  // Do NEG on the lower byte.
  buildMI(MBB, MBBI, MCS51::NEGRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // Do an extra SBC.
  auto MISBCI =
      buildMI(MBB, MBBI, MCS51::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(ZERO_REGISTER);
  if (ImpIsDead)
    MISBCI->getOperand(3).setIsDead();
  // SREG is always implicitly killed
  MISBCI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::CPWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = MCS51::CPRdRr;
  unsigned OpHi = MCS51::CPCRdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::CPCWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = MCS51::CPCRdRr;
  unsigned OpHi = MCS51::CPCRdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LDIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = MCS51::LDIRdK;
  unsigned OpHi = MCS51::LDIRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF | MCS51II::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | MCS51II::MO_HI);
    break;
  }
  case MachineOperand::MO_BlockAddress: {
    const BlockAddress *BA = MI.getOperand(1).getBlockAddress();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.add(MachineOperand::CreateBA(BA, TF | MCS51II::MO_LO));
    MIBHI.add(MachineOperand::CreateBA(BA, TF | MCS51II::MO_HI));
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LDSWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = MCS51::LDSRdK;
  unsigned OpHi = MCS51::LDSRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF);
    MIBHI.addGlobalAddress(GV, Offs + 1, TF);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm);
    MIBHI.addImm(Imm + 1);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LDWRdPtr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::LDRdPtr;
  unsigned OpHi = MCS51::LDDRdPtrQ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(CurDstLoReg, RegState::Define)
                   .addReg(SrcReg);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, MCS51::PUSHRr).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(CurDstHiReg, RegState::Define)
    .addReg(SrcReg, getKillRegState(SrcIsKill))
    .addImm(1);

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, MCS51::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, MCS51::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LDWRdPtrPi>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsDead = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::LDRdPtrPi;
  unsigned OpHi = MCS51::LDRdPtrPi;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(SrcReg, RegState::Define)
    .addReg(SrcReg, RegState::Kill);

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(SrcReg, RegState::Define | getDeadRegState(SrcIsDead))
    .addReg(SrcReg, RegState::Kill);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LDWRdPtrPd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsDead = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::LDRdPtrPd;
  unsigned OpHi = MCS51::LDRdPtrPd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(SrcReg, RegState::Define)
    .addReg(SrcReg, RegState::Kill);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(SrcReg, RegState::Define | getDeadRegState(SrcIsDead))
    .addReg(SrcReg, RegState::Kill);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LDDWRdPtrQ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::LDDRdPtrQ;
  unsigned OpHi = MCS51::LDDRdPtrQ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Offset is out of range");

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(CurDstLoReg, RegState::Define)
    .addReg(SrcReg)
    .addImm(Imm);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, MCS51::PUSHRr).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(CurDstHiReg, RegState::Define)
    .addReg(SrcReg, getKillRegState(SrcIsKill))
    .addImm(Imm + 1);

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, MCS51::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, MCS51::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LPMWRdZ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::LPMRdZPi;
  unsigned OpHi = MCS51::LPMRdZ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
      .addReg(CurDstLoReg, RegState::Define)
      .addReg(SrcReg);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, MCS51::PUSHRr).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
      .addReg(CurDstHiReg, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, MCS51::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, MCS51::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LPMWRdZPi>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("wide LPMPi is unimplemented");
}

template<typename Func>
bool MCS51ExpandPseudo::expandAtomic(Block &MBB, BlockIt MBBI, Func f) {
  // Remove the pseudo instruction.
  MachineInstr &MI = *MBBI;

  // Store the SREG.
  buildMI(MBB, MBBI, MCS51::INRdA)
    .addReg(SCRATCH_REGISTER, RegState::Define)
    .addImm(SREG_ADDR);

  // Disable exceptions.
  buildMI(MBB, MBBI, MCS51::BCLRs).addImm(7); // CLI

  f(MI);

  // Restore the status reg.
  buildMI(MBB, MBBI, MCS51::OUTARr)
    .addImm(SREG_ADDR)
    .addReg(SCRATCH_REGISTER);

  MI.eraseFromParent();
  return true;
}

template<typename Func>
bool MCS51ExpandPseudo::expandAtomicBinaryOp(unsigned Opcode,
                                           Block &MBB,
                                           BlockIt MBBI,
                                           Func f) {
  return expandAtomic(MBB, MBBI, [&](MachineInstr &MI) {
      auto Op1 = MI.getOperand(0);
      auto Op2 = MI.getOperand(1);

      MachineInstr &NewInst =
          *buildMI(MBB, MBBI, Opcode).add(Op1).add(Op2).getInstr();
      f(NewInst);
  });
}

bool MCS51ExpandPseudo::expandAtomicBinaryOp(unsigned Opcode,
                                           Block &MBB,
                                           BlockIt MBBI) {
  return expandAtomicBinaryOp(Opcode, MBB, MBBI, [](MachineInstr &MI) {});
}

bool MCS51ExpandPseudo::expandAtomicArithmeticOp(unsigned Width,
                                               unsigned ArithOpcode,
                                               Block &MBB,
                                               BlockIt MBBI) {
  return expandAtomic(MBB, MBBI, [&](MachineInstr &MI) {
      auto DstReg = MI.getOperand(0).getReg();
      auto PtrOp = MI.getOperand(1);
      auto SrcReg = MI.getOperand(2).getReg();

      unsigned LoadOpcode = (Width == 8) ? MCS51::LDRdPtr : MCS51::LDWRdPtr;
      unsigned StoreOpcode = (Width == 8) ? MCS51::STPtrRr : MCS51::STWPtrRr;

      // FIXME: this returns the new value (after the operation), not the old
      // value as the atomicrmw instruction is supposed to do!

      // Create the load
      buildMI(MBB, MBBI, LoadOpcode, DstReg).addReg(PtrOp.getReg());

      // Create the arithmetic op
      buildMI(MBB, MBBI, ArithOpcode, DstReg).addReg(DstReg).addReg(SrcReg);

      // Create the store
      buildMI(MBB, MBBI, StoreOpcode).add(PtrOp).addReg(DstReg);
  });
}

Register MCS51ExpandPseudo::scavengeGPR8(MachineInstr &MI) {
  MachineBasicBlock &MBB = *MI.getParent();
  RegScavenger RS;

  RS.enterBasicBlock(MBB);
  RS.forward(MI);

  BitVector Candidates =
      TRI->getAllocatableSet
      (*MBB.getParent(), &MCS51::GPR8RegClass);

  // Exclude all the registers being used by the instruction.
  for (MachineOperand &MO : MI.operands()) {
    if (MO.isReg() && MO.getReg() != 0 && !MO.isDef() &&
        !Register::isVirtualRegister(MO.getReg()))
      Candidates.reset(MO.getReg());
  }

  BitVector Available = RS.getRegsAvailable(&MCS51::GPR8RegClass);
  Available &= Candidates;

  signed Reg = Available.find_first();
  assert(Reg != -1 && "ran out of registers");
  return Reg;
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoad8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(MCS51::LDRdPtr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoad16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(MCS51::LDWRdPtr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicStore8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(MCS51::STPtrRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicStore16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(MCS51::STWPtrRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadAdd8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(8, MCS51::ADDRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadAdd16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(16, MCS51::ADDWRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadSub8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(8, MCS51::SUBRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadSub16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(16, MCS51::SUBWRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadAnd8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(8, MCS51::ANDRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadAnd16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(16, MCS51::ANDWRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadOr8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(8, MCS51::ORRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadOr16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(16, MCS51::ORWRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadXor8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(8, MCS51::EORRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicLoadXor16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicArithmeticOp(16, MCS51::EORWRdRr, MBB, MBBI);
}

template<>
bool MCS51ExpandPseudo::expand<MCS51::AtomicFence>(Block &MBB, BlockIt MBBI) {
  // On MCS51, there is only one core and so atomic fences do nothing.
  MBBI->eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::STSWKRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::STSKRr;
  unsigned OpHi = MCS51::STSKRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Write the high byte first in case this address belongs to a special
  // I/O address with a special temporary register.
  auto MIBHI = buildMI(MBB, MBBI, OpHi);
  auto MIBLO = buildMI(MBB, MBBI, OpLo);

  switch (MI.getOperand(0).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(0).getGlobal();
    int64_t Offs = MI.getOperand(0).getOffset();
    unsigned TF = MI.getOperand(0).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF);
    MIBHI.addGlobalAddress(GV, Offs + 1, TF);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(0).getImm();

    MIBLO.addImm(Imm);
    MIBHI.addImm(Imm + 1);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MIBLO.addReg(SrcLoReg, getKillRegState(SrcIsKill));
  MIBHI.addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::STWPtrRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsUndef = MI.getOperand(0).isUndef();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::STPtrRr;
  unsigned OpHi = MCS51::STDPtrQRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  //:TODO: need to reverse this order like inw and stsw?
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg, getUndefRegState(DstIsUndef))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, getUndefRegState(DstIsUndef))
    .addImm(1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::STWPtrPiRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(3).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = MCS51::STPtrPiRr;
  unsigned OpHi = MCS51::STPtrPiRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg, RegState::Define)
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::STWPtrPdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(3).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = MCS51::STPtrPdRr;
  unsigned OpHi = MCS51::STPtrPdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, RegState::Define)
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::STDWPtrQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(1).getImm();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = MCS51::STDPtrQRr;
  unsigned OpHi = MCS51::STDPtrQRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Offset is out of range");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg)
    .addImm(Imm)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, getKillRegState(DstIsKill))
    .addImm(Imm + 1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::INWRdA>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  unsigned Imm = MI.getOperand(1).getImm();
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = MCS51::INRdA;
  unsigned OpHi = MCS51::INRdA;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(Imm);

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(Imm + 1);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::OUTWARr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  unsigned Imm = MI.getOperand(0).getImm();
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = MCS51::OUTARr;
  unsigned OpHi = MCS51::OUTARr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");

  // 16 bit I/O writes need the high byte first
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addImm(Imm + 1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addImm(Imm)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::PUSHWRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(0).isKill();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = MCS51::PUSHRr;
  unsigned OpHi = MCS51::PUSHRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::POPWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = MCS51::POPRd;
  unsigned OpHi = MCS51::POPRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpHi, DstHiReg).setMIFlags(Flags); // High
  buildMI(MBB, MBBI, OpLo, DstLoReg).setMIFlags(Flags); // Low

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ROLBRd>(Block &MBB, BlockIt MBBI) {
  // In MCS51, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  unsigned OpShift, OpCarry;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  OpShift = MCS51::ADDRdRr;
  OpCarry = MCS51::ADCRdRr;

  // add r16, r16
  // adc r16, r1

  // Shift part
  buildMI(MBB, MBBI, OpShift)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(DstReg);

  // Add the carry bit
  auto MIB = buildMI(MBB, MBBI, OpCarry)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(ZERO_REGISTER);

  // SREG is always implicitly killed
  MIB->getOperand(2).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::RORBRd>(Block &MBB, BlockIt MBBI) {
  // In MCS51, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();

  // bst r16, 0
  // ror r16
  // bld r16, 7

  // Move the lowest bit from DstReg into the T bit
  buildMI(MBB, MBBI, MCS51::BST).addReg(DstReg).addImm(0);

  // Rotate to the right
  buildMI(MBB, MBBI, MCS51::RORRd, DstReg).addReg(DstReg);

  // Move the T bit into the highest bit of DstReg.
  buildMI(MBB, MBBI, MCS51::BLD, DstReg).addReg(DstReg).addImm(7);

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LSLWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = MCS51::ADDRdRr; // ADD Rd, Rd <==> LSL Rd
  unsigned OpHi = MCS51::ADCRdRr; // ADC Rd, Rd <==> ROL Rd
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg)
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg)
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandLSLW4Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // swap Rh
  // swap Rl
  buildMI(MBB, MBBI, MCS51::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));
  buildMI(MBB, MBBI, MCS51::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // andi Rh, 0xf0
  auto MI0 =
      buildMI(MBB, MBBI, MCS51::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addImm(0xf0);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // eor Rh, Rl
  auto MI1 =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg);
  // SREG is implicitly dead.
  MI1->getOperand(3).setIsDead();

  // andi Rl, 0xf0
  auto MI2 =
      buildMI(MBB, MBBI, MCS51::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addImm(0xf0);
  // SREG is implicitly dead.
  MI2->getOperand(3).setIsDead();

  // eor Rh, Rl
  auto MI3 =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg);
  if (ImpIsDead)
    MI3->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandLSLW8Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // mov Rh, Rl
  buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg);

  // clr Rl
  auto MIBLO =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBLO->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandLSLW12Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // mov Rh, Rl
  buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg);

  // swap Rh
  buildMI(MBB, MBBI, MCS51::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));

  // andi Rh, 0xf0
  auto MI0 =
      buildMI(MBB, MBBI, MCS51::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addImm(0xf0);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // clr Rl
  auto MI1 =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MI1->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LSLWNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 4:
    return expandLSLW4Rd(MBB, MBBI);
  case 8:
    return expandLSLW8Rd(MBB, MBBI);
  case 12:
    return expandLSLW12Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lslwn");
    return false;
  }
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LSRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = MCS51::RORRd;
  unsigned OpHi = MCS51::LSRRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandLSRW4Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // swap Rh
  // swap Rl
  buildMI(MBB, MBBI, MCS51::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));
  buildMI(MBB, MBBI, MCS51::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // andi Rl, 0xf
  auto MI0 =
      buildMI(MBB, MBBI, MCS51::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // eor Rl, Rh
  auto MI1 =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg);
  // SREG is implicitly dead.
  MI1->getOperand(3).setIsDead();

  // andi Rh, 0xf
  auto MI2 =
      buildMI(MBB, MBBI, MCS51::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI2->getOperand(3).setIsDead();

  // eor Rl, Rh
  auto MI3 =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg);
  if (ImpIsDead)
    MI3->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandLSRW8Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // Clear upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandLSRW12Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // swap Rl
  buildMI(MBB, MBBI, MCS51::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // andi Rl, 0xf
  auto MI0 =
      buildMI(MBB, MBBI, MCS51::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // Clear upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, MCS51::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LSRWNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 4:
    return expandLSRW4Rd(MBB, MBBI);
  case 8:
    return expandLSRW8Rd(MBB, MBBI);
  case 12:
    return expandLSRW12Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lsrwn");
    return false;
  }
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::RORWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("RORW unimplemented");
  return false;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ROLWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("ROLW unimplemented");
  return false;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ASRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = MCS51::RORRd;
  unsigned OpHi = MCS51::ASRRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandASRW8Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // Move the sign bit to the C flag.
  buildMI(MBB, MBBI, MCS51::ADDRdRr)
      .addReg(DstHiReg, RegState::Define, getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill) | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));

  // Set upper byte to 0 or -1.
  auto MIBHI =
      buildMI(MBB, MBBI, MCS51::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ASRWNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 8:
    return expandASRW8Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented asrwn");
    return false;
  }
}

bool MCS51ExpandPseudo::expandLSLB7Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  // ror r24
  // clr r24
  // ror r24

  buildMI(MBB, MBBI, MCS51::RORRd)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      ->getOperand(3).setIsUndef(true);

  buildMI(MBB, MBBI, MCS51::EORRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  auto MIRRC =
      buildMI(MBB, MBBI, MCS51::RORRd)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LSLBNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 7:
    return expandLSLB7Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lslbn");
    return false;
  }
}

bool MCS51ExpandPseudo::expandLSRB7Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  // rol r24
  // clr r24
  // rol r24

  buildMI(MBB, MBBI, MCS51::ADCRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill))
      ->getOperand(4).setIsUndef(true);

  buildMI(MBB, MBBI, MCS51::EORRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  auto MIRRC =
      buildMI(MBB, MBBI, MCS51::ADCRdRr)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, getKillRegState(DstIsKill))
          .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::LSRBNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 7:
    return expandLSRB7Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lsrbn");
    return false;
  }
}

bool MCS51ExpandPseudo::expandASRB7Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  // lsl r24
  // sbc r24, r24

  buildMI(MBB, MBBI, MCS51::ADDRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  auto MIRRC = buildMI(MBB, MBBI, MCS51::SBCRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::ASRBNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 7:
    return expandASRB7Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented asrbn");
    return false;
  }
}

template <> bool MCS51ExpandPseudo::expand<MCS51::SEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  // sext R17:R16, R17
  // mov     r16, r17
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R13
  // mov     r16, r13
  // mov     r17, r13
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R16
  // mov     r17, r16
  // lsl     r17
  // sbc     r17, r17
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    auto MOV = buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(SrcReg);

    if (SrcReg == DstHiReg) {
      MOV->getOperand(1).setIsKill();
    }
  }

  if (SrcReg != DstHiReg) {
    buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstHiReg, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  buildMI(MBB, MBBI, MCS51::ADDRdRr) // LSL Rd <==> ADD Rd, Rr
    .addReg(DstHiReg, RegState::Define)
    .addReg(DstHiReg)
    .addReg(DstHiReg, RegState::Kill);

  auto SBC = buildMI(MBB, MBBI, MCS51::SBCRdRr)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, RegState::Kill)
    .addReg(DstHiReg, RegState::Kill);

  if (ImpIsDead)
    SBC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  SBC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <> bool MCS51ExpandPseudo::expand<MCS51::ZEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  // zext R25:R24, R20
  // mov      R24, R20
  // eor      R25, R25
  // zext R25:R24, R24
  // eor      R25, R25
  // zext R25:R24, R25
  // mov      R24, R25
  // eor      R25, R25
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    buildMI(MBB, MBBI, MCS51::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  auto EOR = buildMI(MBB, MBBI, MCS51::EORRdRr)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, RegState::Kill | RegState::Undef)
    .addReg(DstHiReg, RegState::Kill | RegState::Undef);

  if (ImpIsDead)
    EOR->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::SPREAD>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = MCS51::INRdA;
  unsigned OpHi = MCS51::INRdA;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(0x3d)
    .setMIFlags(Flags);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(0x3e)
    .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

template <>
bool MCS51ExpandPseudo::expand<MCS51::SPWRITE>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned Flags = MI.getFlags();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  buildMI(MBB, MBBI, MCS51::INRdA)
    .addReg(MCS51::R0, RegState::Define)
    .addImm(SREG_ADDR)
    .setMIFlags(Flags);

  buildMI(MBB, MBBI, MCS51::BCLRs).addImm(0x07).setMIFlags(Flags);

  buildMI(MBB, MBBI, MCS51::OUTARr)
    .addImm(0x3e)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  buildMI(MBB, MBBI, MCS51::OUTARr)
    .addImm(SREG_ADDR)
    .addReg(MCS51::R0, RegState::Kill)
    .setMIFlags(Flags);

  buildMI(MBB, MBBI, MCS51::OUTARr)
    .addImm(0x3d)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

bool MCS51ExpandPseudo::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)               \
  case Op:                       \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    EXPAND(MCS51::ADDWRdRr);
    EXPAND(MCS51::ADCWRdRr);
    EXPAND(MCS51::SUBWRdRr);
    EXPAND(MCS51::SUBIWRdK);
    EXPAND(MCS51::SBCWRdRr);
    EXPAND(MCS51::SBCIWRdK);
    EXPAND(MCS51::ANDWRdRr);
    EXPAND(MCS51::ANDIWRdK);
    EXPAND(MCS51::ORWRdRr);
    EXPAND(MCS51::ORIWRdK);
    EXPAND(MCS51::EORWRdRr);
    EXPAND(MCS51::COMWRd);
    EXPAND(MCS51::NEGWRd);
    EXPAND(MCS51::CPWRdRr);
    EXPAND(MCS51::CPCWRdRr);
    EXPAND(MCS51::LDIWRdK);
    EXPAND(MCS51::LDSWRdK);
    EXPAND(MCS51::LDWRdPtr);
    EXPAND(MCS51::LDWRdPtrPi);
    EXPAND(MCS51::LDWRdPtrPd);
  case MCS51::LDDWRdYQ: //:FIXME: remove this once PR13375 gets fixed
    EXPAND(MCS51::LDDWRdPtrQ);
    EXPAND(MCS51::LPMWRdZ);
    EXPAND(MCS51::LPMWRdZPi);
    EXPAND(MCS51::AtomicLoad8);
    EXPAND(MCS51::AtomicLoad16);
    EXPAND(MCS51::AtomicStore8);
    EXPAND(MCS51::AtomicStore16);
    EXPAND(MCS51::AtomicLoadAdd8);
    EXPAND(MCS51::AtomicLoadAdd16);
    EXPAND(MCS51::AtomicLoadSub8);
    EXPAND(MCS51::AtomicLoadSub16);
    EXPAND(MCS51::AtomicLoadAnd8);
    EXPAND(MCS51::AtomicLoadAnd16);
    EXPAND(MCS51::AtomicLoadOr8);
    EXPAND(MCS51::AtomicLoadOr16);
    EXPAND(MCS51::AtomicLoadXor8);
    EXPAND(MCS51::AtomicLoadXor16);
    EXPAND(MCS51::AtomicFence);
    EXPAND(MCS51::STSWKRr);
    EXPAND(MCS51::STWPtrRr);
    EXPAND(MCS51::STWPtrPiRr);
    EXPAND(MCS51::STWPtrPdRr);
    EXPAND(MCS51::STDWPtrQRr);
    EXPAND(MCS51::INWRdA);
    EXPAND(MCS51::OUTWARr);
    EXPAND(MCS51::PUSHWRr);
    EXPAND(MCS51::POPWRd);
    EXPAND(MCS51::ROLBRd);
    EXPAND(MCS51::RORBRd);
    EXPAND(MCS51::LSLWRd);
    EXPAND(MCS51::LSRWRd);
    EXPAND(MCS51::RORWRd);
    EXPAND(MCS51::ROLWRd);
    EXPAND(MCS51::ASRWRd);
    EXPAND(MCS51::LSLWNRd);
    EXPAND(MCS51::LSRWNRd);
    EXPAND(MCS51::ASRWNRd);
    EXPAND(MCS51::LSLBNRd);
    EXPAND(MCS51::LSRBNRd);
    EXPAND(MCS51::ASRBNRd);
    EXPAND(MCS51::SEXT);
    EXPAND(MCS51::ZEXT);
    EXPAND(MCS51::SPREAD);
    EXPAND(MCS51::SPWRITE);
  }
#undef EXPAND
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(MCS51ExpandPseudo, "mcs51-expand-pseudo",
                MCS51_EXPAND_PSEUDO_NAME, false, false)
namespace llvm {

FunctionPass *createMCS51ExpandPseudoPass() { return new MCS51ExpandPseudo(); }

} // end of namespace llvm
