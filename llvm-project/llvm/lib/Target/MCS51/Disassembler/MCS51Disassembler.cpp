//===- MCS51Disassembler.cpp - Disassembler for MCS51 ---------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the MCS51 Disassembler.
//
//===----------------------------------------------------------------------===//

#include "MCS51.h"
#include "MCS51RegisterInfo.h"
#include "MCS51Subtarget.h"
#include "MCTargetDesc/MCS51MCTargetDesc.h"
#include "TargetInfo/MCS51TargetInfo.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "MCS51-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// A disassembler class for MCS51.
class MCS51Disassembler : public MCDisassembler {
public:
  MCS51Disassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  virtual ~MCS51Disassembler() {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;
};
}

static MCDisassembler *createMCS51Disassembler(const Target &T,
                                             const MCSubtargetInfo &STI,
                                             MCContext &Ctx) {
  return new MCS51Disassembler(STI, Ctx);
}


extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMCS51Disassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheMCS51Target(),
                                         createMCS51Disassembler);
}

static const uint16_t GPRDecoderTable[] = {
  MCS51::R0, MCS51::R1, MCS51::R2, MCS51::R3,
  MCS51::R4, MCS51::R5, MCS51::R6, MCS51::R7,
  MCS51::R8, MCS51::R9, MCS51::R10, MCS51::R11,
  MCS51::R12, MCS51::R13, MCS51::R14, MCS51::R15,
  MCS51::R16, MCS51::R17, MCS51::R18, MCS51::R19,
  MCS51::R20, MCS51::R21, MCS51::R22, MCS51::R23,
  MCS51::R24, MCS51::R25, MCS51::R26, MCS51::R27,
  MCS51::R28, MCS51::R29, MCS51::R30, MCS51::R31,
};

static DecodeStatus DecodeGPR8RegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address, const void *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeLD8RegisterClass(MCInst &Inst, unsigned RegNo,
                                           uint64_t Address, const void *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo+16];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodePTRREGSRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address, const void *Decoder) {
  // Note: this function must be defined but does not seem to be called.
  assert(false && "unimplemented: PTRREGS register class");
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Insn,
                                     uint64_t Address, const void *Decoder);

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn,
                              uint64_t Address, const void *Decoder);

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder);

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder);

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

#include "MCS51GenDisassemblerTables.inc"

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  Inst.addOperand(MCOperand::createImm(addr));
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(addr));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = fieldFromInstruction(Insn, 3, 5);
  unsigned b = fieldFromInstruction(Insn, 0, 3);
  Inst.addOperand(MCOperand::createImm(addr));
  Inst.addOperand(MCOperand::createImm(b));
  return MCDisassembler::Success;
}

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Field,
                                     uint64_t Address, const void *Decoder) {
  // Call targets need to be shifted left by one so this needs a custom
  // decoder.
  Inst.addOperand(MCOperand::createImm(Field << 1));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn,
                              uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder) {
  if (decodeFRd(Inst, Insn, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(MCS51::R31R30));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 3) + 16;
  unsigned r = fieldFromInstruction(Insn, 0, 3) + 16;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned r = fieldFromInstruction(Insn, 4, 4) * 2;
  unsigned d = fieldFromInstruction(Insn, 0, 4) * 2;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 2) * 2 + 24; // starts at r24:r25
  unsigned k = 0;
  k |= fieldFromInstruction(Insn, 0, 4);
  k |= fieldFromInstruction(Insn, 6, 2) << 4;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(k));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned rd = fieldFromInstruction(Insn, 4, 4) + 16;
  unsigned rr = fieldFromInstruction(Insn, 0, 4) + 16;
  if (DecodeGPR8RegisterClass(Inst, rd, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, rr, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {
  if (Bytes.size() < 2) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 2;
  Insn = (Bytes[0] << 0) | (Bytes[1] << 8);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 4;
  Insn = (Bytes[0] << 16) | (Bytes[1] << 24) | (Bytes[2] << 0) | (Bytes[3] << 8);

  return MCDisassembler::Success;
}

static const uint8_t *getDecoderTable(uint64_t Size) {

  switch (Size) {
    case 2: return DecoderTable16;
    case 4: return DecoderTable32;
    default: llvm_unreachable("instructions must be 16 or 32-bits");
  }
}

DecodeStatus MCS51Disassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  uint32_t Insn;

  DecodeStatus Result;

  // Try decode a 16-bit instruction.
  {
    Result = readInstruction16(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail) return MCDisassembler::Fail;

    // Try to auto-decode a 16-bit instruction.
    Result = decodeInstruction(getDecoderTable(Size), Instr,
                               Insn, Address, this, STI);

    if (Result != MCDisassembler::Fail)
      return Result;
  }

  // Try decode a 32-bit instruction.
  {
    Result = readInstruction32(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail) return MCDisassembler::Fail;

    Result = decodeInstruction(getDecoderTable(Size), Instr, Insn,
                               Address, this, STI);

    if (Result != MCDisassembler::Fail) {
      return Result;
    }

    return MCDisassembler::Fail;
  }
}

typedef DecodeStatus (*DecodeFunc)(MCInst &MI, unsigned insn, uint64_t Address,
                                   const void *Decoder);

