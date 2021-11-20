//===-- MCS51.h - Top-level interface for MCS51 representation ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// MCS51 back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCS51_H
#define LLVM_MCS51_H

#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class MCS51TargetMachine;
class FunctionPass;

Pass *createMCS51ShiftExpandPass();
FunctionPass *createMCS51ISelDag(MCS51TargetMachine &TM,
                               CodeGenOpt::Level OptLevel);
FunctionPass *createMCS51ExpandPseudoPass();
FunctionPass *createMCS51FrameAnalyzerPass();
FunctionPass *createMCS51RelaxMemPass();
FunctionPass *createMCS51DynAllocaSRPass();
FunctionPass *createMCS51BranchSelectionPass();

void initializeMCS51ShiftExpandPass(PassRegistry &);
void initializeMCS51ExpandPseudoPass(PassRegistry&);
void initializeMCS51RelaxMemPass(PassRegistry&);

/// Contains the MCS51 backend.
namespace MCS51 {

/// An integer that identifies all of the supported MCS51 address spaces.
enum AddressSpace { DataMemory, ProgramMemory };

/// Checks if a given type is a pointer to program memory.
template <typename T> bool isProgramMemoryAddress(T *V) {
  return cast<PointerType>(V->getType())->getAddressSpace() == ProgramMemory;
}

inline bool isProgramMemoryAccess(MemSDNode const *N) {
  auto V = N->getMemOperand()->getValue();

  return (V != nullptr) ? isProgramMemoryAddress(V) : false;
}

} // end of namespace MCS51

} // end namespace llvm

#endif // LLVM_MCS51_H
