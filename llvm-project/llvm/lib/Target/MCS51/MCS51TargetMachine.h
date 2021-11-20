//===-- MCS51TargetMachine.h - Define TargetMachine for MCS51 -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MCS51 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCS51_TARGET_MACHINE_H
#define LLVM_MCS51_TARGET_MACHINE_H

#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#include "MCS51FrameLowering.h"
#include "MCS51ISelLowering.h"
#include "MCS51InstrInfo.h"
#include "MCS51SelectionDAGInfo.h"
#include "MCS51Subtarget.h"

namespace llvm {

/// A generic MCS51 implementation.
class MCS51TargetMachine : public LLVMTargetMachine {
public:
  MCS51TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Optional<Reloc::Model> RM,
                   Optional<CodeModel::Model> CM,
                   CodeGenOpt::Level OL, bool JIT);

  const MCS51Subtarget *getSubtargetImpl() const;
  const MCS51Subtarget *getSubtargetImpl(const Function &) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return this->TLOF.get();
  }

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  bool isMachineVerifierClean() const override {
    return false;
  }

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  MCS51Subtarget SubTarget;
};

} // end namespace llvm

#endif // LLVM_MCS51_TARGET_MACHINE_H
