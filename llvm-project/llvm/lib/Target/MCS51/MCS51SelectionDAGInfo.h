//===-- MCS51SelectionDAGInfo.h - MCS51 SelectionDAG Info -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MCS51 subclass for SelectionDAGTargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCS51_SELECTION_DAG_INFO_H
#define LLVM_MCS51_SELECTION_DAG_INFO_H

#include "llvm/CodeGen/SelectionDAGTargetInfo.h"

namespace llvm {

/// Holds information about the MCS51 instruction selection DAG.
class MCS51SelectionDAGInfo : public SelectionDAGTargetInfo {
public:
};

} // end namespace llvm

#endif // LLVM_MCS51_SELECTION_DAG_INFO_H
