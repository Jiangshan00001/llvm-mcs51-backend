//===-- MCS51TargetStreamer.h - MCS51 Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCS51_TARGET_STREAMER_H
#define LLVM_MCS51_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic MCS51 target output stream.
class MCS51TargetStreamer : public MCTargetStreamer {
public:
  explicit MCS51TargetStreamer(MCStreamer &S);

  void finish() override;
};

/// A target streamer for textual MCS51 assembly code.
class MCS51TargetAsmStreamer : public MCS51TargetStreamer {
public:
  explicit MCS51TargetAsmStreamer(MCStreamer &S);
};

} // end namespace llvm

#endif // LLVM_MCS51_TARGET_STREAMER_H
