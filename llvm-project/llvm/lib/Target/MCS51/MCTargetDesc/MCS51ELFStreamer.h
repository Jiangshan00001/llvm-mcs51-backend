//===----- MCS51ELFStreamer.h - MCS51 Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCS51_ELF_STREAMER_H
#define LLVM_MCS51_ELF_STREAMER_H

#include "MCS51TargetStreamer.h"

namespace llvm {

/// A target streamer for an MCS51 ELF object file.
class MCS51ELFStreamer : public MCS51TargetStreamer {
public:
  MCS51ELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  MCELFStreamer &getStreamer() {
    return static_cast<MCELFStreamer &>(Streamer);
  }
};

} // end namespace llvm

#endif
