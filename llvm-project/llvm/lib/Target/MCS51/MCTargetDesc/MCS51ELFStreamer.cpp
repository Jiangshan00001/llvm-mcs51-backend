#include "MCS51ELFStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/FormattedStream.h"

#include "MCS51MCTargetDesc.h"

namespace llvm {

static unsigned getEFlagsForFeatureSet(const FeatureBitset &Features) {
  unsigned EFlags = 0;
  EFlags |= ELF::EM_IAMCU;

#if 0
  // Set architecture
  if (Features[MCS51::ELFArchMCS511])
    EFlags |= ELF::EF_MCS51_ARCH_MCS511;
  else if (Features[MCS51::ELFArchMCS512])
    EFlags |= ELF::EF_MCS51_ARCH_MCS512;
  else if (Features[MCS51::ELFArchMCS5125])
    EFlags |= ELF::EF_MCS51_ARCH_MCS5125;
  else if (Features[MCS51::ELFArchMCS513])
    EFlags |= ELF::EF_MCS51_ARCH_MCS513;
  else if (Features[MCS51::ELFArchMCS5131])
    EFlags |= ELF::EF_MCS51_ARCH_MCS5131;
  else if (Features[MCS51::ELFArchMCS5135])
    EFlags |= ELF::EF_MCS51_ARCH_MCS5135;
  else if (Features[MCS51::ELFArchMCS514])
    EFlags |= ELF::EF_MCS51_ARCH_MCS514;
  else if (Features[MCS51::ELFArchMCS515])
    EFlags |= ELF::EF_MCS51_ARCH_MCS515;
  else if (Features[MCS51::ELFArchMCS5151])
    EFlags |= ELF::EF_MCS51_ARCH_MCS5151;
  else if (Features[MCS51::ELFArchMCS516])
    EFlags |= ELF::EF_MCS51_ARCH_MCS516;
  else if (Features[MCS51::ELFArchTiny])
    EFlags |= ELF::EF_MCS51_ARCH_MCS51TINY;
  else if (Features[MCS51::ELFArchXMEGA1])
    EFlags |= ELF::EF_MCS51_ARCH_XMEGA1;
  else if (Features[MCS51::ELFArchXMEGA2])
    EFlags |= ELF::EF_MCS51_ARCH_XMEGA2;
  else if (Features[MCS51::ELFArchXMEGA3])
    EFlags |= ELF::EF_MCS51_ARCH_XMEGA3;
  else if (Features[MCS51::ELFArchXMEGA4])
    EFlags |= ELF::EF_MCS51_ARCH_XMEGA4;
  else if (Features[MCS51::ELFArchXMEGA5])
    EFlags |= ELF::EF_MCS51_ARCH_XMEGA5;
  else if (Features[MCS51::ELFArchXMEGA6])
    EFlags |= ELF::EF_MCS51_ARCH_XMEGA6;
  else if (Features[MCS51::ELFArchXMEGA7])
    EFlags |= ELF::EF_MCS51_ARCH_XMEGA7;
#endif
  return EFlags;
}

MCS51ELFStreamer::MCS51ELFStreamer(MCStreamer &S,
                               const MCSubtargetInfo &STI)
    : MCS51TargetStreamer(S) {

  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned EFlags = MCA.getELFHeaderEFlags();

  EFlags |= getEFlagsForFeatureSet(STI.getFeatureBits());

  MCA.setELFHeaderEFlags(EFlags);
}

} // end namespace llvm
