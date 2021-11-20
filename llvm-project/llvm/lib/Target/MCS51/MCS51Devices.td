//===---------------------------------------------------------------------===//
// MCS51 Device Definitions
//===---------------------------------------------------------------------===//

// :TODO: Implement the skip errata, see `gcc/config/mcs51/mcs51-arch.h` for details
// :TODO: We define all devices with SRAM to have all variants of LD/ST/LDD/STD.
//        In reality, mcs51-1 (no SRAM) has one variant each of `LD` and `ST`.
//        mcs51-2 (with SRAM) adds the rest of the variants.


// A feature set aggregates features, grouping them. We don't want to create a
// new member in MCS51Subtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
  : SubtargetFeature<name, "m_FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, list<SubtargetFeature> i>
  : FeatureSet<name, !strconcat("The device is a part of the ",
               name, " family"), i>;

// The device has SRAM, and supports the bare minimum of
// SRAM-relevant instructions.
//
// These are:
// LD - all 9 variants
// ST - all 9 variants
// LDD - two variants for Y and Z
// STD - two variants for Y and Z
// `LDS Rd, K`
// `STS k, Rr`
// `PUSH`/`POP`
def FeatureSRAM           : SubtargetFeature<"sram", "m_hasSRAM", "true",
                                  "The device has random access memory">;

// The device supports the `JMP k` and `CALL k` instructions.
def FeatureJMPCALL        : SubtargetFeature<"jmpcall", "m_hasJMPCALL", "true",
                                  "The device supports the `JMP` and "
                                  "`CALL` instructions">;


// The device supports the indirect branches `IJMP` and `ICALL`.
def FeatureIJMPCALL       : SubtargetFeature<"ijmpcall", "m_hasIJMPCALL",
                                  "true",
                                  "The device supports `IJMP`/`ICALL`"
                                  "instructions">;

// The device supports the extended indirect branches `EIJMP` and `EICALL`.
def FeatureEIJMPCALL      : SubtargetFeature<"eijmpcall", "m_hasEIJMPCALL",
                                  "true", "The device supports the "
                                  "`EIJMP`/`EICALL` instructions">;

// The device supports `ADDI Rd, K`, `SUBI Rd, K`.
def FeatureADDSUBIW       : SubtargetFeature<"addsubiw", "m_hasADDSUBIW",
                                  "true", "Enable 16-bit register-immediate "
                                  "addition and subtraction instructions">;

// The device has an 8-bit stack pointer (SP) register.
def FeatureSmallStack     : SubtargetFeature<"smallstack", "m_hasSmallStack",
                                  "true", "The device has an 8-bit "
                                  "stack pointer">;

// The device supports the 16-bit GPR pair MOVW instruction.
def FeatureMOVW           : SubtargetFeature<"movw", "m_hasMOVW", "true",
                                  "The device supports the 16-bit MOVW "
                                  "instruction">;

// The device supports the `LPM` instruction, with implied destination being r0.
def FeatureLPM            : SubtargetFeature<"lpm", "m_hasLPM", "true",
                                  "The device supports the `LPM` instruction">;

// The device supports the `LPM Rd, Z[+] instruction.
def FeatureLPMX           : SubtargetFeature<"lpmx", "m_hasLPMX", "true",
                                  "The device supports the `LPM Rd, Z[+]` "
                                  "instruction">;

// The device supports the `ELPM` instruction.
def FeatureELPM           : SubtargetFeature<"elpm", "m_hasELPM", "true",
                                  "The device supports the ELPM instruction">;

// The device supports the `ELPM Rd, Z[+]` instructions.
def FeatureELPMX          : SubtargetFeature<"elpmx", "m_hasELPMX", "true",
                                  "The device supports the `ELPM Rd, Z[+]` "
                                  "instructions">;

// The device supports the `SPM` instruction.
def FeatureSPM            : SubtargetFeature<"spm", "m_hasSPM", "true",
                                  "The device supports the `SPM` instruction">;

// The device supports the `SPM Z+` instruction.
def FeatureSPMX           : SubtargetFeature<"spmx", "m_hasSPMX", "true",
                                  "The device supports the `SPM Z+` "
                                  "instruction">;

// The device supports the `DES k` instruction.
def FeatureDES            : SubtargetFeature<"des", "m_hasDES", "true",
                                  "The device supports the `DES k` encryption "
                                  "instruction">;

// The device supports the Read-Write-Modify instructions
// XCH, LAS, LAC, and LAT.
def FeatureRMW            : SubtargetFeature<"rmw", "m_supportsRMW", "true",
                                  "The device supports the read-write-modify "
                                  "instructions: XCH, LAS, LAC, LAT">;

// The device supports the `[F]MUL[S][U]` family of instructions.
def FeatureMultiplication : SubtargetFeature<"mul", "m_supportsMultiplication",
                                  "true", "The device supports the "
                                  "multiplication instructions">;

// The device supports the `BREAK` instruction.
def FeatureBREAK          : SubtargetFeature<"break", "m_hasBREAK", "true",
                                  "The device supports the `BREAK` debugging "
                                  "instruction">;

// The device has instruction encodings specific to the Tiny core.
def FeatureTinyEncoding   : SubtargetFeature<"tinyencoding",
                                  "m_hasTinyEncoding", "true",
                                  "The device has Tiny core specific "
                                  "instruction encodings">;

// The device has CPU registers mapped in data address space
def FeatureMMR : SubtargetFeature<"memmappedregs", "m_hasMemMappedGPR",
                                  "true", "The device has CPU registers "
                                  "mapped in data address space">;

class ELFArch<string name>  : SubtargetFeature<"", "ELFArch",
                                    !strconcat("ELF::",name), "">;

// ELF e_flags architecture values
def ELFArchMCS511    : ELFArch<"EF_MCS51_ARCH_MCS511">;
def ELFArchMCS512    : ELFArch<"EF_MCS51_ARCH_MCS512">;
def ELFArchMCS5125   : ELFArch<"EF_MCS51_ARCH_MCS5125">;
def ELFArchMCS513    : ELFArch<"EF_MCS51_ARCH_MCS513">;
def ELFArchMCS5131   : ELFArch<"EF_MCS51_ARCH_MCS5131">;
def ELFArchMCS5135   : ELFArch<"EF_MCS51_ARCH_MCS5135">;
def ELFArchMCS514    : ELFArch<"EF_MCS51_ARCH_MCS514">;
def ELFArchMCS515    : ELFArch<"EF_MCS51_ARCH_MCS515">;
def ELFArchMCS5151   : ELFArch<"EF_MCS51_ARCH_MCS5151">;
def ELFArchMCS516    : ELFArch<"EF_MCS51_ARCH_MCS516">;
def ELFArchTiny    : ELFArch<"EF_MCS51_ARCH_MCS51TINY">;
def ELFArchXMEGA1  : ELFArch<"EF_MCS51_ARCH_XMEGA1">;
def ELFArchXMEGA2  : ELFArch<"EF_MCS51_ARCH_XMEGA2">;
def ELFArchXMEGA3  : ELFArch<"EF_MCS51_ARCH_XMEGA3">;
def ELFArchXMEGA4  : ELFArch<"EF_MCS51_ARCH_XMEGA4">;
def ELFArchXMEGA5  : ELFArch<"EF_MCS51_ARCH_XMEGA5">;
def ELFArchXMEGA6  : ELFArch<"EF_MCS51_ARCH_XMEGA6">;
def ELFArchXMEGA7  : ELFArch<"EF_MCS51_ARCH_XMEGA7">;

//===---------------------------------------------------------------------===//
// MCS51 Families
//===---------------------------------------------------------------------===//

// The device has at least the bare minimum that **every** single MCS51
// device should have.
def FamilyMCS510           : Family<"mcs510", []>;

def FamilyMCS511           : Family<"mcs511", [FamilyMCS510, FeatureLPM, FeatureMMR]>;

def FamilyMCS512           : Family<"mcs512",
                                 [FamilyMCS511, FeatureIJMPCALL, FeatureADDSUBIW,
                                  FeatureSRAM]>;

def FamilyMCS5125          : Family<"mcs5125",
                                 [FamilyMCS512, FeatureMOVW, FeatureLPMX,
                                  FeatureSPM, FeatureBREAK]>;

def FamilyMCS513           : Family<"mcs513",
                                 [FamilyMCS512, FeatureJMPCALL]>;

def FamilyMCS5131          : Family<"mcs5131",
                                 [FamilyMCS513, FeatureELPM]>;

def FamilyMCS5135          : Family<"mcs5135",
                                 [FamilyMCS513, FeatureMOVW, FeatureLPMX,
                                  FeatureSPM, FeatureBREAK]>;

def FamilyMCS514           : Family<"mcs514",
                                 [FamilyMCS512, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK]>;

def FamilyMCS515           : Family<"mcs515",
                                 [FamilyMCS513, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK]>;

def FamilyMCS5151          : Family<"mcs5151",
                                 [FamilyMCS515, FeatureELPM, FeatureELPMX]>;

def FamilyMCS516           : Family<"mcs516",
                                 [FamilyMCS5151]>;

def FamilyTiny           : Family<"mcs51tiny",
                                 [FamilyMCS510, FeatureBREAK, FeatureSRAM,
                                  FeatureTinyEncoding]>;

def FamilyXMEGA          : Family<"xmega",
                                 [FamilyMCS510, FeatureLPM, FeatureIJMPCALL, FeatureADDSUBIW,
                                  FeatureSRAM, FeatureJMPCALL, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK, FeatureEIJMPCALL, FeatureSPMX,
                                  FeatureDES, FeatureELPM, FeatureELPMX]>;

def FamilyXMEGAU         : Family<"xmegau",
                                  [FamilyXMEGA, FeatureRMW]>;

def FeatureSetSpecial    : FeatureSet<"special",
                                      "Enable use of the entire instruction "
                                      "set - used for debugging",
                                      [FeatureSRAM, FeatureJMPCALL,
                                       FeatureIJMPCALL, FeatureEIJMPCALL,
                                       FeatureADDSUBIW, FeatureMOVW,
                                       FeatureLPM, FeatureLPMX, FeatureELPM,
                                       FeatureELPMX, FeatureSPM, FeatureSPMX,
                                       FeatureDES, FeatureRMW,
                                       FeatureMultiplication, FeatureBREAK, FeatureMMR]>;

//===---------------------------------------------------------------------===//
// MCS51 microcontrollers supported.
//===---------------------------------------------------------------------===//

class Device<string Name, Family Fam, ELFArch Arch,
             list<SubtargetFeature> ExtraFeatures = []>
  : Processor<Name, NoItineraries, !listconcat([Fam,Arch],ExtraFeatures)>;

// Generic MCUs
// Note that several versions of GCC has strange ELF architecture
// settings for backwards compatibility - see `gas/config/tc-mcs51.c`
// in MCS51 binutils. We do not replicate this.
def : Device<"mcs511",      FamilyMCS511,    ELFArchMCS511>;
def : Device<"mcs512",      FamilyMCS512,    ELFArchMCS512>;
def : Device<"mcs5125",     FamilyMCS5125,   ELFArchMCS5125>;
def : Device<"mcs513",      FamilyMCS513,    ELFArchMCS513>;
def : Device<"mcs5131",     FamilyMCS5131,   ELFArchMCS5131>;
def : Device<"mcs5135",     FamilyMCS5135,   ELFArchMCS5135>;
def : Device<"mcs514",      FamilyMCS514,    ELFArchMCS514>;
def : Device<"mcs515",      FamilyMCS515,    ELFArchMCS515>;
def : Device<"mcs5151",     FamilyMCS5151,   ELFArchMCS5151>;
def : Device<"mcs516",      FamilyMCS516,    ELFArchMCS516>;
def : Device<"mcs51xmega1", FamilyXMEGA,   ELFArchXMEGA1>;
def : Device<"mcs51xmega2", FamilyXMEGA,   ELFArchXMEGA2>;
def : Device<"mcs51xmega3", FamilyXMEGA,   ELFArchXMEGA3>;
def : Device<"mcs51xmega4", FamilyXMEGA,   ELFArchXMEGA4>;
def : Device<"mcs51xmega5", FamilyXMEGA,   ELFArchXMEGA5>;
def : Device<"mcs51xmega6", FamilyXMEGA,   ELFArchXMEGA6>;
def : Device<"mcs51xmega7", FamilyXMEGA,   ELFArchXMEGA7>;
def : Device<"mcs51tiny",   FamilyTiny,    ELFArchTiny>;

// Specific MCUs
def : Device<"at90s1200",          FamilyMCS510, ELFArchMCS511>;
def : Device<"attiny11",           FamilyMCS511, ELFArchMCS511>;
def : Device<"attiny12",           FamilyMCS511, ELFArchMCS511>;
def : Device<"attiny15",           FamilyMCS511, ELFArchMCS511>;
def : Device<"attiny28",           FamilyMCS511, ELFArchMCS511>;
def : Device<"at90s2313",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90s2323",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90s2333",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90s2343",          FamilyMCS512, ELFArchMCS512>;
def : Device<"attiny22",           FamilyMCS512, ELFArchMCS512>;
def : Device<"attiny26",           FamilyMCS512, ELFArchMCS512, [FeatureLPMX]>;
def : Device<"at86rf401",          FamilyMCS512, ELFArchMCS5125,
             [FeatureMOVW, FeatureLPMX]>;
def : Device<"at90s4414",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90s4433",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90s4434",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90s8515",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90c8534",          FamilyMCS512, ELFArchMCS512>;
def : Device<"at90s8535",          FamilyMCS512, ELFArchMCS512>;
def : Device<"ata5272",            FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny13",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny13a",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny2313",         FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny2313a",        FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny24",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny24a",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny4313",         FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny44",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny44a",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny84",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny84a",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny25",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny45",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny85",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny261",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny261a",         FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny441",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny461",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny461a",         FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny841",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny861",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny861a",         FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny87",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny43u",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny48",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny88",           FamilyMCS5125, ELFArchMCS5125>;
def : Device<"attiny828",          FamilyMCS5125, ELFArchMCS5125>;
def : Device<"at43usb355",         FamilyMCS513,  ELFArchMCS513>;
def : Device<"at76c711",           FamilyMCS513,  ELFArchMCS513>;
def : Device<"atmega103",          FamilyMCS5131, ELFArchMCS5131>;
def : Device<"at43usb320",         FamilyMCS5131, ELFArchMCS5131>;
def : Device<"attiny167",          FamilyMCS5135, ELFArchMCS5135>;
def : Device<"at90usb82",          FamilyMCS5135, ELFArchMCS5135>;
def : Device<"at90usb162",         FamilyMCS5135, ELFArchMCS5135>;
def : Device<"ata5505",            FamilyMCS5135, ELFArchMCS5135>;
def : Device<"atmega8u2",          FamilyMCS5135, ELFArchMCS5135>;
def : Device<"atmega16u2",         FamilyMCS5135, ELFArchMCS5135>;
def : Device<"atmega32u2",         FamilyMCS5135, ELFArchMCS5135>;
def : Device<"attiny1634",         FamilyMCS5135, ELFArchMCS5135>;
def : Device<"atmega8",            FamilyMCS512,  ELFArchMCS514,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"ata6289",            FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega8a",           FamilyMCS512,  ELFArchMCS514,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"ata6285",            FamilyMCS514,  ELFArchMCS514>;
def : Device<"ata6286",            FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega48",           FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega48a",          FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega48pa",         FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega48pb",         FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega48p",          FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega88",           FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega88a",          FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega88p",          FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega88pa",         FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega88pb",         FamilyMCS514,  ELFArchMCS514>;
def : Device<"atmega8515",         FamilyMCS512,  ELFArchMCS514,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8535",         FamilyMCS512,  ELFArchMCS514,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8hva",         FamilyMCS514,  ELFArchMCS514>;
def : Device<"at90pwm1",           FamilyMCS514,  ELFArchMCS514>;
def : Device<"at90pwm2",           FamilyMCS514,  ELFArchMCS514>;
def : Device<"at90pwm2b",          FamilyMCS514,  ELFArchMCS514>;
def : Device<"at90pwm3",           FamilyMCS514,  ELFArchMCS514>;
def : Device<"at90pwm3b",          FamilyMCS514,  ELFArchMCS514>;
def : Device<"at90pwm81",          FamilyMCS514,  ELFArchMCS514>;
def : Device<"ata5790",            FamilyMCS515,  ELFArchMCS515>;
def : Device<"ata5795",            FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16",           FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16a",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega161",          FamilyMCS513,  ELFArchMCS515,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega162",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega163",          FamilyMCS513,  ELFArchMCS515,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega164a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega164p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega164pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega165",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega165a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega165p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega165pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega168",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega168a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega168p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega168pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega168pb",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega169",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega169a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega169p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega169pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32",           FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32a",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega323",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega324a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega324p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega324pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega324pb",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega325",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega325a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega325p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega325pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3250",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3250a",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3250p",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3250pa",       FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega328",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega328p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega328pb",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega329",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega329a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega329p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega329pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3290",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3290a",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3290p",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega3290pa",       FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega406",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega64",           FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega64a",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega640",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega644",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega644a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega644p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega644pa",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega645",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega645a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega645p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega649",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega649a",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega649p",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega6450",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega6450a",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega6450p",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega6490",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega6490a",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega6490p",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega64rfr2",       FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega644rfr2",      FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16hva",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16hva2",       FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16hvb",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16hvbrevb",    FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32hvb",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32hvbrevb",    FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega64hve",        FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90can32",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90can64",          FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90pwm161",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90pwm216",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90pwm316",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32c1",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega64c1",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16m1",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32m1",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega64m1",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega16u4",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32u4",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega32u6",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90usb646",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90usb647",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"at90scr100",         FamilyMCS515,  ELFArchMCS515>;
def : Device<"at94k",              FamilyMCS513,  ELFArchMCS515,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX]>;
def : Device<"m3000",              FamilyMCS515,  ELFArchMCS515>;
def : Device<"atmega128",          FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega128a",         FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega1280",         FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega1281",         FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega1284",         FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega1284p",        FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega128rfa1",      FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega128rfr2",      FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega1284rfr2",     FamilyMCS5151, ELFArchMCS5151>;
def : Device<"at90can128",         FamilyMCS5151, ELFArchMCS5151>;
def : Device<"at90usb1286",        FamilyMCS5151, ELFArchMCS5151>;
def : Device<"at90usb1287",        FamilyMCS5151, ELFArchMCS5151>;
def : Device<"atmega2560",         FamilyMCS516,  ELFArchMCS516>;
def : Device<"atmega2561",         FamilyMCS516,  ELFArchMCS516>;
def : Device<"atmega256rfr2",      FamilyMCS516,  ELFArchMCS516>;
def : Device<"atmega2564rfr2",     FamilyMCS516,  ELFArchMCS516>;
def : Device<"atxmega16a4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega16a4u",       FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16c4",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16d4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4u",       FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32c4",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32d4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32e5",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16e5",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega8e5",         FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32x1",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega64a3",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a3u",       FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64a4u",       FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b1",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b3",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64c3",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64d3",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64d4",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a1",        FamilyXMEGA, ELFArchXMEGA5>;
def : Device<"atxmega64a1u",       FamilyXMEGAU, ELFArchXMEGA5>;
def : Device<"atxmega128a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b1",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128d4",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256a3b",      FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3bu",     FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega384c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega384d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a1",       FamilyXMEGA, ELFArchXMEGA7>;
def : Device<"atxmega128a1u",      FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"atxmega128a4u",      FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"attiny4",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny5",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny9",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny10",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny20",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny40",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny102",          FamilyTiny, ELFArchTiny>;
def : Device<"attiny104",          FamilyTiny, ELFArchTiny>;

