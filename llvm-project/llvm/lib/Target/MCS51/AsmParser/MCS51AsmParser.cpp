//===---- MCS51AsmParser.cpp - Parse MCS51 assembly to MCInst instructions ----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCS51.h"
#include "MCS51RegisterInfo.h"
#include "MCTargetDesc/MCS51MCELFStreamer.h"
#include "MCTargetDesc/MCS51MCExpr.h"
#include "MCTargetDesc/MCS51MCTargetDesc.h"
#include "TargetInfo/MCS51TargetInfo.h"

#include "llvm/ADT/APInt.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

#include <sstream>

#define DEBUG_TYPE "MCS51-asm-parser"

using namespace llvm;

namespace {
/// Parses MCS51 assembly from a stream.
class MCS51AsmParser : public MCTargetAsmParser {
  const MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;
  const std::string GENERATE_STUBS = "gs";

#define GET_ASSEMBLER_HEADER
#include "MCS51GenAsmMatcher.inc"

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;
  OperandMatchResultTy tryParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                        SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  OperandMatchResultTy parseMemriOperand(OperandVector &Operands);

  bool parseOperand(OperandVector &Operands);
  int parseRegisterName(unsigned (*matchFn)(StringRef));
  int parseRegisterName();
  int parseRegister(bool RestoreOnFailure = false);
  bool tryParseRegisterOperand(OperandVector &Operands);
  bool tryParseExpression(OperandVector &Operands);
  bool tryParseRelocExpression(OperandVector &Operands);
  void eatComma();

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  unsigned toDREG(unsigned Reg, unsigned From = MCS51::sub_lo) {
    MCRegisterClass const *Class = &MCS51MCRegisterClasses[MCS51::DREGSRegClassID];
    return MRI->getMatchingSuperReg(Reg, From, Class);
  }

  bool emit(MCInst &Instruction, SMLoc const &Loc, MCStreamer &Out) const;
  bool invalidOperand(SMLoc const &Loc, OperandVector const &Operands,
                      uint64_t const &ErrorInfo);
  bool missingFeature(SMLoc const &Loc, uint64_t const &ErrorInfo);

  bool parseLiteralValues(unsigned SizeInBytes, SMLoc L);

public:
  MCS51AsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
               const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), STI(STI), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }

  MCAsmParser &getParser() const { return Parser; }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }
};

/// An parsed MCS51 assembly operand.
class MCS51Operand : public MCParsedAsmOperand {
  typedef MCParsedAsmOperand Base;
  enum KindTy { k_Immediate, k_Register, k_Token, k_Memri } Kind;

public:
  MCS51Operand(StringRef Tok, SMLoc const &S)
      : Base(), Kind(k_Token), Tok(Tok), Start(S), End(S) {}
  MCS51Operand(unsigned Reg, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Register), RegImm({Reg, nullptr}), Start(S), End(E) {}
  MCS51Operand(MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Immediate), RegImm({0, Imm}), Start(S), End(E) {}
  MCS51Operand(unsigned Reg, MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Memri), RegImm({Reg, Imm}), Start(S), End(E) {}

  struct RegisterImmediate {
    unsigned Reg;
    MCExpr const *Imm;
  };
  union {
    StringRef Tok;
    RegisterImmediate RegImm;
  };

  SMLoc Start, End;

public:
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Register && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediate when possible
    if (!Expr)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Immediate && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  /// Adds the contained reg+imm operand to an instruction.
  void addMemriOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Memri && "Unexpected operand kind");
    assert(N == 2 && "Invalid number of operands");

    Inst.addOperand(MCOperand::createReg(getReg()));
    addExpr(Inst, getImm());
  }

  void addImmCom8Operands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    // The operand is actually a imm8, but we have its bitwise
    // negation in the assembly source, so twiddle it here.
    const auto *CE = cast<MCConstantExpr>(getImm());
    Inst.addOperand(MCOperand::createImm(~(uint8_t)CE->getValue()));
  }

  bool isImmCom8() const {
    if (!isImm()) return false;
    const auto *CE = dyn_cast<MCConstantExpr>(getImm());
    if (!CE) return false;
    int64_t Value = CE->getValue();
    return isUInt<8>(Value);
  }

  bool isReg() const override { return Kind == k_Register; }
  bool isImm() const override { return Kind == k_Immediate; }
  bool isToken() const override { return Kind == k_Token; }
  bool isMem() const override { return Kind == k_Memri; }
  bool isMemri() const { return Kind == k_Memri; }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return Tok;
  }

  unsigned getReg() const override {
    assert((Kind == k_Register || Kind == k_Memri) && "Invalid access!");

    return RegImm.Reg;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate || Kind == k_Memri) && "Invalid access!");
    return RegImm.Imm;
  }

  static std::unique_ptr<MCS51Operand> CreateToken(StringRef Str, SMLoc S) {
    return std::make_unique<MCS51Operand>(Str, S);
  }

  static std::unique_ptr<MCS51Operand> CreateReg(unsigned RegNum, SMLoc S,
                                               SMLoc E) {
    return std::make_unique<MCS51Operand>(RegNum, S, E);
  }

  static std::unique_ptr<MCS51Operand> CreateImm(const MCExpr *Val, SMLoc S,
                                               SMLoc E) {
    return std::make_unique<MCS51Operand>(Val, S, E);
  }

  static std::unique_ptr<MCS51Operand>
  CreateMemri(unsigned RegNum, const MCExpr *Val, SMLoc S, SMLoc E) {
    return std::make_unique<MCS51Operand>(RegNum, Val, S, E);
  }

  void makeToken(StringRef Token) {
    Kind = k_Token;
    Tok = Token;
  }

  void makeReg(unsigned RegNo) {
    Kind = k_Register;
    RegImm = {RegNo, nullptr};
  }

  void makeImm(MCExpr const *Ex) {
    Kind = k_Immediate;
    RegImm = {0, Ex};
  }

  void makeMemri(unsigned RegNo, MCExpr const *Imm) {
    Kind = k_Memri;
    RegImm = {RegNo, Imm};
  }

  SMLoc getStartLoc() const override { return Start; }
  SMLoc getEndLoc() const override { return End; }

  void print(raw_ostream &O) const override {
    switch (Kind) {
    case k_Token:
      O << "Token: \"" << getToken() << "\"";
      break;
    case k_Register:
      O << "Register: " << getReg();
      break;
    case k_Immediate:
      O << "Immediate: \"" << *getImm() << "\"";
      break;
    case k_Memri: {
      // only manually print the size for non-negative values,
      // as the sign is inserted automatically.
      O << "Memri: \"" << getReg() << '+' << *getImm() << "\"";
      break;
    }
    }
    O << "\n";
  }
};

} // end anonymous namespace.

// Auto-generated Match Functions

/// Maps from the set of all register names to a register number.
/// \note Generated by TableGen.
static unsigned MatchRegisterName(StringRef Name);

/// Maps from the set of all alternative registernames to a register number.
/// \note Generated by TableGen.
static unsigned MatchRegisterAltName(StringRef Name);

bool MCS51AsmParser::invalidOperand(SMLoc const &Loc,
                                  OperandVector const &Operands,
                                  uint64_t const &ErrorInfo) {
  SMLoc ErrorLoc = Loc;
  char const *Diag = 0;

  if (ErrorInfo != ~0U) {
    if (ErrorInfo >= Operands.size()) {
      Diag = "too few operands for instruction.";
    } else {
      MCS51Operand const &Op = (MCS51Operand const &)*Operands[ErrorInfo];

      // TODO: See if we can do a better error than just "invalid ...".
      if (Op.getStartLoc() != SMLoc()) {
        ErrorLoc = Op.getStartLoc();
      }
    }
  }

  if (!Diag) {
    Diag = "invalid operand for instruction";
  }

  return Error(ErrorLoc, Diag);
}

bool MCS51AsmParser::missingFeature(llvm::SMLoc const &Loc,
                                  uint64_t const &ErrorInfo) {
  return Error(Loc, "instruction requires a CPU feature not currently enabled");
}

bool MCS51AsmParser::emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const {
  Inst.setLoc(Loc);
  Out.emitInstruction(Inst, STI);

  return false;
}

bool MCS51AsmParser::MatchAndEmitInstruction(SMLoc Loc, unsigned &Opcode,
                                           OperandVector &Operands,
                                           MCStreamer &Out, uint64_t &ErrorInfo,
                                           bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);

  switch (MatchResult) {
  case Match_Success:        return emit(Inst, Loc, Out);
  case Match_MissingFeature: return missingFeature(Loc, ErrorInfo);
  case Match_InvalidOperand: return invalidOperand(Loc, Operands, ErrorInfo);
  case Match_MnemonicFail:   return Error(Loc, "invalid instruction");
  default:                   return true;
  }
}

/// Parses a register name using a given matching function.
/// Checks for lowercase or uppercase if necessary.
int MCS51AsmParser::parseRegisterName(unsigned (*matchFn)(StringRef)) {
  StringRef Name = Parser.getTok().getString();

  int RegNum = matchFn(Name);

  // GCC supports case insensitive register names. Some of the MCS51 registers
  // are all lower case, some are all upper case but non are mixed. We prefer
  // to use the original names in the register definitions. That is why we
  // have to test both upper and lower case here.
  if (RegNum == MCS51::NoRegister) {
    RegNum = matchFn(Name.lower());
  }
  if (RegNum == MCS51::NoRegister) {
    RegNum = matchFn(Name.upper());
  }

  return RegNum;
}

int MCS51AsmParser::parseRegisterName() {
  int RegNum = parseRegisterName(&MatchRegisterName);

  if (RegNum == MCS51::NoRegister)
    RegNum = parseRegisterName(&MatchRegisterAltName);

  return RegNum;
}

int MCS51AsmParser::parseRegister(bool RestoreOnFailure) {
  int RegNum = MCS51::NoRegister;

  if (Parser.getTok().is(AsmToken::Identifier)) {
    // Check for register pair syntax
    if (Parser.getLexer().peekTok().is(AsmToken::Colon)) {
      AsmToken HighTok = Parser.getTok();
      Parser.Lex();
      AsmToken ColonTok = Parser.getTok();
      Parser.Lex(); // Eat high (odd) register and colon

      if (Parser.getTok().is(AsmToken::Identifier)) {
        // Convert lower (even) register to DREG
        RegNum = toDREG(parseRegisterName());
      }
      if (RegNum == MCS51::NoRegister && RestoreOnFailure) {
        getLexer().UnLex(std::move(ColonTok));
        getLexer().UnLex(std::move(HighTok));
      }
    } else {
      RegNum = parseRegisterName();
    }
  }
  return RegNum;
}

bool MCS51AsmParser::tryParseRegisterOperand(OperandVector &Operands) {
  int RegNo = parseRegister();

  if (RegNo == MCS51::NoRegister)
    return true;

  AsmToken const &T = Parser.getTok();
  Operands.push_back(MCS51Operand::CreateReg(RegNo, T.getLoc(), T.getEndLoc()));
  Parser.Lex(); // Eat register token.

  return false;
}

bool MCS51AsmParser::tryParseExpression(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();

  if (!tryParseRelocExpression(Operands))
    return false;

  if ((Parser.getTok().getKind() == AsmToken::Plus ||
       Parser.getTok().getKind() == AsmToken::Minus) &&
      Parser.getLexer().peekTok().getKind() == AsmToken::Identifier) {
    // Don't handle this case - it should be split into two
    // separate tokens.
    return true;
  }

  // Parse (potentially inner) expression
  MCExpr const *Expression;
  if (getParser().parseExpression(Expression))
    return true;

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(MCS51Operand::CreateImm(Expression, S, E));
  return false;
}

bool MCS51AsmParser::tryParseRelocExpression(OperandVector &Operands) {
  bool isNegated = false;
  MCS51MCExpr::VariantKind ModifierKind = MCS51MCExpr::VK_MCS51_None;

  SMLoc S = Parser.getTok().getLoc();

  // Check for sign
  AsmToken tokens[2];
  size_t ReadCount = Parser.getLexer().peekTokens(tokens);

  if (ReadCount == 2) {
    if ((tokens[0].getKind() == AsmToken::Identifier &&
         tokens[1].getKind() == AsmToken::LParen) ||
        (tokens[0].getKind() == AsmToken::LParen &&
         tokens[1].getKind() == AsmToken::Minus)) {

      AsmToken::TokenKind CurTok = Parser.getLexer().getKind();
      if (CurTok == AsmToken::Minus ||
          tokens[1].getKind() == AsmToken::Minus) {
        isNegated = true;
      } else {
        assert(CurTok == AsmToken::Plus);
        isNegated = false;
      }

      // Eat the sign
      if (CurTok == AsmToken::Minus || CurTok == AsmToken::Plus)
        Parser.Lex();
    }
  }

  // Check if we have a target specific modifier (lo8, hi8, &c)
  if (Parser.getTok().getKind() != AsmToken::Identifier ||
      Parser.getLexer().peekTok().getKind() != AsmToken::LParen) {
    // Not a reloc expr
    return true;
  }
  StringRef ModifierName = Parser.getTok().getString();
  ModifierKind = MCS51MCExpr::getKindByName(ModifierName.str().c_str());

  if (ModifierKind != MCS51MCExpr::VK_MCS51_None) {
    Parser.Lex();
    Parser.Lex(); // Eat modifier name and parenthesis
    if (Parser.getTok().getString() == GENERATE_STUBS &&
        Parser.getTok().getKind() == AsmToken::Identifier) {
      std::string GSModName = ModifierName.str() + "_" + GENERATE_STUBS;
      ModifierKind = MCS51MCExpr::getKindByName(GSModName.c_str());
      if (ModifierKind != MCS51MCExpr::VK_MCS51_None)
        Parser.Lex(); // Eat gs modifier name
    }
  } else {
    return Error(Parser.getTok().getLoc(), "unknown modifier");
  }

  if (tokens[1].getKind() == AsmToken::Minus ||
      tokens[1].getKind() == AsmToken::Plus) {
    Parser.Lex();
    assert(Parser.getTok().getKind() == AsmToken::LParen);
    Parser.Lex(); // Eat the sign and parenthesis
  }

  MCExpr const *InnerExpression;
  if (getParser().parseExpression(InnerExpression))
    return true;

  if (tokens[1].getKind() == AsmToken::Minus ||
      tokens[1].getKind() == AsmToken::Plus) {
    assert(Parser.getTok().getKind() == AsmToken::RParen);
    Parser.Lex(); // Eat closing parenthesis
  }

  // If we have a modifier wrap the inner expression
  assert(Parser.getTok().getKind() == AsmToken::RParen);
  Parser.Lex(); // Eat closing parenthesis

  MCExpr const *Expression = MCS51MCExpr::create(ModifierKind, InnerExpression,
                                               isNegated, getContext());

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(MCS51Operand::CreateImm(Expression, S, E));

  return false;
}

bool MCS51AsmParser::parseOperand(OperandVector &Operands) {
  LLVM_DEBUG(dbgs() << "parseOperand\n");

  switch (getLexer().getKind()) {
  default:
    return Error(Parser.getTok().getLoc(), "unexpected token in operand");

  case AsmToken::Identifier:
    // Try to parse a register, if it fails,
    // fall through to the next case.
    if (!tryParseRegisterOperand(Operands)) {
      return false;
    }
    LLVM_FALLTHROUGH;
  case AsmToken::LParen:
  case AsmToken::Integer:
  case AsmToken::Dot:
    return tryParseExpression(Operands);
  case AsmToken::Plus:
  case AsmToken::Minus: {
    // If the sign preceeds a number, parse the number,
    // otherwise treat the sign a an independent token.
    switch (getLexer().peekTok().getKind()) {
    case AsmToken::Integer:
    case AsmToken::BigNum:
    case AsmToken::Identifier:
    case AsmToken::Real:
      if (!tryParseExpression(Operands))
        return false;
      break;
    default:
      break;
    }
    // Treat the token as an independent token.
    Operands.push_back(MCS51Operand::CreateToken(Parser.getTok().getString(),
                                               Parser.getTok().getLoc()));
    Parser.Lex(); // Eat the token.
    return false;
  }
  }

  // Could not parse operand
  return true;
}

OperandMatchResultTy
MCS51AsmParser::parseMemriOperand(OperandVector &Operands) {
  LLVM_DEBUG(dbgs() << "parseMemriOperand()\n");

  SMLoc E, S;
  MCExpr const *Expression;
  int RegNo;

  // Parse register.
  {
    RegNo = parseRegister();

    if (RegNo == MCS51::NoRegister)
      return MatchOperand_ParseFail;

    S = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    Parser.Lex(); // Eat register token.
  }

  // Parse immediate;
  {
    if (getParser().parseExpression(Expression))
      return MatchOperand_ParseFail;

    E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  }

  Operands.push_back(MCS51Operand::CreateMemri(RegNo, Expression, S, E));

  return MatchOperand_Success;
}

bool MCS51AsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                 SMLoc &EndLoc) {
  StartLoc = Parser.getTok().getLoc();
  RegNo = parseRegister(/*RestoreOnFailure=*/false);
  EndLoc = Parser.getTok().getLoc();

  return (RegNo == MCS51::NoRegister);
}

OperandMatchResultTy MCS51AsmParser::tryParseRegister(unsigned &RegNo,
                                                    SMLoc &StartLoc,
                                                    SMLoc &EndLoc) {
  StartLoc = Parser.getTok().getLoc();
  RegNo = parseRegister(/*RestoreOnFailure=*/true);
  EndLoc = Parser.getTok().getLoc();

  if (RegNo == MCS51::NoRegister)
    return MatchOperand_NoMatch;
  return MatchOperand_Success;
}

void MCS51AsmParser::eatComma() {
  if (getLexer().is(AsmToken::Comma)) {
    Parser.Lex();
  } else {
    // GCC allows commas to be omitted.
  }
}

bool MCS51AsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                    StringRef Mnemonic, SMLoc NameLoc,
                                    OperandVector &Operands) {
  Operands.push_back(MCS51Operand::CreateToken(Mnemonic, NameLoc));

  bool first = true;
  while (getLexer().isNot(AsmToken::EndOfStatement)) {
    if (!first) eatComma();

    first = false;

    auto MatchResult = MatchOperandParserImpl(Operands, Mnemonic);

    if (MatchResult == MatchOperand_Success) {
      continue;
    }

    if (MatchResult == MatchOperand_ParseFail) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();

      return Error(Loc, "failed to parse register and immediate pair");
    }

    if (parseOperand(Operands)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }
  }
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool MCS51AsmParser::ParseDirective(llvm::AsmToken DirectiveID) {
  StringRef IDVal = DirectiveID.getIdentifier();
  if (IDVal.lower() == ".long") {
    parseLiteralValues(SIZE_LONG, DirectiveID.getLoc());
  } else if (IDVal.lower() == ".word" || IDVal.lower() == ".short") {
    parseLiteralValues(SIZE_WORD, DirectiveID.getLoc());
  } else if (IDVal.lower() == ".byte") {
    parseLiteralValues(1, DirectiveID.getLoc());
  }
  return true;
}

bool MCS51AsmParser::parseLiteralValues(unsigned SizeInBytes, SMLoc L) {
  MCAsmParser &Parser = getParser();
  MCS51MCELFStreamer &MCS51Streamer =
      static_cast<MCS51MCELFStreamer &>(Parser.getStreamer());
  AsmToken Tokens[2];
  size_t ReadCount = Parser.getLexer().peekTokens(Tokens);
  if (ReadCount == 2 && Parser.getTok().getKind() == AsmToken::Identifier &&
      Tokens[0].getKind() == AsmToken::Minus &&
      Tokens[1].getKind() == AsmToken::Identifier) {
    MCSymbol *Symbol = getContext().getOrCreateSymbol(".text");
    MCS51Streamer.emitValueForModiferKind(Symbol, SizeInBytes, L,
            MCS51MCExpr::VK_MCS51_None);
    return false;
  }

  if (Parser.getTok().getKind() == AsmToken::Identifier &&
      Parser.getLexer().peekTok().getKind() == AsmToken::LParen) {
    StringRef ModifierName = Parser.getTok().getString();
    MCS51MCExpr::VariantKind ModifierKind =
        MCS51MCExpr::getKindByName(ModifierName.str().c_str());
    if (ModifierKind != MCS51MCExpr::VK_MCS51_None) {
      Parser.Lex();
      Parser.Lex(); // Eat the modifier and parenthesis
    } else {
      return Error(Parser.getTok().getLoc(), "unknown modifier");
    }
    MCSymbol *Symbol =
        getContext().getOrCreateSymbol(Parser.getTok().getString());
    MCS51Streamer.emitValueForModiferKind(Symbol, SizeInBytes, L, ModifierKind);
    return false;
  }

  auto parseOne = [&]() -> bool {
    const MCExpr *Value;
    if (Parser.parseExpression(Value))
      return true;
    Parser.getStreamer().emitValue(Value, SizeInBytes, L);
    return false;
  };
  return (parseMany(parseOne));
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMCS51AsmParser() {
  RegisterMCAsmParser<MCS51AsmParser> X(getTheMCS51Target());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "MCS51GenAsmMatcher.inc"

// Uses enums defined in MCS51GenAsmMatcher.inc
unsigned MCS51AsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                  unsigned ExpectedKind) {
  MCS51Operand &Op = static_cast<MCS51Operand &>(AsmOp);
  MatchClassKind Expected = static_cast<MatchClassKind>(ExpectedKind);

  // If need be, GCC converts bare numbers to register names
  // It's ugly, but GCC supports it.
  if (Op.isImm()) {
    if (MCConstantExpr const *Const = dyn_cast<MCConstantExpr>(Op.getImm())) {
      int64_t RegNum = Const->getValue();
      std::ostringstream RegName;
      RegName << "r" << RegNum;
      RegNum = MatchRegisterName(RegName.str().c_str());
      if (RegNum != MCS51::NoRegister) {
        Op.makeReg(RegNum);
        if (validateOperandClass(Op, Expected) == Match_Success) {
          return Match_Success;
        }
      }
      // Let the other quirks try their magic.
    }
  }

  if (Op.isReg()) {
    // If the instruction uses a register pair but we got a single, lower
    // register we perform a "class cast".
    if (isSubclass(Expected, MCK_DREGS)) {
      unsigned correspondingDREG = toDREG(Op.getReg());

      if (correspondingDREG != MCS51::NoRegister) {
        Op.makeReg(correspondingDREG);
        return validateOperandClass(Op, Expected);
      }
    }
  }
  return Match_InvalidOperand;
}
