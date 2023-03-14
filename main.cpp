/* Dissasembler for Intel 8086 mov instructions.
 * Author: Steven Ulin
 *
 * Initial setup:
 * $ git clone https://github.com/Microsoft/vcpkg.git
 * $ ./vcpkg/bootstrap-vcpkg.sh
 * $ cmake -B build -S .
 *
 * Rebuild and run:
 * $ cmake --build build
 * $ wdiff <(./build/disassemble ./data/many_reg_move) ./data/many_reg_move.asm
 */

// [2024-03-05]: Format is default disabled by the clang++ compiler.
#include <fmt/format.h>

#include <array>
#include <cstdint>  // uint8_t
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

typedef uint8_t u8;
typedef uint16_t u16;

using namespace std::string_view_literals;

// Names of the registers indexed by the register code.
// The fist element is when the w bit is 0 and the second is when it is 1.
// See pg. 4-20 of the Intel 8086 manual.
constexpr std::array<std::array<std::string_view, 2>, 8> kRegisterNames = {{
    {"al"sv, "ax"sv},  // 0b000
    {"cl"sv, "cx"sv},  // 0b001
    {"dl"sv, "dx"sv},  // 0b010
    {"bl"sv, "bx"sv},  // 0b011
    {"ah"sv, "sp"sv},  // 0b100
    {"ch"sv, "bp"sv},  // 0b101
    {"dh"sv, "si"sv},  // 0b110
    {"bh"sv, "di"sv}   // 0b111
}};

constexpr static std::array<std::string_view, 8> kEffectiveAddresses = {{
    "bx + si"sv,
    "bx + di"sv,
    "bp + si"sv,
    "bp + di"sv,
    "si"sv,
    "di"sv,
    "bp"sv,
    "bx"sv,
}};

constexpr static std::array<std::string_view, 8> kArithmaticOps = {{
    "add"sv,  // 0b000
    "or"sv,   // 0b001
    "adc"sv,  // 0b010
    "sbb"sv,  // 0b011
    "and"sv,  // 0b100
    "sub"sv,  // 0b101
    "xor"sv,  // 0b110
    "cmp"sv,  // 0b111
}};

std::string GenAsmRegisterToFromRegister(const std::string_view op,
                                         const bool w, const bool d,
                                         const u8 reg, const u8 rm) {
  std::string_view reg_name = kRegisterNames[reg][w];
  std::string_view rm_name = kRegisterNames[rm][w];
  if (d) return fmt::format("{} {}, {}", op, reg_name, rm_name);
  return fmt::format("{} {}, {}", op, rm_name, reg_name);
}

std::string GenAsmRegisterToFromEffectiveAddress(const std::string_view op,
                                                 const bool w, const bool d,
                                                 const u8 reg, const u8 rm,
                                                 const u16 disp) {
  std::string_view reg_name = kRegisterNames[reg][w];
  std::string_view ea = kEffectiveAddresses[rm];
  std::string offset = "";
  if (disp != 0) {
    offset = fmt::format(" + {}", disp);
  }
  if (d) return fmt::format("{} {}, [{}{}]", op, reg_name, ea, offset);
  return fmt::format("{} [{}{}], {}", op, ea, offset, reg_name);
}

std::string GenAsmRegisterToFromDirectAddress(const std::string_view op,
                                              const bool w, const bool d,
                                              const u8 reg, const u16 disp,
                                              const u16 data) {
  std::string_view reg_name = kRegisterNames[reg][w];
  std::string suffix = "";
  if (data) suffix = fmt::format(" + {}", data);
  if (d) return fmt::format("{} {}, [{}]{}", op, reg_name, disp, suffix);
  return fmt::format("{} [{}]{}, {}", op, disp, suffix, reg_name);
}

std::string GenAsmImmediateToRegister(const std::string_view op, const bool w,
                                      const u8 reg, const u16 data) {
  std::string_view reg_name = kRegisterNames[reg][w];
  return fmt::format("{} {}, {}", op, reg_name, data);
}

std::string GenAsmImmediateToEffectiveAddress(const std::string_view op,
                                              const u8 rm, u16 disp, u16 data) {
  std::string_view ea = kEffectiveAddresses[rm];
  std::string offset = "";
  if (disp != 0) {
    offset = fmt::format(" + {}", disp);
  }
  return fmt::format("{} [{}{}], {}", op, ea, offset, data);
}

std::string GenAsmImmediateToDirectAddress(const std::string_view op,
                                           const u16 disp, const u16 data) {
  return fmt::format("{} [{}], {}", op, disp, data);
}

namespace Instruction {

// Parses instructions that match the pattern:
// [......dw] [mod reg rm] [disp_lo] [disp_hi]

class DWPattern {
 public:
  static bool TryParse(u8*& head, const u8* const end, std::string& asm_str) {
    // Precondition: head + 2 <= end
    auto* inst = reinterpret_cast<const DWPattern*>(head);
    std::optional<std::string_view> op = std::nullopt;
    for (const auto& [opcode_, name] : kOpcodeTable_) {
      if (inst->opcode_ == opcode_) {
        op = name;
        break;
      }
    }
    if (!op.has_value()) return false;
    if (inst->mod_ == 0b00 && inst->rm_ == 0b110) {
      if (head + 2 + 2 > end) return false;
      const u16 addr =
          static_cast<u16>(*(head + 2)) | (static_cast<u16>(*(head + 3)) << 8);
      asm_str = GenAsmRegisterToFromDirectAddress(*op, inst->w_, inst->d_,
                                                  inst->reg_, addr, 0);
      head += 4;
      return true;
    }
    if (inst->mod_ == 0b11) {
      asm_str = GenAsmRegisterToFromRegister(*op, inst->w_, inst->d_,
                                             inst->reg_, inst->rm_);
      head += 2;
      return true;
    }
    if (head + 2 + inst->mod_ > end) return false;
    u16 disp = 0;
    if (inst->mod_ >= 0b01) disp = static_cast<u16>(*(head + 2));
    if (inst->mod_ == 0b10) disp |= static_cast<u16>(*(head + 3)) << 8;
    asm_str = GenAsmRegisterToFromEffectiveAddress(*op, inst->w_, inst->d_,
                                                   inst->reg_, inst->rm_, disp);
    head += 2 + inst->mod_;
    return true;
  }

 private:
  constexpr static auto kOpcodeTable_ =
      std::to_array<std::pair<u8, std::string_view>>({
          {0b000000, "add"sv},
          {0b000010, "or"sv},
          {0b000100, "adc"sv},
          {0b000110, "sbb"sv},
          {0b001000, "and"sv},
          {0b001010, "sub"sv},
          {0b001100, "xor"sv},
          {0b001110, "cmp"sv},
          {0b100010, "mov"sv},
      });

  bool w_ : 1;
  bool d_ : 1;
  u8 opcode_ : 6;

  u8 rm_ : 3;
  u8 reg_ : 3;
  u8 mod_ : 2;
};

class SWPattern {
 public:
  static bool TryParse(u8*& head, const u8* const end, std::string& asm_str) {
    // Precondition: head + 2 <= end
    auto* inst = reinterpret_cast<const SWPattern*>(head);
    if (inst->opcode_ != kOpCode_) return false;
    std::string_view op = kArithmaticOps[inst->op_index_];
    bool two_byte_data = inst->s_ == 0 && inst->w_ == 1;
    if (inst->mod_ == 0b00 && inst->rm_ == 0b110) {
      // Direct Addressing
      if (head + 2 + 2 + 1 + two_byte_data > end) return false;
      u16 addr =
          static_cast<u16>(*(head + 2)) | (static_cast<u16>(*(head + 3)) << 8);
      u16 data = static_cast<u16>(*(head + 4));
      if (two_byte_data) data |= (static_cast<u16>(*(head + 5)) << 8);
      asm_str = GenAsmImmediateToDirectAddress(op, addr, data);
      head += 5 + two_byte_data;
      return true;
    }
    if (inst->mod_ == 0b11) {
      if (head + 2 + 1 + two_byte_data > end) return false;
      u16 data = static_cast<u16>(*(head + 2));
      if (two_byte_data) data |= (static_cast<u16>(*(head + 3)) << 8);
      asm_str = GenAsmImmediateToRegister(op, inst->w_, inst->rm_, data);
      head += 2 + 1 + two_byte_data;
      return true;
    }
    if (head + 2 + inst->mod_ + 1 + two_byte_data > end) return false;
    u16 disp = inst->mod_ >= 0b01 ? static_cast<u16>(*(head + 2)) : 0;
    disp |= inst->mod_ == 0b10 ? static_cast<u16>(*(head + 3)) << 8 : 0;
    u16 data = static_cast<u16>(*(head + 2 + inst->mod_));
    if (two_byte_data)
      data |= (static_cast<u16>(*(head + 3 + inst->mod_)) << 8);
    asm_str = GenAsmImmediateToEffectiveAddress(op, inst->rm_, disp, data);
    head += 2 + inst->mod_ + 1 + two_byte_data;
    return true;
  }

 private:
  constexpr static u8 kOpCode_ = 0b100000;
  bool w_ : 1;
  bool s_ : 1;
  u8 opcode_ : 6;
  u8 rm_ : 3;
  u8 op_index_ : 3;
  u8 mod_ : 2;
};

class MovImmToReg {
 public:
  static bool TryParse(u8*& head, const u8* const end, std::string& asm_str) {
    // Precondition: head + 2 <= end
    auto* inst = reinterpret_cast<const MovImmToReg*>(head);
    if (inst->opcode_ != kOpcode) return false;
    if (inst->w_ == 1 && head + 3 > end) return false;
    asm_str = inst->GenAsm();
    head += 2 + inst->w_;
    return true;
  }

 private:
  std::string GenAsm() const {
    std::string_view reg = kRegisterNames[reg_][w_];
    if (w_)
      return fmt::format(
          "mov {}, {}", reg,
          static_cast<u16>(data0_) | (static_cast<u16>(data1_) << 8));
    return fmt::format("mov {}, {}", reg, data0_);
  }

  constexpr static u8 kOpcode = 0b1011;

  u8 reg_ : 3;
  u8 w_ : 1;
  u8 opcode_ : 4;

  u8 data0_ : 8;
  u8 data1_ : 8;
};

class ImmToAccumulator {
 public:
  static bool TryParse(u8*& head, const u8* const end, std::string& asm_str) {
    // Precondition: head + 2 <= end
    auto* inst = reinterpret_cast<const ImmToAccumulator*>(head);
    if (inst->prefix_ != 0b00 || inst->suffix_ != 0b10) return false;
    if (inst->w_ == 1 && head + 3 > end) return false;
    u16 data = *(head + 1);
    if (inst->w_) data |= (static_cast<u16>(*(head + 2)) << 8);
    std::string_view op = kArithmaticOps[inst->arith_];
    asm_str = GenAsmImmediateToRegister(op, inst->w_, 0 /*reg: al, ax*/, data);
    head += 2 + inst->w_;
    return true;
  }

 private:
  bool w_ : 1;
  u8 suffix_ : 2;
  u8 arith_ : 3;
  u8 prefix_ : 2;
};

class Jumps {
 public:
  static bool TryParse(u8*& head, const u8* const end, std::string& asm_str) {
    // Precondition: head + 2 <= end
    auto* inst = reinterpret_cast<const Jumps*>(head);
    if (inst->opcode_ != kOpCode_) return false;
    int disp = *(head + 1);
    if (disp > 128) disp -= 256;
    asm_str = fmt::format("{} {}", kJumpOps[inst->cond_], disp);
    head += 2;
    return true;
  }

 private:
  constexpr static u8 kOpCode_ = 0b0111;
  constexpr static std::array<std::string_view, 16> kJumpOps = {{
      "jo"sv,
      "jno"sv,
      "jb"sv,
      "jnb"sv,
      "jz"sv,
      "jnz"sv,
      "jbe"sv,
      "ja"sv,
      "js"sv,
      "jns"sv,
      "jpe"sv,
      "jpo"sv,
      "jl"sv,
      "jge"sv,
      "jle"sv,
      "jg"sv,
  }};

  u8 cond_ : 4;
  u8 opcode_ : 4;
};

class Loops {
 public:
  static bool TryParse(u8*& head, const u8* const end, std::string& asm_str) {
    // Precondition: head + 2 <= end
    auto* inst = reinterpret_cast<const Loops*>(head);
    if (inst->opcode_ != kOpCode_) return false;
    if (inst->cond_ >= 4)
      // Not implemented
      return false;
    int disp = *(head + 1);
    if (disp > 128) disp -= 256;
    asm_str = fmt::format("{} {}", kLoopOps[inst->cond_], disp);
    head += 2;
    return true;
  }

 private:
  constexpr static u8 kOpCode_ = 0b1110;
  constexpr static std::array<std::string_view, 4> kLoopOps = {{
      "loopnz"sv,
      "loopz"sv,
      "loop"sv,
      "jcxz"sv,
  }};
  u8 cond_ : 4;
  u8 opcode_ : 4;
};

};  // namespace Instruction

// Disassembles binary machine code to assembly for a Intel 8086 processor
// Returns false on error. Errors are printed to the output stream.
void DissasembleBytes(u8* head, u8* end) {
  std::string line;
  u8* curr = head;
  while (curr + 1 < end) {
    if (Instruction::DWPattern::TryParse(curr, end, line) ||
        Instruction::SWPattern::TryParse(curr, end, line) ||
        Instruction::ImmToAccumulator::TryParse(curr, end, line) ||
        Instruction::MovImmToReg::TryParse(curr, end, line) ||
        Instruction::Jumps::TryParse(curr, end, line) ||
        Instruction::Loops::TryParse(curr, end, line)) {
      std::cout << line << std::endl;
      continue;
    }
    std::cout << fmt::format("Invalid instruction at position: {}.",
                             std::distance(head, curr))
              << std::endl;
    for (int i = 0; i < 5 || head + i == end; ++i) {
      std::cout << fmt::format("{:08b}, ", *(curr + i));
    }
    return;
  }
  return;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <file>" << std::endl;
    return 1;
  }
  std::ifstream file(argv[1], std::ios::binary);
  if (!file.is_open()) {
    std::cout << "Could not open file: " << argv[1] << std::endl;
    return 1;
  }
  std::vector<u8> bytes = {std::istreambuf_iterator<char>(file), {}};
  if (!file) {
    std::cout << "Could not read file: " << argv[1] << std::endl;
    return 1;
  }
  file.close();
  DissasembleBytes(bytes.data(), bytes.data() + bytes.size());
  return 0;
}
