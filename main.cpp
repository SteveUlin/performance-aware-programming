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
#include <cstdint> // uint8_t
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

typedef uint8_t u8;
typedef uint16_t u16;

using namespace std::string_view_literals;

// Names of the registers indexed by the register code.
// The fist element is when the w bit is 0 and the second is when it is 1.
// See pg. 4-20 of the Intel 8086 manual.
constexpr std::array<std::array<std::string_view, 2>, 8> kregister_names = { {
    {"al"sv, "ax"sv}, // 0b000
    {"cl"sv, "cx"sv}, // 0b001
    {"dl"sv, "dx"sv}, // 0b010
    {"bl"sv, "bx"sv}, // 0b011
    {"ah"sv, "sp"sv}, // 0b100
    {"ch"sv, "bp"sv}, // 0b101
    {"dh"sv, "si"sv}, // 0b110
    {"bh"sv, "di"sv}  // 0b111
} };

namespace Instruction {
  class MovRmToFromReg {
  public:
    static bool tryParse(u8*& head, const u8* const end, std::string& asm_str) {
      if (head + 2 > end)
        return false;
      auto* inst = reinterpret_cast<const MovRmToFromReg*>(head);
      if (inst->opcode_ != kOpcode_)
        return false;
      if (inst->mod_ == 0b11) {
        asm_str = inst->GenRegRegAsm();
        head += 2;
        return true;
      }
      if (inst->mod_ == 0b01 && head + 3 > end)
        return false;
      if (inst->mod_ == 0b10 && head + 4 > end)
        return false;
      asm_str = inst->GenEffectiveAddressAsm();
      head += 2 + inst->mod_;
      return true;
    }

  private:
    std::string GenRegRegAsm() const {
      std::string_view reg = kregister_names[reg_][w_];
      std::string_view rm = kregister_names[rm_][w_];
      if (d_)
        return fmt::format("mov {}, {}", reg, rm);
      return fmt::format("mov {}, {}", rm, reg);
    }

    std::string GenEffectiveAddressAsm() const {
      std::string_view reg = kregister_names[reg_][w_];
      std::string effective = std::string(kEffectiveAddress[rm_]);

      if (mod_ == 0b01 && disp_lo_ != 0)
        effective += fmt::format(" + {}", disp_lo_);
      else if (mod_ == 0b10 && (disp_lo_ != 0 || disp_hi_ != 0))
        effective += fmt::format(" + {}", static_cast<u16>(disp_lo_) | (static_cast<u16>(disp_hi_) << 8));

      if (d_)
        return fmt::format("mov {}, [{}]", reg, effective);
      return fmt::format("mov [{}], {}", effective, reg);
    }

    constexpr static u8 kOpcode_ = 0b100010;

    constexpr static std::array<std::string_view, 8> kEffectiveAddress = { {
        "bx + si"sv,
        "bx + di"sv,
        "bp + si"sv,
        "bp + di"sv,
        "si"sv,
        "di"sv,
        "bp"sv,
        "bx"sv,
    } };

    u8 w_ : 1;
    u8 d_ : 1;
    u8 opcode_ : 6;

    u8 rm_ : 3;
    u8 reg_ : 3;
    u8 mod_ : 2;

    u8 disp_lo_ : 8;

    u8 disp_hi_ : 8;
  };

  class MovImmToReg
  {
  public:
    static bool tryParse(u8*& head, const u8* const end, std::string& asm_str) {
      if (head + 2 > end)
        return false;
      auto* inst = reinterpret_cast<const MovImmToReg*>(head);
      if (inst->opcode_ != kOpcode)
        return false;
      if (inst->w_ == 1 && head + 3 > end)
        return false;
      asm_str = inst->GenAsm();
      head += 2 + inst->w_;
      return true;
    }

  private:
    std::string GenAsm() const {
      std::string_view reg = kregister_names[reg_][w_];
      if (w_)
        return fmt::format("mov {}, {}", reg, static_cast<u16>(data0_) | (static_cast<u16>(data1_) << 8));
      return fmt::format("mov {}, {}", reg, data0_);
    }

    constexpr static u8 kOpcode = 0b1011;

    u8 reg_ : 3;
    u8 w_ : 1;
    u8 opcode_ : 4;

    u8 data0_ : 8;
    u8 data1_ : 8;
  };
};

// Disassembles binary machine code to assembly for a Intel 8086 processor
// Returns false on error. Errors are printed to the output stream.
void DissasembleBytes(u8* head, u8* end)
{
  std::string line;
  u8* curr = head;
  while (curr < end) {
    if (Instruction::MovRmToFromReg::tryParse(curr, end, line) ||
      Instruction::MovImmToReg::tryParse(curr, end, line)) {
      std::cout << line << std::endl;
      continue;
    }
    std::cout << fmt::format("Invalid instruction at position: {}: {:b}", std::distance(head, curr), *curr) << std::endl;
    return;
  }
  return;
}

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <file>" << std::endl;
    return 1;
  }
  std::ifstream file(argv[1], std::ios::binary);
  if (!file.is_open()) {
    std::cout << "Could not open file: " << argv[1] << std::endl;
    return 1;
  }
  std::vector<u8> bytes = { std::istreambuf_iterator<char>(file), {} };
  if (!file) {
    std::cout << "Could not read file: " << argv[1] << std::endl;
    return 1;
  }
  file.close();
  DissasembleBytes(bytes.data(), bytes.data() + bytes.size());
  return 0;
}
