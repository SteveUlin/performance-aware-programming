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
#include <span>
#include <vector>

typedef uint8_t u8;

using namespace std::string_view_literals;

// Names of the registers indexed by the register code.
// The fist element is when the w bit is 0 and the second is when it is 1.
// See pg. 4-20 of the Intel 8086 manual.
constexpr std::array<std::array<std::string_view, 2>, 8> kregister_names = {{
    {"al"sv, "ax"sv}, // 0b000
    {"cl"sv, "cx"sv}, // 0b001
    {"dl"sv, "dx"sv}, // 0b010
    {"bl"sv, "bx"sv}, // 0b011
    {"ah"sv, "sp"sv}, // 0b100
    {"ch"sv, "bp"sv}, // 0b101
    {"dh"sv, "si"sv}, // 0b110
    {"bh"sv, "di"sv}  // 0b111
}};

namespace Instruction
{
  // Two byte Reg to Reg MOV instruction.
  struct RegToReg_MOV2
  {
    u8 w : 1;
    u8 d : 1;
    u8 opcode : 6;

    u8 rm : 3;
    u8 reg : 3;
    u8 mod : 2;

    bool isValid()
    {
      return this->opcode == 0b100010 && this->mod == 0b11;
    }

    std::string GenAsm()
    {
      std::string_view reg = kregister_names[this->reg][this->w];
      std::string_view rm = kregister_names[this->rm][this->w];
      if (d)
        return fmt::format("mov {}, {}", reg, rm);
      return fmt::format("mov {}, {}", rm, reg);
    }
  };

};

// Disassembles binary machine code to assembly for a Intel 8086 processor
// Returns false on error. Errors are printed to the output stream.
void DissasembleBytes(u8 *head, u8 *end)
{
  while (head < end)
  {
    auto *inst = reinterpret_cast<Instruction::RegToReg_MOV2 *>(head);
    if (inst->isValid())
    {
      std::cout << inst->GenAsm() << std::endl;
      head += 2;
    }
    else
    {
      std::cout << fmt::format("Invalid instruction: {}b", *head) << std::endl;
      return;
    }
  }
  return;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: " << argv[0] << " <file>" << std::endl;
    return 1;
  }
  std::ifstream file(argv[1], std::ios::binary);
  if (!file.is_open())
  {
    std::cout << "Could not open file: " << argv[1] << std::endl;
    return 1;
  }
  std::vector<u8> bytes = {std::istreambuf_iterator<char>(file), {}};
  if (!file)
  {
    std::cout << "Could not read file: " << argv[1] << std::endl;
    return 1;
  }
  file.close();
  DissasembleBytes(bytes.data(), bytes.data() + bytes.size() - 3);
}
