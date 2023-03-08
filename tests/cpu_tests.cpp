#include <gtest/gtest.h>
#include "../src/cpu.h"

// Demonstrate some basic assertions.
TEST(CPUTests, lda_0xa9_immediate_load_data) {
  CPU cpu = CPU();
  std::vector<uint8_t> program {0xa9, 0x05, 0x00};
  cpu.load_and_run(program);

  EXPECT_EQ(cpu.register_a, 0x05);
}

TEST(CPUTests, ops_working_together) {
  auto cpu = CPU();
  std::vector<uint8_t> program {0xa9, 0xc0,  0xaa, 0xe8, 0x00};
  cpu.load_and_run(program);

  EXPECT_EQ(cpu.register_x, 0xc1);
}

TEST(CPUTests, inx_overflow) {
  CPU cpu = CPU();
  std::vector<uint8_t> program {0xe8, 0xe8, 0x00};
  cpu.load(program);
  cpu.reset();
  cpu.register_x = 0xff;
  cpu.run();

  EXPECT_EQ(cpu.register_x, 1);
}