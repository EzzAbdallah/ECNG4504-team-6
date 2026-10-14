// Minimal host stub so maze_map.cpp can be tested on a PC.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#define LED_BUILTIN 13
#define F(x) x
typedef const char __FlashStringHelper;
using std::min; using std::max;
struct Print {
  void print(const char* s){ printf("%s", s);} void print(char c){ printf("%c", c);} 
  void print(long v){ printf("%ld", v);} void print(int v){ printf("%d", v);} 
  void print(unsigned v){ printf("%u", v);} void print(unsigned char v){ printf("%u", v);} 
  void print(unsigned short v){ printf("%u", v);} void print(signed char v){ printf("%d", v);} 
  void println(const char* s=""){ printf("%s\n", s);} void println(unsigned char v){ printf("%u\n", v);} 
};
