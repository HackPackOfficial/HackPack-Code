#pragma once
#include <Arduino.h>

// Index of each display segment is shown below:
//          
//           --- 2 ---       So:    ---------  For character "A"
//          |         |            |         |
//          4         6            |         |
//          |         |            |         |
//           --- 1 ---              ---------       
//          |         |            |         |
//          3         5            |         | is mapped as:
//          |         |            |         | {0, 1, 1, 1, 1, 1, 1}
//           --- 0 ---                          ^
//                                    segment at index 1 is off
// 
//
// The charSets below represent the avaliable characters, and their segment 
// mappings. You can extra characters onto NIXIE's atlas here by appending
// the glyph to the list and then creating its corresponding mapping below.
// 
// Currently, only lowercase chars are mapped to a glyph. In the 7 segment 
// format, some letters like "A" and "R" would appear the same. For glyphs,
// a mix of upper case and lower cases are used to make each character unique.
// Some conflicts and strange characters were resolved as: 
// 0:   _     o:
//     | |         _
//     |_|        |_|
//
// 2:   _     Z:   _ 
//      _|          |
//     |_         |_
//
// 8:   _     b:    
//     |_|        |_
//     |_|        |_|
//
// a:   _     r:  
//     |_|         _  
//     | |        |
// 
// h:         k:         x:   _
//     |_|        |_|         _
//     | |        |_          _
//
// m:   _     n:  
//     | |         _  
//      _         | |
//
// u:         v:         w:
//                |_|        |_|
//     |_|                    _
//





const char NIXIE_CHARSET[] = {
  ' ', '0','1','2','3','4','5','6','7','8','9',
  'a','b','c','d','e','f','g','h','i','j','k',
  'l','m','n','o','p','q','r','s','t','u','v',
  'w','x','y','z','-','_','^','@','#','<','>','O'
};

// Segment order per panel: {bottom, mid, top, L-lower, L-upper, R-lower, R-upper}

const bool NIXIE_CHARSEGMENTS[][7] = {
  {0, 0, 0, 0, 0, 0, 0}, // ' '
  {1, 0, 1, 1, 1, 1, 1}, // '0'
  {0, 0, 0, 0, 0, 1, 1}, // '1'
  {1, 1, 1, 1, 0, 0, 1}, // '2'
  {1, 1, 1, 0, 0, 1, 1}, // '3'
  {0, 1, 0, 0, 1, 1, 1}, // '4'
  {1, 1, 1, 0, 1, 1, 0}, // '5'
  {1, 1, 1, 1, 1, 1, 0}, // '6'
  {0, 0, 1, 0, 0, 1, 1}, // '7'
  {1, 1, 1, 1, 1, 1, 1}, // '8'
  {0, 1, 1, 0, 1, 1, 1}, // '9'
  {0, 1, 1, 1, 1, 1, 1}, // 'a' 
  {1, 1, 0, 1, 1, 1, 0}, // 'b' 
  {1, 0, 1, 1, 1, 0, 0}, // 'c'
  {1, 1, 0, 1, 0, 1, 1}, // 'd'
  {1, 1, 1, 1, 1, 0, 0}, // 'e'
  {0, 1, 1, 1, 1, 0, 0}, // 'f'
  {1, 0, 1, 1, 1, 1, 0}, // 'g
  {0, 1, 0, 1, 1, 1, 1}, // 'h'
  {0, 0, 0, 1, 1, 0, 0}, // 'i'
  {1, 0, 0, 1, 0, 1, 1}, // 'j'
  {1, 1, 0, 1, 1, 0, 1}, // 'k'
  {1, 0, 0, 1, 1, 0, 0}, // 'l'
  {1, 0, 1, 0, 1, 0, 1}, // 'm'
  {0, 1, 0, 1, 0, 1, 0}, // 'n'
  {1, 1, 0, 1, 0, 1, 0}, // 'o'
  {0, 1, 1, 1, 1, 0, 1}, // 'p'
  {0, 1, 1, 0, 1, 1, 1}, // 'q'
  {0, 1, 0, 1, 0, 0, 0}, // 'r'
  {1, 1, 1, 0, 1, 1, 0}, // 's'
  {1, 1, 0, 1, 1, 0, 0}, // 't'
  {1, 0, 0, 1, 0, 1, 0}, // 'u'
  {0, 1, 0, 0, 1, 0, 1}, // 'v'
  {1, 1, 0, 0, 1, 0, 1}, // 'w'
  {1, 1, 1, 0, 0, 0, 0}, // 'x'
  {1, 1, 0, 0, 1, 1, 1}, // 'y'
  {1, 0, 1, 1, 0, 0, 1}, // 'z'
  {0, 1, 0, 0, 0, 0, 0}, // '-'
  {1, 0, 0, 0, 0, 0, 0}, // '_'
  {0, 0, 1, 0, 1, 0, 1}, // '^'
  {0, 0, 1, 0, 1, 0, 0}, // '@'
  {0, 0, 1, 0, 0, 0, 1}, // '#'
  {1, 0, 0, 1, 0, 0, 0}, // '<'
  {1, 0, 0, 0, 0, 1, 0}, // '>'
  {0, 1, 1, 0, 1, 0, 1}  // 'O'
};