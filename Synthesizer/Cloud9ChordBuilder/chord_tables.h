#ifndef CHORD_TABLES_H
#define CHORD_TABLES_H

#include <Arduino.h>

/*
 * MOOD-FOCUSED CHORD PROGRESSIONS - Beautiful transitions with satisfying resolutions:
 * 
 * CLASSIC (Perfect for any song):
 * • Pop foundation: 0→1→4→0 (Major→minor→sus4→Major) - timeless progression
 * • Sophisticated: 2→3→7→0 (Maj7→m7→M6→Major) - smooth jazz-pop feel
 * • Emotional arc: 1→4→6→7→0 (minor→sus4→add9→M6→Major) - builds to resolution
 * 
 * MINOR PENTATONIC (Soulful expression):
 * • Blues walk: 0→1→3→5→7 (Am→Am7→sus4→Am9→drone) - classic blues progression
 * • Soulful journey: 1→5→6→0→7 (Am7→Am9→jazzy→Am→drone) - emotional depth
 * • Meditative: 7→2→4→0→7 (drone→5ths→sus2→Am→drone) - circular peace
 * 
 * MELANCHOLY (Beautiful sadness):
 * • Hope through darkness: 0→1→2→5→0 (minor→Major→m7→M6→minor) - bittersweet
 * • Emotional contrast: 4→1→3→6→7 (dim→Major→Maj7→m9→sus2) - complex feelings
 * • Acceptance: 0→6→1→7 (minor→m9→Major→sus2) - finding peace
 * 
 * ANGELIC (Heavenly textures):
 * • Ascending to heaven: 0→1→2→3→7 (Maj7→5ths→M6→bells→drone) - pure ascension
 * • Floating dreams: 4→5→6→7 (sus2→add9→m9→drone) - weightless beauty
 * • Celestial cycle: 7→3→1→0 (drone→bells→5ths→Maj7) - eternal peace
 * 
 * UPLIFTING (Pure joy):
 * • Building excitement: 0→1→3→4→7 (Major→add9→sus2→Maj7→6/9) - crescendo of joy
 * • Perfect resolution: 5→4→2→7 (sus4→Maj7→M6→6/9) - ultimate satisfaction
 * • Celebration: 6→1→0→7 (aug→add9→Major→6/9) - sparkling happiness
 */

// ===== MOOD-FOCUSED CHORD TABLES =====

// Table 0: Classic Pop/Rock - The foundation with perfect resolution points
const static int8_t chord_table_classic[8][4] PROGMEM = {
 { 0,  4,  7, 12 },  // 0: Major (I) - strong foundation & resolution
 { 0,  3,  7, 12 },  // 1: minor (vi) - emotional depth
 { 0,  4,  7, 11 },  // 2: Major 7th (IMaj7) - sophisticated resolution
 { 0,  3,  7, 10 },  // 3: minor 7th (vim7) - smooth, complete feeling
 { 0,  5,  7, 12 },  // 4: sus4 (Isus4) - perfect tension that wants to resolve
 { 0,  2,  7, 12 },  // 5: sus2 (Isus2) - open, floating, but stable
 { 0,  4,  7, 14 },  // 6: Major add9 (Iadd9) - colorful but complete
 { 0,  4,  7,  9 }   // 7: Major 6th (I6) - warm, satisfying resolution
};

// Table 1: Minor Pentatonic Walk - Soulful, bluesy, emotional pentatonic harmony
const static int8_t chord_table_minor_pentatonic[8][4] PROGMEM = {
 { 0, 10, 15, 22 },  // 0: 7th and 9th (A-G-B-G) - jazzy pentatonic PEAK
 { 0, 12, 19, 24 },  // 1: octave + 5th drone (A-A-D-A) - meditative peak
 { 0,  7, 15, 19 },  // 2: 5th add9 (A-E-B-D) - harmonic richness
 { 0,  3, 10, 12 },  // 3: minor 7th (Am7: A-C-G-A) - bluesy descent
 { 0,  5,  7, 12 },  // 4: sus4 (Asus4: A-D-E-A) - tension
 { 0,  2,  7, 12 },  // 5: sus2 (Asus2: A-B-E-A) - floating
 { 0,  7, 12, 15 },  // 6: perfect 5ths (A-E-A-C) - open sound
 { 0,  3,  7, 12 }   // 7: minor triad (Am: A-C-E-A) - ROOT foundation
};

// Table 2: Melancholy - Beautiful sadness with satisfying resolutions
const static int8_t chord_table_melancholy[8][4] PROGMEM = {
 { 0,  3,  7, 14 },  // 0: minor add9 (im9) - dreamy melancholy PEAK
 { 0,  3,  7, 10 },  // 1: minor 7th (im7) - bittersweet descent
 { 0,  3,  6, 12 },  // 2: diminished (i°) - deeper sadness
 { 0,  4,  7, 11 },  // 3: Major 7th (IMaj7) - sophisticated hope
 { 0,  4,  7,  9 },  // 4: Major 6th (I6) - warm resolution
 { 0,  3,  7, 12 },  // 5: minor (i) - foundation of sadness
 { 0,  2,  7, 12 },  // 6: sus2 (sus2) - peaceful acceptance
 { 0,  4,  7, 12 }   // 7: Major (I) - hope breaking through ROOT
};

// Table 3: Dreamy/Angelic - Ethereal, heavenly, floating harmonies
const static int8_t chord_table_angelic[8][4] PROGMEM = {
 { 0, 12, 19, 24 },  // 0: octave + 5th drone (celestial) - ultimate PEAK
 { 0,  7, 14, 21 },  // 1: perfect 5ths stacked (pure) - dramatic peak
 { 0,  4, 11, 16 },  // 2: Major 7th + octave (heavenly descent)
 { 0,  5, 12, 17 },  // 3: perfect 4ths (church bells)
 { 0,  4,  9, 16 },  // 4: Major 6th + octave (sweet)
 { 0,  7, 12, 19 },  // 5: perfect 5ths (building)
 { 0,  2,  7, 14 },  // 6: sus2 add9 (floating)
 { 0,  4,  7, 14 }   // 7: Major add9 (bright) - ROOT resolution
};

// Table 4: Uplifting - Joyful, energetic, with strong resolution points
const static int8_t chord_table_uplifting[8][4] PROGMEM = {
 { 0,  4, 10, 14 },  // 0: 6/9 chord - ultimate happy PEAK
 { 0,  4,  7, 14 },  // 1: Major add9 (Iadd9) - sparkling happiness peak
 { 0,  2,  7, 14 },  // 2: sus2 add9 - floating optimism
 { 0,  4,  7, 11 },  // 3: Major 7th (IMaj7) - sophisticated descent
 { 0,  4,  8, 12 },  // 4: augmented (I+) - bright, upward energy
 { 0,  5,  7, 12 },  // 5: sus4 (Isus4) - anticipation
 { 0,  4,  7,  9 },  // 6: Major 6th (I6) - warm satisfaction
 { 0,  4,  7, 12 }   // 7: Major (I) - pure joy ROOT
};

// Table 1: Innocent Love - Smooth voice leading with gentle movement
// Key: C Major - Classic I-vi-IV-V progression with extensions for warmth
const static int8_t chord_table_innocent[8][4] PROGMEM = {
  { 9, 12, 16, 21 },  // 0: Am (A-C-E-A) - relative minor PEAK
  { 0,  4,  7, 14 },  // 1: Cadd9 (C-E-G-D) - home with color peak
  { 7, 11, 14, 19 },  // 2: G (G-B-D-G) - V chord building
  { 9, 12, 16, 19 },  // 3: Am7 (A-C-E-G) - gentle descent
  { 5,  9, 12, 17 },  // 4: F (F-A-C-F) - IV chord warmth
  { 5,  9, 12, 14 },  // 5: F6 (F-A-C-D) - gentle color
  { 0,  4,  9, 12 },  // 6: C6 (C-E-A-C) - gentle color
  { 0,  4,  7, 12 }   // 7: C Major (C-E-G-C) - pure foundation ROOT
};

// Table 2: Beautiful Sorrow - Smooth minor progression with hope
// Key: A minor - Natural minor with gentle voice leading and major contrast
const static int8_t chord_table_sorrow[8][4] PROGMEM = {

  { 9, 12, 16, 21 },  // 0: Am (A-C-E-A) - peak of sorrow
  { 9, 12, 16, 19 },  // 5: Am7 (A-C-E-G) - melancholy with warmth
  { 9, 11, 16, 19 },  // 6: Am7 (A-C-E-G) - peaceful return
  { 7, 11, 14, 19 },  // 2: G (G-B-D-G) - major VII building
  { 5,  9, 12, 17 },   // 7: F (F-A-C-F) - major VI hope resolution
  { 4,  7, 11, 16 },  // 1: Em (E-G-B-E) - v chord, descent
  { 2,  5,  9, 14 },  // 3: Dm7 (D-F-A-C) - subdominant warmth
  { 0,  5,  9, 12 },  // 4: Dm (D-F-A-C) - iv chord center
};

// Table 3: Victorious Triumph - Rising progression from minor to major victory
// Key: A minor to C Major - Ascending emotional arc with smooth voice leading
const static int8_t chord_table_triumph[8][4] PROGMEM = {
  { 9, 12, 16, 21 },  // 0: Am (A-C-E-A) - peak of struggle PEAK
  { 7, 11, 14, 19 },  // 1: G (G-B-D-G) - rising power peak
  { 5,  9, 12, 17 },  // 4: F (F-A-C-F) - strength building
  { 5,  9, 12, 16 },  // 6: F (F-A-C-E) - celebration
  { 4,  7, 11, 16 },  // 3: Em (E-G-B-E) - final push descent
  { 0,  5,  9, 12 },  // 5: Dm (D-F-A-C) - determination
  { 0,  4,  7, 11 },  // 2: CMaj7 (C-E-G-B) - victorious resolution
  { 0,  4,  7, 12 },   // 7: C (C-E-G-C) - major triumph ROOT
};

// Table 4: Mysterious Epic - Smooth chromatic descent with modal color
// Key: D minor - Dark progression with chromatic voice leading
const static int8_t chord_table_mysterious[8][4] PROGMEM = {
  { 9, 12, 16, 21 },  // 0: Am (A-C-E-A) - v chord, peak of mystery PEAK
  { 2,  5,  9, 14 },  // 1: Dm (D-F-A-D) - dark foundation peak
  { 1,  5,  8, 13 },  // 2: DbMaj7 (Db-F-Ab-C) - chromatic step
  { 10, 1,  5, 10 },  // 3: BbMaj7 (Bb-D-F-A) - bVI, chromatic mystery
  { 7, 10, 14, 19 },  // 4: Gm (G-Bb-D-G) - iv chord
  { 5,  9, 12, 17 },  // 5: F (F-A-C-F) - bIII
  { 0,  4,  7, 12 },  // 6: C (C-E-G-C) - bVII
  { 2,  5,  9, 12 }   // 7: Dm (D-F-A-C) - mysterious return ROOT
};

// Table 5: Meditative Bliss - Gentle floating progression with smooth voice leading
// Key: C Major - Peaceful I-vi-IV-V with extensions for transcendence
const static int8_t chord_table_meditative[8][4] PROGMEM = {
  { 9, 12, 16, 21 },  // 0: Am (A-C-E-A) - peaceful minor PEAK START
  { 9, 12, 16, 19 },  // 1: Am7 (A-C-E-G) - gentle descent
  { 7, 11, 14, 16 },  // 6: G6 (G-B-D-E) - gentle V building to transcendence
  { 5,  9, 12, 17 },  // 2: F (F-A-C-F) - stable IV warmth
  { 5,  9, 12, 14 },  // 4: F6 (F-A-C-D) - warm IV with color CENTER
  { 0,  4,  7, 12 },  // 3: C (C-E-G-C) - pure simplicity CENTER
  { 0,  4,  7, 11 },   // 7: CMaj7 (C-E-G-B) - transcendent resolution PEAK END
  { 0,  4,  7, 14 },  // 5: Cadd9 (C-E-G-D) - peaceful with color
};

// Table 6: Nostalgic Warmth - Gentle jazz progressions with rich extensions
// Key: C Major - Warm jazz harmony with smooth voice leading and nostalgic feel
const static int8_t chord_table_nostalgic[8][4] PROGMEM = {
  { 0,  4, 11, 14 },  // 0: CMaj9 (C-E-B-D) - warm nostalgic PEAK
  { 9, 12, 16, 19 },  // 1: Am7 (A-C-E-G) - gentle minor warmth
  { 2,  5,  9, 12 },  // 2: Dm7 (D-F-A-C) - subdominant jazz
  { 7, 11, 14, 17 },  // 3: G7 (G-B-D-F) - dominant color
  { 4,  7, 11, 14 },  // 4: Em7 (E-G-B-D) - relative minor
  { 5,  9, 12, 15 },  // 5: FMaj7 (F-A-C-E) - IV with color
  { 0,  4,  9, 12 },  // 6: C6 (C-E-A-C) - gentle resolution
  { 0,  4,  7, 12 }   // 7: C Major (C-E-G-C) - home ROOT
};

// Table 7: Ethereal Dreams - Floating ambient progressions with otherworldly beauty
// Modal - Suspended and open harmonies that create a sense of weightless wonder
const static int8_t chord_table_ethereal[8][4] PROGMEM = {
  { 0,  7, 19, 24 },  // 0: Perfect 5ths + octaves (C-G-G-C) - cosmic PEAK
  { 0,  5, 12, 17 },  // 1: Perfect 4ths (C-F-C-F) - church bells
  { 0,  2,  9, 16 },  // 2: Sus2 + 6th (C-D-A-E) - floating mystery
  { 0,  4, 11, 18 },  // 3: Maj7 + 11th (C-E-B-F#) - bright suspension
  { 0,  3,  7, 15 },  // 4: Minor add9 (C-Eb-G-D) - gentle melancholy
  { 0,  5,  7, 14 },  // 5: Sus4 add9 (C-F-G-D) - open suspension
  { 0,  2,  7, 12 },  // 6: Sus2 (C-D-G-C) - peaceful floating
  { 0,  4,  7, 12 }   // 7: Major triad (C-E-G-C) - pure resolution ROOT
};

// Table 8: Endless Ascension - Continuously rising with rich extensions
// Modal - Each chord higher than the last, exploring complex upper structures
const static int8_t chord_table_ascension[8][4] PROGMEM = {
  { 0,  3,  7, 10 },  // 0: Cm7 (C-Eb-G-Bb) - dark foundation
  { 2,  5,  9, 12 },  // 1: Dm7 (D-F-A-C) - step up, shared C
  { 4,  7, 11, 14 },  // 2: Em7 (E-G-B-D) - continuing climb
  { 5,  9, 12, 16 },  // 3: FMaj7 (F-A-C-E) - major brightness
  { 7, 11, 14, 17 },  // 4: GMaj7 (G-B-D-F) - higher still
  { 9, 12, 16, 19 },  // 5: Am7 (A-C-E-G) - ascending through octave
  { 11, 14, 18, 21 }, // 6: BMaj7 (B-D-F#-A) - approaching ultimate peak
  { 12, 16, 19, 24 }  // 7: CMaj9 (C-E-G-D+octave) - ultimate ascension peak
};

// Table 9: World Domination - Ascending power with exotic modal richness
// Key: D minor Hungarian - Commanding progression with complex extensions
const static int8_t chord_table_domination[8][4] PROGMEM = {
  { 12, 15, 19, 24 }, // 0: CmMaj9 (C-Eb-G-B+octave) - ultimate exotic power PEAK
  { 10, 14, 17, 21 }, // 1: BbMaj9 (Bb-D-F-A) - rich descent
  { 9, 12, 16, 20 },  // 2: AmMaj7 (A-C-E-G#) - ascending power
  { 7, 11, 14, 18 },  // 3: GMaj7#11 (G-B-D-F#) - Lydian brightness
  { 6, 10, 13, 17 },  // 4: F#dim7 (F#-A-C#-E) - diminished tension
  { 5,  8, 12, 15 },  // 5: FmMaj7 (F-Ab-C-E) - minor with major 7th
  { 3,  7, 10, 14 },  // 6: EbMaj7 (Eb-G-Bb-D) - Hungarian bII
  { 2,  5,  9, 13 }   // 7: Dm6 (D-F-A-B) - exotic minor foundation ROOT
};

// Array of pointers to chord tables for easy switching
const int8_t (*chord_tables[11])[4] = {
  chord_table_classic,          // 0: Classic Pop/Rock (original)
  chord_table_innocent,         // 1: Innocent Love - smooth peak-to-root progression
  chord_table_sorrow,           // 2: Beautiful Sorrow - emotional peak-to-root journey
  chord_table_triumph,          // 3: Victorious Triumph - struggle to victory arc
  chord_table_mysterious,       // 4: Mysterious Epic - dark modal progression
  chord_table_meditative,       // 5: Meditative Bliss - peaceful peak-to-root flow
  chord_table_nostalgic,        // 6: Nostalgic Warmth - jazz harmony warmth
  chord_table_minor_pentatonic, // 7: Minor Pentatonic Walk - soulful blues progression
  chord_table_ethereal,         // 8: Ethereal Dreams - floating ambient beauty
  chord_table_ascension,        // 9: Endless Ascension
  chord_table_domination        // 10: World Domination
};

// Table names for display/debugging
const char* table_names[] = {
  "Classic", "Innocent", "Sorrow", "Triumph", "Mysterious", 
  "Meditative", "Nostalgic", "MinorPenta", "Ethereal", "Ascension", "Domination"
};

#endif // CHORD_TABLES_H
