#ifndef DRUM_SEQUENCES_H
#define DRUM_SEQUENCES_H

// All drum patterns from XY Controller
// Compatible with Mozzi drum sequencer format
// Enabled instruments: kick, snare, hat

// ROCK/DANCE GENRE (Med Tempo)
const bool kickPatternMed1[16] PROGMEM = {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0}; // Classic 4/4 rock
const bool kickPatternMed2[16] PROGMEM = {1,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0}; // Rock with extra kick
const bool kickPatternMed3[16] PROGMEM = {1,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0}; // Funky rock
const bool kickPatternMed4[16] PROGMEM = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}; // Dance 8th notes
const bool kickPatternMed5[16] PROGMEM = {1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0}; // Complex dance

const bool snarePatternMed1[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // No snare (minimal house)
const bool snarePatternMed2[16] PROGMEM = {0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0}; // Classic rock 2&4
const bool snarePatternMed3[16] PROGMEM = {0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0}; // House with extra
const bool snarePatternMed4[16] PROGMEM = {0,0,1,0,1,0,1,0,0,0,1,0,1,0,1,0}; // Funky rock groove
const bool snarePatternMed5[16] PROGMEM = {0,1,1,0,1,0,1,1,0,1,1,0,1,0,1,1}; // Dance/disco fill

// ROCK/DANCE - Hi-hat patterns
const bool hihatPatternMed1[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // No hi-hats (minimal house)
const bool hihatPatternMed2[16] PROGMEM = {0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0}; // Simple rock off-beat
const bool hihatPatternMed3[16] PROGMEM = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}; // Classic 8th note rock
const bool hihatPatternMed4[16] PROGMEM = {1,1,0,1,1,0,1,1,1,1,0,1,1,0,1,1}; // Dance/house groove
const bool hihatPatternMed5[16] PROGMEM = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // Full dance energy

// const bool hihatPatternMed1[16] PROGMEM = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
// const bool hihatPatternMed2[16] PROGMEM = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
// const bool hihatPatternMed3[16] PROGMEM = {1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1};
// const bool hihatPatternMed4[16] PROGMEM = {1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1};
// const bool hihatPatternMed5[16] PROGMEM = {1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,1};

// DnB/BREAKBEAT GENRE (Fast Tempo)
const bool kickPatternFast1[16] PROGMEM = {1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0}; // Minimal DnB
const bool kickPatternFast2[16] PROGMEM = {1,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0}; // DnB groove
const bool kickPatternFast3[16] PROGMEM = {1,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0}; // Breakbeat style
const bool kickPatternFast4[16] PROGMEM = {1,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0}; // Complex breaks
const bool kickPatternFast5[16] PROGMEM = {1,0,1,1,0,1,1,1,1,0,1,0,1,1,1,0}; // Jungle madness

const bool snarePatternFast1[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // No snare (just breaks)
const bool snarePatternFast2[16] PROGMEM = {0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0}; // Minimal DnB snare
const bool snarePatternFast3[16] PROGMEM = {0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0}; // DnB groove
const bool snarePatternFast4[16] PROGMEM = {0,0,1,1,0,1,0,0,0,1,1,1,0,1,0,0}; // Breakbeat rolls
const bool snarePatternFast5[16] PROGMEM = {0,1,1,0,0,1,0,0,0,1,0,1,0,1,0,0}; // Jungle snare chaos

const bool hihatPatternFast1[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const bool hihatPatternFast2[16] PROGMEM = {0,0,1,0,0,0,0,1,0,0,0,0,0,1,1,0};
const bool hihatPatternFast3[16] PROGMEM = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
const bool hihatPatternFast4[16] PROGMEM = {1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1};
const bool hihatPatternFast5[16] PROGMEM = {0,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1};

// const bool hihatPatternFast1[16] PROGMEM = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
// const bool hihatPatternFast2[16] PROGMEM = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
// const bool hihatPatternFast3[16] PROGMEM = {1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1};
// const bool hihatPatternFast4[16] PROGMEM = {1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1};
// const bool hihatPatternFast5[16] PROGMEM = {1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1};

// HIP HOP/TRAP GENRE (Slow Tempo)
const bool kickPatternSlow1[16] PROGMEM = {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0}; // Minimal trap
const bool kickPatternSlow2[16] PROGMEM = {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0}; // Classic hip hop
const bool kickPatternSlow3[16] PROGMEM = {1,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0}; // Latin clave
const bool kickPatternSlow4[16] PROGMEM = {1,0,0,1,0,0,1,0,1,0,0,1,0,1,1,0}; // Complex Latin
const bool kickPatternSlow5[16] PROGMEM = {1,0,1,1,0,1,1,0,1,0,1,1,0,1,1,0}; // Trap madness

const bool snarePatternSlow1[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // No snare (just kick/bass)
const bool snarePatternSlow2[16] PROGMEM = {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0}; // Minimal (just beat 2)
const bool snarePatternSlow3[16] PROGMEM = {0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0}; // Classic 2 and 4
const bool snarePatternSlow4[16] PROGMEM = {0,0,0,0,1,0,0,1,0,0,0,0,1,0,0,1}; // Hip hop ghosts
const bool snarePatternSlow5[16] PROGMEM = {0,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1}; // Trap rolls

// HIP HOP/TRAP - Hi-hat patterns
const bool hihatPatternSlow1[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // No hi-hats
const bool hihatPatternSlow2[16] PROGMEM = {0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0}; // Sparse trap style
const bool hihatPatternSlow3[16] PROGMEM = {1,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0}; // Hip hop groove
const bool hihatPatternSlow4[16] PROGMEM = {1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,1}; // Complex trap
const bool hihatPatternSlow5[16] PROGMEM = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // Full trap rolls

// const bool hihatPatternSlow1[16] PROGMEM = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
// const bool hihatPatternSlow2[16] PROGMEM = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
// const bool hihatPatternSlow3[16] PROGMEM = {1,1,1,1,0,1,1,0,1,1,1,1,0,1,1,0};
// const bool hihatPatternSlow4[16] PROGMEM = {1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1};
// const bool hihatPatternSlow5[16] PROGMEM = {1,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1};

// Bass timing patterns - when bass notes should trigger
// SLOW TEMPO - Bass timing patterns
const bool bassTimingSlow1[16] PROGMEM = {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0}; // Root on beats
const bool bassTimingSlow2[16] PROGMEM = {1,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0}; // Syncopated
const bool bassTimingSlow3[16] PROGMEM = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}; // 8th notes
const bool bassTimingSlow4[16] PROGMEM = {1,0,0,1,0,1,0,0,1,0,0,1,0,1,0,0}; // Off-beat emphasis
const bool bassTimingSlow5[16] PROGMEM = {1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1}; // Complex rhythm

// MEDIUM TEMPO - Bass timing patterns
const bool bassTimingMed1[16] PROGMEM = {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0}; // Root on beats
const bool bassTimingMed2[16] PROGMEM = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}; // 8th notes
const bool bassTimingMed3[16] PROGMEM = {1,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0}; // Syncopated
const bool bassTimingMed4[16] PROGMEM = {1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1}; // House groove
const bool bassTimingMed5[16] PROGMEM = {1,1,0,1,0,1,0,0,1,1,0,1,0,1,0,0}; // Driving rhythm

// FAST TEMPO - Bass timing patterns
const bool bassTimingFast1[16] PROGMEM = {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0}; // Root on beats
const bool bassTimingFast2[16] PROGMEM = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}; // 8th notes
const bool bassTimingFast3[16] PROGMEM = {1,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1}; // DnB style
const bool bassTimingFast4[16] PROGMEM = {1,0,0,1,1,0,1,0,1,0,0,1,1,0,1,0}; // Breakbeat
const bool bassTimingFast5[16] PROGMEM = {1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1}; // Intense rhythm

#endif