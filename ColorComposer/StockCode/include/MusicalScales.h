#ifndef MUSICAL_SCALES_H
#define MUSICAL_SCALES
#include <MusicTools.h>

// This uses a macro that makes it easy to define a new Chord object. Chords in the context of this program are just collections
// of note data. They can be up to 32 notes, and you can use them as chords, scales, arpeggios, melodies, or however you want.
// See MusicTools.h to see how this macro is constructed.

// C Pentatonic Major - A five-note scale derived from the major scale by omitting the fourth 
// and seventh degrees (C, D, E, G, A), widely used in pop, country, and blues.
DEFINE_CHORD(scale_CPentatonicMajor, "C3", "D3", "E3", "G3", "A3");

// C Harmonic Minor - A seven-note hybrid scale combining the major scale with a lowered sixth 
// degree (C, D, E, F, G, A♭, B), creating a distinctive sound found in works by Debussy and Takemitsu.
DEFINE_CHORD(scale_CHarmonicMajor, "C3", "D3", "E3", "F3", "G3", "G#3", "B3");

// Eb Pentatonic Minor - A five-note scale derived from the natural minor scale by omitting the 
// second and sixth degrees (E♭, G♭, A♭, B♭, D♭), commonly used in blues, rock, and pop.
DEFINE_CHORD(scale_EbPentatonicMinor, "D#3", "F#3", "G#3", "A#3", "C#4");

// C Lydian - The fourth mode of G major consisting of seven notes (C, D, E, F♯, G, A, B), 
// characterized by a raised fourth degree compared to the major scale.
DEFINE_CHORD(scale_CLydian, "C3", "D3", "E3", "F#3", "G3", "A3", "B3");

// C Dorian - Minor mode with a raised 6th, classic jazz and folk sound
DEFINE_CHORD(scale_CDorian, "C3", "D3", "Eb3", "F3", "G3", "A3", "Bb3");

// C Whole Tone - Six-note symmetrical scale, all whole steps. Creates an
// ambiguous, dreamlike quality with no leading tone.
DEFINE_CHORD(scale_CWholeTone, "C3", "D3", "E3", "F#3", "G#3", "A#3");

// C Diminished (Half-Whole) - Eight-note octatonic scale. Alternates half-step,
// whole-step. Used extensively in classical and jazz for tension and resolution.
DEFINE_CHORD(scale_CDiminishedHalfWhole, "C3", "Db3", "Eb3", "E3", "F#3", "G3", "A3", "Bb3");

// C Blues - Essential six-note blues scale with a flatted fifth (blue note)
DEFINE_CHORD(scale_CBlues, "C3", "Eb3", "F3", "Gb3", "G3", "Bb3");

// C Melodic Minor (ascending) - Jazz minor scale with both minor and major
// character. Ascending form only, as per jazz convention.
DEFINE_CHORD(scale_CMelodicMinor, "C3", "D3", "Eb3", "F3", "G3", "A3", "B3");

// C Hirajoshi - Japanese pentatonic scale.
// Traditionally used in Japanese folk and classical music.
DEFINE_CHORD(scale_CHirajoshi, "C3", "Db3", "E3", "G3", "Ab3");

// C Hungarian Minor - Dramatic Eastern European scale.
// Features both a raised 4th and raised 7th, creating intense tension.
DEFINE_CHORD(scale_CHungarianMinor, "C3", "D3", "Eb3", "F#3", "G3", "Ab3", "B3");

// C Phrygian Dominant - Spanish and Arabic scale, the 5th mode of harmonic minor.
DEFINE_CHORD(scale_CPhrygianDominant, "C3", "Db3", "E3", "F3", "G3", "Ab3", "Bb3");

// F Major Pentatonic - Warm and pastoral
DEFINE_CHORD(scale_FMajPentatonic, "F3", "G3", "A3", "C4", "D4");

// Bb Major Pentatonic - mellow and rounded
DEFINE_CHORD(scale_BbMajPentatonic, "Bb3", "C4", "D4", "F4", "G4");

// A Minor Pentatonic - The most common minor pentatonic, very versatile
DEFINE_CHORD(scale_AMinPentatonic, "A3", "C4", "D4", "E4", "G4");

// D Hirajoshi - Japanese pentatonic with a haunting sound
DEFINE_CHORD(scale_DHirajoshi, "D3", "Eb3", "G3", "A3", "Bb3");

// G Egyptian (Suspended Pentatonic) - Neither major nor minor, creates
// an ambiguous, ancient quality. Used in traditional music across North Africa
// and the Middle East. Formula: 1, 2, 4, 5, b7
DEFINE_CHORD(scale_GEgyptian, "G3", "A3", "C4", "D4", "F4");

// E Kumoi - Japanese pentatonic often used in koto music. Has a melancholy
// but resolved quality. Formula: 1, 2, b3, 5, 6
DEFINE_CHORD(scale_EKumoi, "E3", "F#3", "G3", "B3", "C#4");

// D Iwato - Another Japanese pentatonic with an especially stark,
// unresolved character. Formula: 1, b2, 4, b5, b7
DEFINE_CHORD(scale_DIwato, "D3", "Eb3", "G3", "Ab3", "C4");








#endif