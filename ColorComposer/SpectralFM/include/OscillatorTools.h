#ifndef OSCILLATOR_TOOLS_H
#define OSCILLATOR_TOOLS_H

#include <Arduino.h>
#include <MusicTypes.h>
#include <FixMath.h>

// struct for storing parameters for each oscillator
struct oscillatorParams
{
  const char *note = 0;
  const char *lastNote = 0;
  MIDI_NOTE noteMIDINumber = 0;
  MIDI_NOTE lastNoteMIDINumber = 0;
  float frequency = 0.0;
  uint8_t volume = 0;
};

// currently this can only set the frequencies from up to 4 notes, need to revise this to be more flexible. basically,
// I want this to look at how many oscillators are active in the sketch, how many notes are in the chord, find the min()
// of those two, and then iterate through the chord, converting notes to frequencies until the min() is reached.
void setFreqsFromChord(const Chord &chord, UFix<12, 15> &f1, UFix<12, 15> &f2, UFix<12, 15> &f3, UFix<12, 15> &f4)
{
    UFix<12, 15>* freqs[] = {&f1, &f2, &f3, &f4};
    for (uint8_t i = 0; i < 4; i++)
    {
        MIDI_NOTE note = chord.getNote(i);
        *freqs[i] = (note < 128) ? mtof(UFix<7, 0>(note)) : 0;
    }
}


#endif