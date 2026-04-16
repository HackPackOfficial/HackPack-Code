#ifndef MUSIC_TYPES_H
#define MUSIC_TYPES_H

#include <Arduino.h>

// This is just an alias made for clarity of intent. MIDI_NOTE is exactly the same as uint8_t, and you can use them interchangeably.
// I just made this so that its clear when the code is talking about a MIDI note.
using MIDI_NOTE = uint8_t;

// This struct lets you store a group of notes in flash (PROGMEM) as a chord object. Because it just stores a pointer to the first
// element in the array, you can store an arbitrary number of notes in the chord, up to 256. And it should only take up 3 bytes of
// RAM per chord regardless of the number of notes in the chord, which is a huge gain in efficiency.
struct Chord {
    const MIDI_NOTE* notes;   // pointer to PROGMEM array
    uint8_t numNotes;

    MIDI_NOTE getNote(uint8_t index) const {
        if (index >= numNotes) return 255;
        return pgm_read_byte(&notes[index]);
    }
};

#endif