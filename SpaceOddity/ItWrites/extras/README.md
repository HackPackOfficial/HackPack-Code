# Glyph Point Extractor

Converts a hand-drawn SVG of letters and numbers into a C header of
point-array data, suitable for driving a plotter. Each character
becomes a fixed-length array of `{x, y}` points, resampled at even
spacing along the stroke, centered on its own centroid, and normalized
to the range `[-1.0, 1.0]`.

## What kind of SVG you need

This script assumes a specific drawing convention. It was built against
SVGs exported from Concepts (the sketching app), but any tool that
produces similarly structured output will work.

- **One stroke per character, drawn without lifting the pen.** Each
  character must be a single unbroken path — no dotted `i`, no
  separate crossbar on a `t` drawn as its own stroke, no multi-part
  glyphs. If your plotter can only trace one continuous path per
  character, draw it that way from the start (see the `A` example,
  where the crossbar gets looped into the same stroke as the two
  legs).
- **One `<path>` element per character in the SVG.** The script counts
  `<path>` elements and matches them, in order, against the expected
  character list. Anything that isn't a distinct stroke — construction
  lines, a background rectangle, grid guides — will throw off the
  count and needs to be removed from the file before running the
  script.
- **Path data must use only `M`, `L`, and `Q` commands** (moveto,
  lineto, and quadratic Bezier), with absolute coordinates. This is
  what Concepts exports for freehand strokes. Cubic Beziers (`C`),
  arcs (`A`), or relative-coordinate commands (lowercase letters) are
  not currently handled — if your export uses those, the script will
  either error or misparse the geometry.
- **Draw the characters in a consistent, known order.** By default the
  script expects 36 strokes in the order `A-Z`, then `1-9`, then `0`.
  If you draw a different set of characters, or in a different order,
  use the `--names` flag to tell the script what to expect (see
  below).

If you're not sure whether your SVG qualifies, open it in a text
editor and look for a run of `<path ... d="M ... Q ... Q ... ">`
elements — one per character, each starting with a single `M` and
using only `Q`/`L` after that.

## Requirements

Python 3, with `numpy` installed:

```
pip install numpy
```

## Usage

```
python3 extract_all_glyphs.py path/to/drawing.svg
```

This writes a header file (by default `drawing_points.h`, based on
the input filename) in the current directory.

### Options

| Flag | Description |
|---|---|
| `-o`, `--output` | Output path for the generated header. Default: `<svg filename>_points.h` |
| `-n`, `--n-points` | Number of evenly-spaced points per glyph. Default: `100` |
| `--names` | The characters, in stroke order, as one string (e.g. `"ABCDEFGHIJKLMNOPQRSTUVWXYZ123456789 0"`). Default: `A-Z`, `1-9`, `0` |
| `--samples-per-seg` | Dense pre-sampling resolution per path segment, before arc-length resampling. Default: `40`. Raise this if curved strokes come out looking faceted rather than smooth. |

### Examples

Default 36-character alphabet + digit set, 100 points each:
```
python3 extract_all_glyphs.py my_drawing.svg
```

Only the digits, 50 points each, custom output name:
```
python3 extract_all_glyphs.py digits_only.svg --names "1234567890" -n 50 -o digits.h
```

If the script finds a different number of strokes than the `--names`
string expects, it exits with an error instead of writing a
mismatched file — check the SVG for extra or missing paths, or fix
the `--names` argument.

## Output format

The generated header defines one array per character:

```c
static const float GLYPH_A[100][2] = {
    {-0.711561f, -0.974274f}, {-0.688619f, -0.920131f}, ...
};
```

Digit arrays are named `GLYPH_NUM_<digit>` (e.g. `GLYPH_NUM_3`) since
`GLYPH_3` isn't a valid-looking identifier convention to mix with the
letter names, though the character itself is preserved correctly
elsewhere in the file.

A lookup table ties characters to their arrays:

```c
typedef struct {
    char character;
    const float (*points)[2];
} glyph_entry_t;

static const glyph_entry_t GLYPH_TABLE[GLYPH_COUNT] = {
    {'A', GLYPH_A},
    {'B', GLYPH_B},
    ...
};
```

`GLYPH_COUNT` and `GLYPH_POINT_COUNT` are also defined, so firmware
can iterate the table or index a specific character's points without
hardcoding either number.

## Notes on the coordinate normalization

- **Centering** uses the centroid (mean) of the resampled points, not
  the geometric center of the bounding box. For a closed shape like
  `O` these are close together; for something like `L` or `T`, where
  path length isn't evenly distributed around the shape, the centroid
  can sit off from the visual middle.
- **Scaling is isotropic and per-glyph.** Each character is scaled
  independently so its longer axis (X or Y) spans exactly `[-1, 1]`.
  This preserves each letter's proportions — an `I` stays skinny
  rather than getting stretched into a square — but it also means
  absolute size varies from glyph to glyph. If your plotter needs
  every character to come out the same physical width regardless of
  shape, this script would need a shared scale factor across all
  glyphs instead; ask if that's what you need.
