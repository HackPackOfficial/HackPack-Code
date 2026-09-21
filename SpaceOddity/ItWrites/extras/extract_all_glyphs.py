import re
import argparse
import sys
import numpy as np

# Order the 36 strokes are expected to appear in the SVG (drawing order):
# A-Z, then 1-9, then 0. Edit this if you draw a different character set
# or in a different order.
DEFAULT_GLYPH_NAMES = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ") + list("123456789") + ["0"]


def get_path_ds(svg_path):
    with open(svg_path) as f:
        content = f.read()
    return re.findall(r'\sd="([^"]+)"', content)


def parse_path(d):
    """
    Parse a path string containing M (moveto), L (lineto) and Q (quadratic
    Bezier) commands with absolute coordinates. Returns the start point and
    a list of segments, each either ('L', p0, p1) or ('Q', p0, ctrl, p1).
    """
    tokens = re.findall(r'[MQL]|-?\d+\.?\d*', d)
    i = 0
    cmd = None
    cur = None
    start = None
    segs = []

    def read_point(idx):
        return (float(tokens[idx]), float(tokens[idx + 1])), idx + 2

    while i < len(tokens):
        tok = tokens[i]
        if tok in ('M', 'Q', 'L'):
            cmd = tok
            i += 1
            continue
        if cmd == 'M':
            p, i = read_point(i)
            cur = p
            start = p
        elif cmd == 'L':
            p, i = read_point(i)
            segs.append(('L', cur, p))
            cur = p
        elif cmd == 'Q':
            c, i = read_point(i)
            p, i = read_point(i)
            segs.append(('Q', cur, c, p))
            cur = p
        else:
            raise ValueError(f"Unhandled command before token: {tok}")
    return start, segs


def dense_sample(start, segs, samples_per_seg=40):
    pts = [start]
    for seg in segs:
        if seg[0] == 'L':
            _, p0, p1 = seg
            for k in range(1, samples_per_seg + 1):
                t = k / samples_per_seg
                pts.append((p0[0] + (p1[0] - p0[0]) * t,
                             p0[1] + (p1[1] - p0[1]) * t))
        else:  # 'Q'
            _, p0, c, p1 = seg
            for k in range(1, samples_per_seg + 1):
                t = k / samples_per_seg
                mt = 1 - t
                x = mt * mt * p0[0] + 2 * mt * t * c[0] + t * t * p1[0]
                y = mt * mt * p0[1] + 2 * mt * t * c[1] + t * t * p1[1]
                pts.append((x, y))
    return np.array(pts)


def resample_even_arclength(pts, n_points):
    deltas = np.diff(pts, axis=0)
    seg_lens = np.hypot(deltas[:, 0], deltas[:, 1])
    cum = np.concatenate([[0], np.cumsum(seg_lens)])
    total_len = cum[-1]
    targets = np.linspace(0, total_len, n_points)
    out = np.empty((n_points, 2))
    for i, t in enumerate(targets):
        idx = np.searchsorted(cum, t, side='right') - 1
        idx = min(max(idx, 0), len(cum) - 2)
        denom = cum[idx + 1] - cum[idx]
        frac = (t - cum[idx]) / denom if denom > 0 else 0
        out[i] = pts[idx] + frac * (pts[idx + 1] - pts[idx])
    return out


def center_and_normalize(pts):
    """
    Shift so the centroid (mean of the resampled points) is at the origin,
    flip Y to a Y-up frame, then scale isotropically so the larger of the
    two extents maps to [-1, 1]. Isotropic scaling keeps letter proportions
    correct -- a normalized 'I' stays skinny instead of getting stretched
    into a square.
    """
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    centered[:, 1] *= -1  # SVG is Y-down; flip to Y-up
    max_extent = np.abs(centered).max()
    normalized = centered / max_extent
    return normalized


def to_c_array(name, pts, n_points):
    lines = [f"static const float {name}[{n_points}][2] = {{"]
    row_strs = []
    for x, y in pts:
        row_strs.append(f"{{{x:.6f}f, {y:.6f}f}}")
    for i in range(0, len(row_strs), 4):
        chunk = ", ".join(row_strs[i:i + 4])
        lines.append(f"    {chunk},")
    lines.append("};")
    return "\n".join(lines)


def parse_args():
    p = argparse.ArgumentParser(
        description=(
            "Extract single-stroke glyphs from a Concepts-exported SVG "
            "(M/L/Q path commands only) into a C header of resampled, "
            "centered, normalized point arrays for a plotter."
        )
    )
    p.add_argument("svg_path", help="Path to the input SVG file")
    p.add_argument(
        "-o", "--output", default=None,
        help="Path to write the generated C header (default: <svg_stem>_points.h)"
    )
    p.add_argument(
        "-n", "--n-points", type=int, default=100,
        help="Number of evenly-spaced points to resample each glyph to (default: 100)"
    )
    p.add_argument(
        "--names", default=None,
        help=(
            "Characters, in the order the strokes appear in the SVG, as a "
            "single string (e.g. 'ABCDEFGHIJKLMNOPQRSTUVWXYZ123456789 0'). "
            "Default: A-Z, 1-9, 0 (36 characters)."
        )
    )
    p.add_argument(
        "--samples-per-seg", type=int, default=40,
        help="Dense sampling resolution per path segment before arc-length "
             "resampling (default: 40; raise if curves look faceted)"
    )
    return p.parse_args()


def main():
    args = parse_args()

    svg_path = args.svg_path
    n_points = args.n_points
    glyph_names = list(args.names) if args.names else DEFAULT_GLYPH_NAMES

    if args.output:
        out_path = args.output
    else:
        stem = re.sub(r"\.svg$", "", svg_path, flags=re.IGNORECASE)
        stem = stem.split("/")[-1]
        out_path = f"{stem}_points.h"

    path_ds = get_path_ds(svg_path)
    if len(path_ds) != len(glyph_names):
        sys.exit(
            f"Error: found {len(path_ds)} stroke(s) in {svg_path} but "
            f"{len(glyph_names)} character name(s) were expected. "
            f"Pass --names to match the actual stroke count/order."
        )

    header_lines = [
        f"// Auto-generated from {svg_path}",
        "// Each glyph is a single continuous pen stroke resampled to",
        f"// {n_points} evenly-spaced points (by arc length), centered on",
        "// its own centroid, Y-up, and isotropically normalized so the",
        "// larger of its X/Y extents spans [-1.0f, 1.0f].",
        "",
        "#ifndef GLYPH_POINTS_H",
        "#define GLYPH_POINTS_H",
        "",
    ]

    array_names = []
    for name, d in zip(glyph_names, path_ds):
        start, segs = parse_path(d)
        dense = dense_sample(start, segs, samples_per_seg=args.samples_per_seg)
        resampled = resample_even_arclength(dense, n_points)
        normalized = center_and_normalize(resampled)

        safe_name = name if name.isalpha() else f"NUM_{name}"
        c_name = f"GLYPH_{safe_name}"
        array_names.append((name, c_name))

        header_lines.append(to_c_array(c_name, normalized, n_points))
        header_lines.append("")

    header_lines.append(f"#define GLYPH_COUNT {len(glyph_names)}")
    header_lines.append(f"#define GLYPH_POINT_COUNT {n_points}")
    header_lines.append("")
    header_lines.append("typedef struct {")
    header_lines.append("    char character;")
    header_lines.append("    const float (*points)[2];")
    header_lines.append("} glyph_entry_t;")
    header_lines.append("")
    header_lines.append("static const glyph_entry_t GLYPH_TABLE[GLYPH_COUNT] = {")
    for name, c_name in array_names:
        header_lines.append(f"    {{'{name}', {c_name}}},")
    header_lines.append("};")
    header_lines.append("")
    header_lines.append("#endif // GLYPH_POINTS_H")

    with open(out_path, "w") as f:
        f.write("\n".join(header_lines))

    print(f"Wrote {len(glyph_names)} glyphs, {n_points} points each, to {out_path}")


if __name__ == "__main__":
    main()
