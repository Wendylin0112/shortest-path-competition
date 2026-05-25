#!/usr/bin/env python3
from pathlib import Path


def esc(text):
    return text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def text_line(x, y, text, font="F1", size=8.7):
    return f"BT /{font} {size:.1f} Tf {x:.1f} {y:.1f} Td ({esc(text)}) Tj ET\n"


def make_pdf(path):
    width, height = 612, 792
    y = 760
    lines = []

    lines.append(text_line(54, y, "Shortest-Path Competition Solver Report", "F2", 15))
    y -= 18
    lines.append(text_line(54, y, "Final solver: CSR + exact ALT landmarks + bucket queues", "F1", 9.5))
    y -= 22

    sections = [
        ("Interface and Correctness", [
            "Required CLI kept: ./solver <graph_file> <query_file> <output_file>.",
            "The implementation is derived from foundation Dijkstra and returns exact shortest-path distances.",
            "The final full local grade.py run reported ok on all 12 released instances.",
        ]),
        ("Main Optimizations", [
            "CSR adjacency arrays, whole-file input, buffered output, and generation counters reduce overhead.",
            "Exact ALT/A* uses 24 farthest-first landmarks, including SOCIAL graphs without coordinates.",
            "GRID/SOCIAL low-diameter graphs use Dial buckets and compact int16 landmark tables when safe.",
            "ROAD uses unidirectional ALT plus bitmap buckets; CLUSTER uses bidirectional ALT with an 8-ary heap.",
            "SOCIAL initializes a landmark-derived upper bound min_k d(s,L_k)+d(L_k,t) for exact-safe pruning.",
        ]),
    ]

    for title, body in sections:
        lines.append(text_line(54, y, title, "F2", 10.2))
        y -= 12
        for item in body:
            lines.append(text_line(66, y, "- " + item, "F1", 8.3))
            y -= 10
        y -= 5

    lines.append(text_line(54, y, "Full Local Grade.py Measurements", "F2", 10.2))
    y -= 12
    lines.append(text_line(66, y, "Command: python3 grade.py --foundation ./foundation --solver ./solver --instances instances.txt --progress", "F1", 7.4))
    y -= 12

    table = [
        "Cat      Instance              T_base     T_solver   Speedup  Note",
        "ROAD     road_small.graph         4.3011     0.1951    22.042  ok",
        "GRID     grid_small.graph         4.4041     0.0952    46.256  ok",
        "CLUSTER  cluster_small.graph      4.9962     0.3745    13.341  ok",
        "SOCIAL   social_small.graph      13.2219     0.1994    66.296  ok",
        "ROAD     road_medium.graph       62.7959     1.4695    42.733  ok",
        "GRID     grid_medium.graph       47.1362     0.7804    60.401  ok",
        "CLUSTER  cluster_medium.graph    76.3894     1.6086    47.487  ok",
        "SOCIAL   social_medium.graph    244.1336     3.0551    79.910  ok",
        "ROAD     road_large.graph       904.8340    13.7633    65.743  ok",
        "GRID     grid_large.graph       587.3162     9.4342    62.254  ok",
        "CLUSTER  cluster_large.graph    1079.0337    10.6161   100.000  ok",
        "SOCIAL   social_large.graph     3620.3446    21.6503   100.000  ok",
    ]
    for row in table:
        lines.append(text_line(66, y, row, "F3", 7.2))
        y -= 8.5
    y -= 7

    lines.append(text_line(54, y, "Geometric Means", "F2", 10.2))
    y -= 12
    geomeans = [
        "CLUSTER  39.865 (n=3)    GRID 55.821 (n=3)    ROAD 39.563 (n=3)    SOCIAL 80.915 (n=3)",
        "Algorithm track 44.486 (n=9)    Engineering track 80.915 (n=3)    Overall 51.662 (n=12)",
    ]
    for line in geomeans:
        lines.append(text_line(66, y, line, "F3", 7.6))
        y -= 10

    y -= 4
    lines.append(text_line(54, y, "Comparison and Reproducibility", "F2", 10.2))
    y -= 12
    footer = [
        "Compared with the fastest reference-style local large run before SOCIAL ALT:",
        "road 14.0391->13.7633 s, grid 9.6468->9.4342 s, cluster 11.3167->10.6161 s, social 68.5482->21.6503 s.",
        "Large-tier geometric improvement over that local reference is about 1.36x.",
        "Prefetch/SIMD intrinsics were left out so the final code remains ISO C++17/std-library-only.",
    ]
    for line in footer:
        lines.append(text_line(66, y, line, "F1", 7.8))
        y -= 10

    stream = "".join(lines).encode("latin-1")
    objects = []
    objects.append(b"<< /Type /Catalog /Pages 2 0 R >>")
    objects.append(b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>")
    objects.append(
        f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {width} {height}] "
        f"/Resources << /Font << /F1 4 0 R /F2 5 0 R /F3 6 0 R >> >> "
        f"/Contents 7 0 R >>".encode("latin-1")
    )
    objects.append(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>")
    objects.append(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>")
    objects.append(b"<< /Type /Font /Subtype /Type1 /BaseFont /Courier >>")
    objects.append(b"<< /Length " + str(len(stream)).encode("ascii") + b" >>\nstream\n" + stream + b"endstream")

    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = [0]
    for i, obj in enumerate(objects, start=1):
        offsets.append(len(out))
        out.extend(f"{i} 0 obj\n".encode("ascii"))
        out.extend(obj)
        out.extend(b"\nendobj\n")
    xref = len(out)
    out.extend(f"xref\n0 {len(objects)+1}\n".encode("ascii"))
    out.extend(b"0000000000 65535 f \n")
    for off in offsets[1:]:
        out.extend(f"{off:010d} 00000 n \n".encode("ascii"))
    out.extend(
        f"trailer\n<< /Size {len(objects)+1} /Root 1 0 R >>\n"
        f"startxref\n{xref}\n%%EOF\n".encode("ascii")
    )
    Path(path).write_bytes(out)


if __name__ == "__main__":
    make_pdf("report.pdf")
