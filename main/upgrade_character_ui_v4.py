#!/usr/bin/env python3
"""Upgrade an already-integrated Analog Character A/B page to V4 controls."""
from pathlib import Path
import argparse
import re
import sys

MARKUP_ANCHOR = '    <div class="exciter-note">\n      <strong>A — Tube Warm:</strong>'
CONTROL_BLOCK = r'''    <div class="band">
      <div class="row-label"><span>Memory — Bias / Charge / Supply</span><b id="characterMemoryVal">50%</b></div>
      <input class="exciter-range" type="range" id="characterMemory" min="0" max="100" step="1" value="50">
    </div>

    <div class="band">
      <div class="row-label"><span>Spectral — Stage Balance</span><b id="characterSpectralVal">50%</b></div>
      <input class="exciter-range" type="range" id="characterSpectral" min="0" max="100" step="1" value="50">
    </div>

'''

def patch(source: str) -> str:
    if 'id="characterMemory"' in source or 'id="characterSpectral"' in source:
        raise ValueError("Trang đã có slider Memory/Spectral; không chèn trùng.")
    if MARKUP_ANCHOR not in source:
        raise ValueError("Không tìm thấy phần ghi chú A/B của Analog Character cũ.")

    out = source.replace(MARKUP_ANCHOR, CONTROL_BLOCK + MARKUP_ANCHOR, 1)
    out = out.replace(
        "Tạo màu âm nhẹ trên vocal và mid. A thiên về độ dày/mượt; B giữ độ tự nhiên và thoáng.\n      Độc lập với Bass Harmonic Exciter.",
        "Preamp DSP 3 tầng với saturation phụ thuộc tần số và memory feedback có kiểm soát. A thiên về body/mượt; B giữ độ tự nhiên và thoáng. Độc lập với Bass Harmonic Exciter.",
        1
    )

    m = re.search(r'const\s+characterControls\s*=\s*\[', out)
    if not m:
        raise ValueError("Không tìm thấy characterControls trong JavaScript.")
    close = out.find('];', m.end())
    if close < 0:
        raise ValueError("Không xác định được cuối mảng characterControls.")
    arr = out[m.end():close]
    if 'memory_pct' in arr or 'spectral_pct' in arr:
        raise ValueError("Mảng controls đã có Memory/Spectral; dừng để tránh trùng.")
    add = '\n  ["characterMemory", "memory_pct", "characterMemoryVal"],\n  ["characterSpectral", "spectral_pct", "characterSpectralVal"]'
    out = out[:close] + add + out[close:]

    out = out.replace(
        '"Đã lưu preset + Bass Exciter + Analog Character vào flash (NVS)"',
        '"Đã lưu preset + Exciter + Analog Character V4 vào flash (NVS)"',
        1
    )
    out = out.replace(
        '"Đã lưu preset + Bass Exciter vào flash (NVS)"',
        '"Đã lưu preset + Exciter + Analog Character V4 vào flash (NVS)"',
        1
    )
    return out

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", type=Path, help="web_dsp.html đã có Analog Character A/B")
    ap.add_argument("-o", "--output", type=Path, default=None,
                    help="File đầu ra (mặc định web_dsp_v4.html)")
    args = ap.parse_args()
    if not args.input.is_file():
        print(f"Không tìm thấy file: {args.input}", file=sys.stderr)
        return 2
    outpath = args.output or args.input.with_name("web_dsp_v4.html")
    try:
        updated = patch(args.input.read_text(encoding="utf-8"))
        outpath.write_text(updated, encoding="utf-8", newline="\n")
    except (OSError, UnicodeError, ValueError) as exc:
        print(f"Lỗi: {exc}", file=sys.stderr)
        return 1
    print(f"Đã tạo: {outpath}")
    print("Đã thêm Memory/Spectral và ánh xạ realtime qua /api/dsp/character.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
