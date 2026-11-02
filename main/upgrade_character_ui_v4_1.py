#!/usr/bin/env python3
"""Upgrade an existing Analog Character V3/V4 page to V4.1 safely."""
from pathlib import Path
import argparse
import re
import sys

MARKUP_ANCHOR = '    <div class="exciter-note">\n      <strong>A — Tube Warm:</strong>'
CONTROL_BLOCK = '''    <div class="band">
      <div class="row-label"><span>Memory — Bias / Charge / Supply</span><b id="characterMemoryVal">50%</b></div>
      <input class="exciter-range" type="range" id="characterMemory" min="0" max="100" step="1" value="50">
    </div>

    <div class="band">
      <div class="row-label"><span>Spectral — Stage Balance</span><b id="characterSpectralVal">50%</b></div>
      <input class="exciter-range" type="range" id="characterSpectral" min="0" max="100" step="1" value="50">
    </div>

'''


def patch(source: str) -> str:
    if 'id="characterMemory"' not in source:
        if MARKUP_ANCHOR not in source:
            raise ValueError("Không tìm thấy phần ghi chú A/B của Analog Character.")
        source = source.replace(MARKUP_ANCHOR, CONTROL_BLOCK + MARKUP_ANCHOR, 1)

    # Fix malformed V4.0 output where the original Air item lacked a comma.
    source = re.sub(
        r'(\["characterAir"\s*,\s*"air_pct"\s*,\s*"characterAirVal"\])\s*(\["characterMemory")',
        r'\1,\n  \2', source, count=1)

    # Add controls if this is still a V3 page.
    m = re.search(r'const\s+characterControls\s*=\s*\[', source)
    if not m:
        raise ValueError("Không tìm thấy characterControls.")
    close = source.find('];', m.end())
    if close < 0:
        raise ValueError("Không xác định được cuối mảng characterControls.")
    arr = source[m.end():close]
    add_items = []
    if 'memory_pct' not in arr:
        add_items.append('["characterMemory", "memory_pct", "characterMemoryVal"]')
    if 'spectral_pct' not in arr:
        add_items.append('["characterSpectral", "spectral_pct", "characterSpectralVal"]')
    if add_items:
        prefix = ',' if arr.strip() and not arr.rstrip().endswith(',') else ''
        addition = prefix + '\n  ' + ',\n  '.join(add_items) + '\n'
        source = source[:close] + addition + source[close:]

    # Toggle Exciter via an enabled-only request. It cannot mutate PEQ/master/DSP state.
    pattern = re.compile(
        r'document\s*\.getElementById\(\s*["\']exciterEnabled["\']\s*\)\s*'
        r'\.addEventListener\(\s*["\']change["\']\s*,\s*updateExciter\s*\)\s*;',
        re.S)
    replacement = '''document.getElementById("exciterEnabled").addEventListener("change", async e => {
  try {
    state = await api("/api/dsp/exciter", {enabled:e.target.checked});
    renderExciter();
    setStatus(e.target.checked ? "Bass Exciter ON" : "Bass Exciter OFF — PEQ vẫn hoạt động");
  } catch(err) {
    console.error(err);
    setStatus("Không thể đổi Bass Exciter");
    await refresh();
  }
});'''
    source, count = pattern.subn(replacement, source, count=1)
    if count == 0 and 'Bass Exciter OFF — PEQ vẫn hoạt động' not in source and 'id="exciterEnabled"' in source:
        # Some custom pages use a separate handler name. Do not guess-rewrite it.
        raise ValueError("Trang có công tắc Exciter nhưng không tìm thấy listener chuẩn; cần kiểm tra thủ công.")

    return source


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('input', type=Path)
    ap.add_argument('-o', '--output', type=Path, default=None)
    args = ap.parse_args()
    if not args.input.is_file():
        print(f"Không tìm thấy file: {args.input}", file=sys.stderr)
        return 2
    output = args.output or args.input.with_name('web_dsp_v4_1.html')
    try:
        updated = patch(args.input.read_text(encoding='utf-8'))
        output.write_text(updated, encoding='utf-8', newline='\n')
    except (OSError, UnicodeError, ValueError) as exc:
        print(f"Lỗi: {exc}", file=sys.stderr)
        return 1
    print(f"Đã tạo: {output}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
