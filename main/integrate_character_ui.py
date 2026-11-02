#!/usr/bin/env python3
"""Inject the Analog Character A/B controls into the existing web_dsp.html."""
from pathlib import Path
import argparse
import re
import sys

MARKUP_MARKER = "  <!-- PARAMETRIC EQ -->"
SCRIPT_MARKER = "/* =========================================================\n   SPATIAL\n   ========================================================= */"
SAVE_OLD = '"Đã lưu preset + Bass Exciter vào flash (NVS)"'
SAVE_NEW = '"Đã lưu preset + Bass Exciter + Analog Character vào flash (NVS)"'

MARKUP = r'''  <!-- ANALOG CHARACTER -->

  <div class="section">

    <div class="peq-title">Analog Character</div>

    <div class="peq-subtitle">
      Tạo màu âm nhẹ trên vocal và mid. A thiên về độ dày/mượt; B giữ độ tự nhiên và thoáng.
      Độc lập với Bass Harmonic Exciter.
    </div>

    <div class="presets" id="characterProfiles">
      <div class="preset-btn" id="characterTube">
        <span class="dot"></span>
        <span class="preset-name">A · TUBE WARM</span>
      </div>
      <div class="preset-btn" id="characterBritish">
        <span class="dot"></span>
        <span class="preset-name">B · BRITISH NATURAL</span>
      </div>
    </div>

    <div class="feature">
      <div class="feature-text">
        <div class="feature-name">Analog Character</div>
        <div class="feature-info">Bật/tắt tầng tạo màu âm, không ảnh hưởng PEQ và Exciter</div>
      </div>
      <label class="switch">
        <input type="checkbox" id="characterEnabled">
        <span class="slider-toggle"></span>
      </label>
    </div>

    <div class="band">
      <div class="row-label"><span>Warmth — H2 / Vocal Body</span><b id="characterWarmthVal">0%</b></div>
      <input class="exciter-range" type="range" id="characterWarmth" min="0" max="100" step="1" value="0">
    </div>

    <div class="band">
      <div class="row-label"><span>Detail — H3 Texture</span><b id="characterDetailVal">0%</b></div>
      <input class="exciter-range" type="range" id="characterDetail" min="0" max="100" step="1" value="0">
    </div>

    <div class="band">
      <div class="row-label"><span>Drive — Harmonic Density</span><b id="characterDriveVal">0%</b></div>
      <input class="exciter-range" type="range" id="characterDrive" min="0" max="100" step="1" value="0">
    </div>

    <div class="band">
      <div class="row-label"><span>Softness — Dynamic Softening</span><b id="characterSoftnessVal">0%</b></div>
      <input class="exciter-range" type="range" id="characterSoftness" min="0" max="100" step="1" value="0">
    </div>

    <div class="band">
      <div class="row-label"><span>Air — High Frequency Openness</span><b id="characterAirVal">0%</b></div>
      <input class="exciter-range" type="range" id="characterAir" min="0" max="100" step="1" value="0">
    </div>

    <div class="exciter-note">
      <strong>A — Tube Warm:</strong> tăng body và độ dày vocal, H2 trội hơn.<br>
      <strong>B — British Natural:</strong> nhẹ hơn, giữ độ thoáng và articulation.<br>
      Chọn A/B để nạp thiết lập gợi ý; sau đó kéo slider để tinh chỉnh. Thay đổi áp dụng realtime,
      nhấn <strong>Save preset</strong> để lưu vào NVS.
    </div>

  </div>


'''

SCRIPT = r'''/* =========================================================
   ANALOG CHARACTER — A/B + realtime controls
   API state: state.character
   ========================================================= */

const characterControls = [
  ["characterWarmth", "warmth_pct", "characterWarmthVal"],
  ["characterDetail", "detail_pct", "characterDetailVal"],
  ["characterDrive", "drive_pct", "characterDriveVal"],
  ["characterSoftness", "softness_pct", "characterSoftnessVal"],
  ["characterAir", "air_pct", "characterAirVal"]
];

let characterTimer = null;

function renderCharacter(){
  const c = state && state.character;
  if(!c) return;

  document.getElementById("characterEnabled").checked = !!c.enabled;

  characterControls.forEach(([inputId, key, valueId]) => {
    const value = Math.max(0, Math.min(100, Number(c[key] ?? 0)));
    document.getElementById(inputId).value = value;
    document.getElementById(valueId).textContent = `${Math.round(value)}%`;
  });

  const mode = Number(c.mode ?? 0);
  document.getElementById("characterTube").classList.toggle("active", mode === 1);
  document.getElementById("characterBritish").classList.toggle("active", mode === 2);
}

async function selectCharacterProfile(mode){
  try{
    state = await api("/api/dsp/character", {mode});
    renderCharacter();
    setStatus(mode === 1 ? "Analog Character: A — Tube Warm" : "Analog Character: B — British Natural");
  }catch(err){
    console.error(err);
    setStatus("Không thể đổi Analog Character profile");
  }
}

document.getElementById("characterTube").addEventListener("click", () => selectCharacterProfile(1));
document.getElementById("characterBritish").addEventListener("click", () => selectCharacterProfile(2));

document.getElementById("characterEnabled").addEventListener("change", async e => {
  try{
    state = await api("/api/dsp/character", {enabled:e.target.checked});
    renderCharacter();
    setStatus(e.target.checked ? "Analog Character ON" : "Analog Character OFF");
  }catch(err){
    console.error(err);
    setStatus("Không thể bật/tắt Analog Character");
    await refresh();
  }
});

function characterPayload(){
  const payload = {mode:0}; // any manual edit switches the profile label to CUSTOM
  characterControls.forEach(([inputId, key]) => {
    payload[key] = Number(document.getElementById(inputId).value);
  });
  return payload;
}

function scheduleCharacterUpdate(){
  clearTimeout(characterTimer);
  characterTimer = setTimeout(updateCharacter, 180);
}

async function updateCharacter(){
  try{
    state = await api("/api/dsp/character", characterPayload());
    renderCharacter();
  }catch(err){
    console.error(err);
    setStatus("Không thể cập nhật Analog Character");
  }
}

characterControls.forEach(([inputId, key, valueId]) => {
  const slider = document.getElementById(inputId);
  slider.addEventListener("input", e => {
    document.getElementById(valueId).textContent = `${Math.round(Number(e.target.value))}%`;
    scheduleCharacterUpdate();
  });
  slider.addEventListener("change", () => {
    clearTimeout(characterTimer);
    updateCharacter();
  });
});


'''


def find_matching_brace(source: str, open_pos: int) -> int:
    """Return the matching } for source[open_pos] == {, ignoring JS strings/comments."""
    depth = 0
    state = "code"
    escaped = False
    i = open_pos
    while i < len(source):
        ch = source[i]
        nxt = source[i + 1] if i + 1 < len(source) else ""

        if state == "line_comment":
            if ch in "\r\n":
                state = "code"
        elif state == "block_comment":
            if ch == "*" and nxt == "/":
                state = "code"
                i += 1
        elif state in ("single", "double", "template"):
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif state == "single" and ch == "'":
                state = "code"
            elif state == "double" and ch == '"':
                state = "code"
            elif state == "template" and ch == "`":
                state = "code"
        else:
            if ch == "/" and nxt == "/":
                state = "line_comment"
                i += 1
            elif ch == "/" and nxt == "*":
                state = "block_comment"
                i += 1
            elif ch == "'":
                state = "single"
            elif ch == '"':
                state = "double"
            elif ch == "`":
                state = "template"
            elif ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    return i
        i += 1
    return -1


def patch_html(source: str) -> str:
    if source.count(MARKUP_MARKER) != 1:
        raise ValueError("Không tìm thấy duy nhất marker <!-- PARAMETRIC EQ -->")
    if source.count(SCRIPT_MARKER) != 1:
        raise ValueError("Không tìm thấy duy nhất marker SPATIAL trong <script>")
    if 'id="characterEnabled"' in source or "/api/dsp/character" in source:
        raise ValueError("File HTML có vẻ đã được tích hợp Character; dừng để tránh chèn trùng.")

    out = source.replace(MARKUP_MARKER, MARKUP + MARKUP_MARKER, 1)
    out = out.replace(SCRIPT_MARKER, SCRIPT + SCRIPT_MARKER, 1)

    # Find renderAll() without depending on indentation or exact whitespace.
    # The body is intentionally located by balanced braces so formatting changes
    # (spaces/tabs/CRLF/minification) do not break the injector.
    signature = re.search(r"\bfunction\s+renderAll\s*\(\s*\)\s*\{", out)
    if not signature:
        raise ValueError("Không tìm thấy function renderAll() trong HTML.")

    open_brace = out.find("{", signature.start(), signature.end())
    close_brace = find_matching_brace(out, open_brace)
    if close_brace < 0:
        raise ValueError("Không xác định được phần kết thúc của renderAll().")

    body = out[open_brace + 1:close_brace]
    if re.search(r"\brenderCharacter\s*\(\s*\)\s*;", body):
        raise ValueError("renderCharacter() đã có trong renderAll(); dừng để tránh chèn trùng.")

    exciter_call = re.search(r"\brenderExciter\s*\(\s*\)\s*;", body)
    if exciter_call:
        insert_at = open_brace + 1 + exciter_call.end()
        out = out[:insert_at] + "\n  renderCharacter();" + out[insert_at:]
    else:
        # If the page uses a differently named render order, add it at the end
        # of renderAll() rather than refusing to patch the complete UI.
        out = out[:close_brace] + "\n  renderCharacter();\n" + out[close_brace:]

    if SAVE_OLD in out:
        out = out.replace(SAVE_OLD, SAVE_NEW, 1)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Đường dẫn web_dsp.html hiện tại")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="File đầu ra (mặc định: web_dsp_character.html cùng thư mục)")
    args = parser.parse_args()

    if not args.input.is_file():
        print(f"Không tìm thấy file: {args.input}", file=sys.stderr)
        return 2

    output = args.output or args.input.with_name("web_dsp_character.html")
    try:
        original = args.input.read_text(encoding="utf-8")
        updated = patch_html(original)
        output.write_text(updated, encoding="utf-8", newline="\n")
    except (OSError, UnicodeError, ValueError) as exc:
        print(f"Lỗi: {exc}", file=sys.stderr)
        return 1

    print(f"Đã tạo: {output}")
    print("Đã thêm profile A/B, công tắc ON/OFF, 5 slider và gọi đúng /api/dsp/character.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
