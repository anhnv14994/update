/* =========================================================
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


