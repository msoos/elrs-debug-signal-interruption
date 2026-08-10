-- Predictable channel sweep for logic-analyzer capture. Copy to /SCRIPTS/TOOLS/ on the SD card.

local toolName = "TNS|CRSF Sweep|TNE"

local GV, FM = 0, 0
local SLAM, RAMP, DWELL = 20, 300, 20

local PHASES = {
  {SLAM,  -100, -100, "mark lo"},
  {SLAM,   100,  100, "mark hi"},
  {SLAM,  -100, -100, "mark lo"},
  {RAMP,  -100,  100, "ramp up"},
  {DWELL,  100,  100, "hold hi"},
  {RAMP,   100, -100, "ramp dn"},
  {DWELL, -100, -100, "hold lo"},
}

local CYCLE = 0
for i = 1, #PHASES do CYCLE = CYCLE + PHASES[i][1] end

local t0 = 0

local function valueAt(t)
  for i = 1, #PHASES do
    local p = PHASES[i]
    if t < p[1] then
      return p[2] + (p[3] - p[2]) * t / p[1], p[4]
    end
    t = t - p[1]
  end
  local last = PHASES[#PHASES]
  return last[3], last[4]
end

local function init()
  t0 = getTime()
end

local function run(event)
  if event == EVT_VIRTUAL_EXIT or event == EVT_EXIT_BREAK then
    model.setGlobalVariable(GV, FM, -100)
    return 1
  end

  local el = getTime() - t0
  local v, phase = valueAt(el % CYCLE)
  v = math.floor(v + 0.5)
  model.setGlobalVariable(GV, FM, v)

  lcd.clear()
  lcd.drawText(1, 1, "CRSF Sweep", INVERS)
  lcd.drawText(1, 12, phase .. "   GV" .. (GV + 1) .. " = " .. v .. "%")
  lcd.drawText(1, 23, "cycle " .. math.floor(el / CYCLE) ..
                      "   " .. math.floor(el / 100) .. "s")
  lcd.drawText(1, 34, "EXIT stops, leaves -100%")
  return 0
end

return {init = init, run = run, name = toolName}
