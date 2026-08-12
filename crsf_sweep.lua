-- Predictable channel sweep for logic-analyzer capture. Copy to /SCRIPTS/TOOLS/ on the SD card.

local toolName = "TNS|CRSF Sweep|TNE"

local GV, FM = 0, 0
local SLAM = 20
local RAMP = 300
local STEP, RAMP_MIN, RAMP_MAX = 25, 25, 3000

-- MARKER inserts a lo/hi/lo slam before each ramp: a sharp edge to measure
-- end-to-end latency with. It shows up as a spike at the bottom of the sweep.
local MARKER = false

local PHASES, CYCLE = {}, 0

local function rebuild()
  PHASES = {
    {RAMP, -100,  100, "ramp up"},
    {RAMP,  100, -100, "ramp dn"},
  }
  if MARKER then
    table.insert(PHASES, 1, {SLAM, -100, -100, "mark lo"})
    table.insert(PHASES, 1, {SLAM,  100,  100, "mark hi"})
    table.insert(PHASES, 1, {SLAM, -100, -100, "mark lo"})
  end
  CYCLE = 0
  for i = 1, #PHASES do CYCLE = CYCLE + PHASES[i][1] end
end

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
  rebuild()
  t0 = getTime()
end

local function run(event)
  if event == EVT_VIRTUAL_EXIT or event == EVT_EXIT_BREAK then
    model.setGlobalVariable(GV, FM, -100)
    return 1
  end

  if event == EVT_VIRTUAL_INC or event == EVT_VIRTUAL_NEXT then
    RAMP = math.min(RAMP_MAX, RAMP + STEP)
    rebuild()
    t0 = getTime()
  elseif event == EVT_VIRTUAL_DEC or event == EVT_VIRTUAL_PREV then
    RAMP = math.max(RAMP_MIN, RAMP - STEP)
    rebuild()
    t0 = getTime()
  end

  local el = getTime() - t0
  local v, phase = valueAt(el % CYCLE)
  v = math.floor(v + 0.5)
  model.setGlobalVariable(GV, FM, v)

  lcd.clear()
  lcd.drawText(1, 1, "CRSF Sweep", INVERS)
  lcd.drawText(1, 12, string.format("ramp %.2fs  cyc %.2fs", RAMP / 100, CYCLE / 100))
  lcd.drawText(1, 23, string.format("GV%d = %d%%  %s", GV + 1, v, phase))
  lcd.drawText(1, 34, "+/- or wheel = speed")
  lcd.drawText(1, 45, "EXIT stops")
  return 0
end

return {init = init, run = run, name = toolName}
