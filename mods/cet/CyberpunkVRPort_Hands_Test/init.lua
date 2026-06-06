-- VR controller raw tracking visualizer.
-- Draws world-space hand gizmos with DebugVisualizer so we can validate OpenXR
-- controller pose math without spawning props or touching weapons.

local isReady = false
local drawEnabled = true
local hideNativeArms = false -- kept for UI only; VRFPP hide path is disabled
local gizmoScale = 1.0
local dumpStatus = "idle"
local chunkDebugEnabled = false
local chunkDebugComponentIndex = 51
local chunkDebugHand = 1
local chunkDebugUseFullMask = true
local chunkDebugBit0 = 0
local chunkDebugBit1 = -1
local chunkDebugBit2 = -1
local chunkDebugBit3 = -1
local needRestoreArms = true

-- Per-hand IK calibration (defaults mirror the baked plugin defaults). Exposed via
-- ImGui sliders so users with different arm lengths/heights can tune live.
local calib = {
    scaleR = 1.05, scaleL = 1.06,
    heightR = 0.23, heightL = 0.23,
    poleR = 0.0, poleL = 0.0,
    swingR = 1.0, swingL = 1.0,
    applied = false,
}

local function num(v, d) if type(v) == 'number' then return v else return d end end

local function applyCalib()
    -- Guard every value: a non-number (e.g. an ImGui binding quirk) must never reach
    -- the plugin as scale, or both hands collapse to the anchor ("hands from the belly").
    local sR, sL = num(calib.scaleR, 1.05), num(calib.scaleL, 1.06)
    local hR, hL = num(calib.heightR, 0.23), num(calib.heightL, 0.23)
    local pR, pL = num(calib.poleR, 0.0), num(calib.poleL, 0.0)
    local wR, wL = num(calib.swingR, 1.0), num(calib.swingL, 1.0)
    calib.scaleR, calib.scaleL = sR, sL
    calib.heightR, calib.heightL = hR, hL
    calib.poleR, calib.poleL = pR, pL
    calib.swingR, calib.swingL = wR, wL
    if type(SetVRBindParams) == 'function' then
        pcall(function() SetVRBindParams(sR, 0.0, 0.0, hR, 1, 0) end) -- hand 0 = right
        pcall(function() SetVRBindParams(sL, 0.0, 0.0, hL, 1, 1) end) -- hand 1 = left
    end
    if type(SetVRElbowPole) == 'function' then
        pcall(function() SetVRElbowPole(pR, 0) end)
        pcall(function() SetVRElbowPole(pL, 1) end)
    end
    if type(SetVRElbowSwing) == 'function' then
        pcall(function() SetVRElbowSwing(wR, 0) end)
        pcall(function() SetVRElbowSwing(wL, 1) end)
    end
end

local status = {
    debugVisualizer = false,
    debugSource = "none",
    debugHistory = false,
    lastDrawErr = "none",
    leftValid = false,
    rightValid = false,
    leftRaw = "n/a",
    rightRaw = "n/a",
    leftWorld = "n/a",
    rightWorld = "n/a",
}

local function v4(x, y, z, w)
    return Vector4.new(x, y, z, w or 0.0)
end

local function add(a, b)
    return v4(a.x + b.x, a.y + b.y, a.z + b.z, 1.0)
end

local function sub(a, b)
    return v4(a.x - b.x, a.y - b.y, a.z - b.z, 1.0)
end

local function mul(v, s)
    return v4(v.x * s, v.y * s, v.z * s, 0.0)
end

local function vecStr(v)
    if not v then return "nil" end
    return string.format("%.2f %.2f %.2f", v.x, v.y, v.z)
end

local function makeColor(r, g, b, a)
    local ok, c = pcall(function()
        return Color.new(r, g, b, a or 255)
    end)
    if ok and c then return c end
    ok, c = pcall(function()
        return Color.new({ Red = r, Green = g, Blue = b, Alpha = a or 255 })
    end)
    if ok and c then return c end
    return nil
end

local BODY_LEFT = makeColor(0, 220, 255, 255)
local BODY_RIGHT = makeColor(255, 180, 0, 255)
local AXIS_RIGHT = makeColor(255, 64, 64, 255)
local AXIS_UP = makeColor(64, 255, 64, 255)
local AXIS_FWD = makeColor(64, 128, 255, 255)
local AXIS_TEXT = makeColor(255, 255, 255, 255)

local function setChunkPreset(index, hand, fullMask)
    chunkDebugEnabled = true
    chunkDebugUseFullMask = fullMask and true or false
    chunkDebugComponentIndex = index
    chunkDebugHand = hand
end

local function getMatrixMath()
    return GetSingleton('Matrix')
end

local function getQuatMath()
    return GetSingleton('Quaternion')
end

local function getDebugVisualizer(player)
    local ok, dvs = pcall(function()
        return GameInstance.GetDebugVisualizerSystem(player:GetGame())
    end)
    if ok and dvs then
        status.debugSource = 'GameInstance.GetDebugVisualizerSystem'
        return dvs
    end
    ok, dvs = pcall(function() return GetSingleton('gameDebugVisualizerSystem') end)
    if ok and dvs then
        status.debugSource = "GetSingleton('gameDebugVisualizerSystem')"
        return dvs
    end
    ok, dvs = pcall(function() return GetSingleton('DebugVisualizerSystem') end)
    if ok and dvs then
        status.debugSource = "GetSingleton('DebugVisualizerSystem')"
        return dvs
    end
    status.debugSource = 'none'
    return nil
end

local function getDebugHistory(player)
    local ok, ddh = pcall(function()
        return GameInstance.GetDebugDrawHistorySystem(player:GetGame())
    end)
    if ok and ddh then
        status.debugHistory = true
        return ddh
    end
    status.debugHistory = false
    return nil
end

local function getCameraWorldPose(player)
    local camera = player:GetFPPCameraComponent()
    if not camera then return nil, nil end
    local l2w = camera:GetLocalToWorld()
    if not l2w then return nil, nil end
    local mat = getMatrixMath()
    if not mat then return nil, nil end
    local camPos = mat:GetTranslation(l2w)
    local camQuat = mat:ToQuat(l2w)
    return camPos, camQuat
end

local function mapLocalPos(rawPos)
    return v4(rawPos.x, -rawPos.z, rawPos.y, 0.0)
end

local function mapLocalQuat(rawQuat)
    return Quaternion.new(rawQuat.i, -rawQuat.k, rawQuat.j, rawQuat.r)
end

local function getHandWorldPose(isLeft, camPos, camQuat)
    local validFn = isLeft and GetLeftVRHandValid or GetRightVRHandValid
    local posFn = isLeft and GetLeftVRHandPos or GetRightVRHandPos
    local rotFn = isLeft and GetLeftVRHandRot or GetRightVRHandRot
    if type(validFn) ~= 'function' or type(posFn) ~= 'function' or type(rotFn) ~= 'function' then
        return nil
    end

    local valid = validFn()
    if isLeft then
        status.leftValid = valid
    else
        status.rightValid = valid
    end
    if not valid then return nil end

    local rawPos = posFn()
    local rawQuat = rotFn()
    if isLeft then
        status.leftRaw = vecStr(rawPos)
    else
        status.rightRaw = vecStr(rawPos)
    end

    local quatMath = getQuatMath()
    if not quatMath then return nil end

    local localPos = mapLocalPos(rawPos)
    local localQuat = mapLocalQuat(rawQuat)

    local worldOffset = quatMath:Transform(camQuat, localPos)
    local worldPos = add(camPos, worldOffset)

    local localForward = quatMath:GetForward(localQuat)
    local localRight = quatMath:GetRight(localQuat)
    local localUp = quatMath:GetUp(localQuat)

    local worldForward = quatMath:Transform(camQuat, localForward)
    local worldRight = quatMath:Transform(camQuat, localRight)
    local worldUp = quatMath:Transform(camQuat, localUp)

    if isLeft then
        status.leftWorld = vecStr(worldPos)
    else
        status.rightWorld = vecStr(worldPos)
    end

    return {
        pos = worldPos,
        forward = worldForward,
        right = worldRight,
        up = worldUp,
    }
end

local function drawLine(dvs, a, b, color, life)
    local ok, err = pcall(function() dvs:DrawLine3D(a, b, color, life) end)
    if not ok then
        ok = pcall(function() dvs:DrawLine3D(a, b, color) end)
    end
    if not ok then
        ok = pcall(function() dvs:DrawLine3D(a, b) end)
    end
    if not ok then
        status.lastDrawErr = tostring(err)
    end
end

local function drawArrow(dvs, a, b, color, life)
    local ok, err = pcall(function() dvs:DrawArrow(a, b, color, life) end)
    if not ok then
        ok = pcall(function() dvs:DrawArrow(a, b, color) end)
    end
    if not ok then
        ok = pcall(function() dvs:DrawArrow(a, b) end)
    end
    if not ok then
        status.lastDrawErr = tostring(err)
    end
end

local function drawSphere(dvs, pos, radius, color, life)
    local ok, err = pcall(function() dvs:DrawWireSphere(pos, radius, color, life) end)
    if not ok then
        ok = pcall(function() dvs:DrawWireSphere(pos, radius, color) end)
    end
    if not ok then
        ok = pcall(function() dvs:DrawWireSphere(pos, radius) end)
    end
    if not ok then
        status.lastDrawErr = tostring(err)
    end
end

local function drawText3D(dvs, pos, text, color, life)
    local ok, err = pcall(function() dvs:DrawText3D(pos, text, color, life) end)
    if not ok then
        ok = pcall(function() dvs:DrawText3D(pos, text, color) end)
    end
    if not ok then
        ok = pcall(function() dvs:DrawText3D(pos, text) end)
    end
    if not ok then
        status.lastDrawErr = tostring(err)
    end
end

local function drawHistSphere(ddh, pos, radius, color, tag)
    local ok, err = pcall(function() ddh:DrawWireSphere(pos, radius, color, tag) end)
    if not ok then
        ok = pcall(function() ddh:DrawWireSphere(pos, radius, color, tostring(tag)) end)
    end
    if not ok then
        status.lastDrawErr = tostring(err)
    end
end

local function drawHistArrow(ddh, pos, direction, color, tag)
    local ok, err = pcall(function() ddh:DrawArrow(pos, direction, color, tag) end)
    if not ok then
        ok = pcall(function() ddh:DrawArrow(pos, direction, color, tostring(tag)) end)
    end
    if not ok then
        status.lastDrawErr = tostring(err)
    end
end

local function drawHandGizmo(dvs, name, hand, bodyColor)
    local life = 0.06
    local core = 0.025 * gizmoScale
    local side = 0.055 * gizmoScale
    local lift = 0.060 * gizmoScale
    local reach = 0.220 * gizmoScale

    local pos = hand.pos
    local rightPos = add(pos, mul(hand.right, side))
    local leftPos = sub(pos, mul(hand.right, side))
    local upPos = add(pos, mul(hand.up, lift))
    local downPos = sub(pos, mul(hand.up, lift))
    local fwdStart = sub(pos, mul(hand.forward, 0.020 * gizmoScale))
    local fwdEnd = add(pos, mul(hand.forward, reach))
    local palmTop = add(upPos, mul(hand.right, side * 0.45))
    local palmBottom = add(downPos, mul(hand.right, side * 0.45))
    local palmTopL = sub(upPos, mul(hand.right, side * 0.45))
    local palmBottomL = sub(downPos, mul(hand.right, side * 0.45))

    drawSphere(dvs, pos, core, bodyColor, life)
    drawLine(dvs, leftPos, rightPos, AXIS_RIGHT, life)
    drawLine(dvs, downPos, upPos, AXIS_UP, life)
    drawLine(dvs, palmBottomL, palmTopL, bodyColor, life)
    drawLine(dvs, palmBottom, palmTop, bodyColor, life)
    drawLine(dvs, palmTopL, palmTop, bodyColor, life)
    drawLine(dvs, palmBottomL, palmBottom, bodyColor, life)
    drawArrow(dvs, fwdStart, fwdEnd, AXIS_FWD, life)
    drawSphere(dvs, fwdEnd, core * 0.45, AXIS_FWD, life)

    drawText3D(dvs, add(upPos, mul(hand.up, core * 1.6)), name, bodyColor, life)
    drawText3D(dvs, add(rightPos, mul(hand.right, core * 0.9)), "R", AXIS_TEXT, life)
    drawText3D(dvs, sub(leftPos, mul(hand.right, core * 0.9)), "L", AXIS_TEXT, life)
    drawText3D(dvs, add(upPos, mul(hand.up, core * 0.9)), "U", AXIS_TEXT, life)
    drawText3D(dvs, sub(downPos, mul(hand.up, core * 0.9)), "D", AXIS_TEXT, life)
    drawText3D(dvs, add(fwdEnd, mul(hand.forward, core * 0.8)), "F", AXIS_TEXT, life)
end

registerForEvent('onInit', function()
    isReady = true
end)

local vrTrackingEnabled = false
local mouseDisableEnabled = true  -- Mouse/look pitch (Y) disabled by default for VR

registerHotkey('ToggleVRHands', 'Toggle VR Hands', function()
    vrTrackingEnabled = not vrTrackingEnabled
    if vrTrackingEnabled then
        pcall(function() Game.InstallVRAnimPoseHook() end)
        pcall(function() Game.ArmVRAnimPosePlayer() end)
        pcall(function() Game.SetVRBindMode(4) end)  -- 4 = full-arm model-space IK
    else
        pcall(function() Game.SetVRBindMode(0) end)
    end
end)

registerHotkey('ToggleMouseY', 'Toggle Disable Mouse Y', function()
    mouseDisableEnabled = not mouseDisableEnabled
end)

-- Last FPP camera world pose, captured each frame for the diagnostic logger.
local lastCamPos = nil
local lastCamQuat = nil

registerHotkey('LogVRDiag', 'Log VR Hand Diagnostic', function()
    if type(SetVRDiagCapture) == 'function' then pcall(function() SetVRDiagCapture(1) end) end
    if type(LogVRDiag) == 'function' and lastCamPos and lastCamQuat then
        pcall(function()
            LogVRDiag(lastCamPos.x, lastCamPos.y, lastCamPos.z,
                      lastCamQuat.i, lastCamQuat.j, lastCamQuat.k, lastCamQuat.r)
        end)
    end
end)

registerForEvent('onUpdate', function(dt)
    if not isReady then return end
    
    local player = Game.GetPlayer()
    if not player then return end

    if mouseDisableEnabled then
        local cam = player:GetFPPCameraComponent()
        if cam then
            -- Force pitch to 0
            cam.pitchMax = 0.0
            cam.pitchMin = 0.0
        end
    else
        local cam = player:GetFPPCameraComponent()
        if cam then
            -- Restore defaults roughly
            cam.pitchMax = 80.0
            cam.pitchMin = -80.0
        end
    end
    -- We only do a one-shot restore in case a previous session hid the arms.
    if needRestoreArms and type(RestoreVRFppArms) == 'function' then
        pcall(function() RestoreVRFppArms() end)
        needRestoreArms = false
    end

    if type(UpdateVRIKAnimInputs) == 'function' then
        pcall(function() UpdateVRIKAnimInputs() end)
    end

    local player = Game.GetPlayer()
    if not player then return end
    
    -- VR Transforms Update for Model-Space IK
    local camPos, camQuat = getCameraWorldPose(player)
    if not camPos or not camQuat then return end

    -- Remember the camera pose for the diagnostic logger hotkey, and keep the
    -- pre-write bone snapshot fresh while tracking is active.
    lastCamPos = camPos
    lastCamQuat = camQuat
    if vrTrackingEnabled and type(SetVRDiagCapture) == 'function' then
        pcall(function() SetVRDiagCapture(1) end)
    end

    if type(SetVRPlayerYaw) == 'function' then
        local ok2, playerOri = pcall(function() return player:GetWorldOrientation() end)
        local yaw = 0.0
        if ok2 and playerOri and type(playerOri.yaw) == 'number' then
            yaw = playerOri.yaw
        end
        pcall(function() SetVRPlayerYaw(yaw, camQuat.i, camQuat.j, camQuat.k, camQuat.r) end)
        status.debugYaw = string.format("Yaw: %.2f", yaw)
    end

    if not dvs and not ddh then return end

    local leftHand = getHandWorldPose(true, camPos, camQuat)
    local rightHand = getHandWorldPose(false, camPos, camQuat)

    if leftHand then
        drawHandGizmo(dvs, "LEFT", leftHand, BODY_LEFT)
    else
        status.leftRaw = "n/a"
        status.leftWorld = "n/a"
    end

    if rightHand then
        drawHandGizmo(dvs, "RIGHT", rightHand, BODY_RIGHT)
    else
        status.rightRaw = "n/a"
        status.rightWorld = "n/a"
    end
end)

registerForEvent('onDraw', function()
    if not isReady then return end

    ImGui.SetNextWindowPos(100, 100, ImGuiCond.FirstUseEver)
    ImGui.SetNextWindowSize(500, 400, ImGuiCond.FirstUseEver)
    ImGui.Begin('VR Hands Control')

    ImGui.Separator()
    ImGui.Text("VR Tracking Controls")
    
    if ImGui.Button(vrTrackingEnabled and 'Stop VR Tracking' or 'Start VR Tracking') then
        vrTrackingEnabled = not vrTrackingEnabled
        if vrTrackingEnabled then
            pcall(function() Game.InstallVRAnimPoseHook() end)
            pcall(function() Game.ArmVRAnimPosePlayer() end)
            pcall(function() Game.SetVRBindMode(4) end)  -- 4 = full-arm model-space IK
            applyCalib()
        else
            pcall(function() Game.SetVRBindMode(0) end)
        end
    end
    
    local mouseChanged, newMouse = ImGui.Checkbox('Disable Mouse Y (Pitch)', mouseDisableEnabled)
    if mouseChanged then
        mouseDisableEnabled = newMouse
    end

    if ImGui.Button('Log VR Diag (gizmo vs bones)') then
        if type(SetVRDiagCapture) == 'function' then pcall(function() SetVRDiagCapture(1) end) end
        if type(LogVRDiag) == 'function' and lastCamPos and lastCamQuat then
            local ok = pcall(function()
                LogVRDiag(lastCamPos.x, lastCamPos.y, lastCamPos.z,
                          lastCamQuat.i, lastCamQuat.j, lastCamQuat.k, lastCamQuat.r)
            end)
            dumpStatus = ok and 'diag logged -> vrik_diag.txt' or 'diag log failed'
        else
            dumpStatus = 'diag: no cam pose yet (move once)'
        end
    end
    ImGui.Separator()
    ImGui.Text('Hand IK Calibration (per arm length / height)')

    -- CET ImGui.SliderFloat returns (value, changed) -- value FIRST.
    local used
    calib.scaleR, used = ImGui.SliderFloat('Reach scale R', num(calib.scaleR, 1.05), 0.90, 1.30)
    if used then applyCalib() end
    calib.scaleL, used = ImGui.SliderFloat('Reach scale L', num(calib.scaleL, 1.06), 0.90, 1.30)
    if used then applyCalib() end
    calib.heightR, used = ImGui.SliderFloat('Height R', num(calib.heightR, 0.23), -0.30, 0.60)
    if used then applyCalib() end
    calib.heightL, used = ImGui.SliderFloat('Height L', num(calib.heightL, 0.23), -0.30, 0.60)
    if used then applyCalib() end
    calib.poleR, used = ImGui.SliderFloat('Elbow pole R', num(calib.poleR, 0.0), -180.0, 180.0)
    if used then applyCalib() end
    calib.poleL, used = ImGui.SliderFloat('Elbow pole L', num(calib.poleL, 0.0), -180.0, 180.0)
    if used then applyCalib() end
    calib.swingR, used = ImGui.SliderFloat('Elbow swing R', num(calib.swingR, 1.0), 0.0, 3.0)
    if used then applyCalib() end
    calib.swingL, used = ImGui.SliderFloat('Elbow swing L', num(calib.swingL, 1.0), 0.0, 3.0)
    if used then applyCalib() end
    if ImGui.Button('Apply calibration') then applyCalib() end
    ImGui.SameLine()
    if ImGui.Button('Reset calibration') then
        calib.scaleR, calib.scaleL = 1.05, 1.06
        calib.heightR, calib.heightL = 0.23, 0.23
        calib.poleR, calib.poleL = 0.0, 0.0
        calib.swingR, calib.swingL = 1.0, 1.0
        applyCalib()
    end

    ImGui.Separator()

    ImGui.Text('DebugVisualizer: ' .. tostring(status.debugVisualizer))
    ImGui.Text('Debug source: ' .. status.debugSource)
    ImGui.Text('DebugHistory: ' .. tostring(status.debugHistory))
    if status.debugYaw then
        ImGui.Text('Player Yaw: ' .. status.debugYaw)
    end
    ImGui.Text('Last draw err: ' .. status.lastDrawErr)
    ImGui.Text('Draw gizmos: ' .. tostring(drawEnabled))
    ImGui.SameLine()
    if ImGui.Button(drawEnabled and 'Disable##gizmos' or 'Enable##gizmos') then
        drawEnabled = not drawEnabled
    end

    ImGui.Text('Hide native arms: ' .. tostring(hideNativeArms))
    ImGui.SameLine()
    if ImGui.Button(hideNativeArms and 'Show arms##toggle' or 'Hide arms##toggle') then
        hideNativeArms = not hideNativeArms
        -- Feature disabled; keep UI state but never call hide path.
        if not hideNativeArms then needRestoreArms = true end
    end

    if ImGui.Button('Dump FPP components') then
        if type(DumpVRFppComponents) == 'function' then
            local ok, result = pcall(function() return DumpVRFppComponents() end)
            if ok then
                dumpStatus = 'dumped ' .. tostring(result) .. ' components'
            else
                dumpStatus = 'dump failed: ' .. tostring(result)
            end
        else
            dumpStatus = 'native function missing'
        end
    end
    ImGui.SameLine()
    ImGui.Text(dumpStatus)

    ImGui.Separator()
    ImGui.Text('Character hand chunk debug:')
    ImGui.Text('FPP chunk debug disabled in this build')
    if ImGui.Button('Disable chunk debug') then
        chunkDebugEnabled = false
        needRestoreArms = true
    end

    if ImGui.Button('Bigger gizmos') then
        gizmoScale = gizmoScale + 0.15
    end
    ImGui.SameLine()
    if ImGui.Button('Smaller gizmos') then
        gizmoScale = math.max(0.35, gizmoScale - 0.15)
    end
    ImGui.SameLine()
    ImGui.Text(string.format('Scale %.2f', gizmoScale))

    ImGui.Separator()
    ImGui.Text('Left valid: ' .. tostring(status.leftValid))
    ImGui.Text('Left raw:   ' .. status.leftRaw)
    ImGui.Text('Left world: ' .. status.leftWorld)

    ImGui.Separator()
    ImGui.Text('Right valid: ' .. tostring(status.rightValid))
    ImGui.Text('Right raw:   ' .. status.rightRaw)
    ImGui.Text('Right world: ' .. status.rightWorld)

    ImGui.End()
end)
