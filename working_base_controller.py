from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
MAX_SPEED = 6.28

camera = robot.getDevice('camera')
camera.enable(timestep)
W = camera.getWidth()
H = camera.getHeight()

sensors = []
for i in range(8):
    s = robot.getDevice(f'ps{i}')
    s.enable(timestep)
    sensors.append(s)

# ── States ───────────────────────────────────────────────
CRUISE, APPROACH, COMMIT, CLEAR_WALL, ALIGN, SCAN = range(6)
STATE_NAMES = ["CRUISE", "APPROACH", "COMMIT", "CLEAR", "ALIGN", "SCAN"]

state = CRUISE
prev_error = 0.0
prev_had_green = False
no_green_frames = 0
clear_counter = 0
commit_counter = 0
scan_counter = 0
align_counter = 0
creep_counter = 0
stuck_counter = 0
reverse_counter = 0
no_color_frames = 0
scan_saw_red = False
peak_color = 0

# ── Tuning parameters ───────────────────────────────────
Kp = 2.5
Kd = 0.9
RED_AVOID_K = 1.8
GREEN_THRESH = 3
RED_THRESH = 5
COMMIT_CX = 0.20
COMMIT_NEAR_G = 3
CLEAR_STEPS = 2
COMMIT_STEPS = 90
SCAN_DELAY = 20
WALL_K = 0.0012
WALL_HARD = 200

ALIGN_TIMEOUT = 25
CREEP_STEPS = 12
STUCK_SENSOR = 150
STUCK_FRAMES = 12
REVERSE_FRAMES = 8
NO_COLOR_TIMEOUT = 20

# Scan geometry (~9°/frame at 0.50 scan speed)
SCAN_LEFT = 16       # frames to turn left ~145°
SCAN_SWEEP = 60      # frames to sweep right ~540° from leftmost position
SCAN_360 = 40        # frames for fallback 360° spin
SCAN_SPEED = 0.50


def analyze_camera():
    """Per-pixel weighted centroid for green and red regions."""
    image = camera.getImage()
    y0 = H // 6

    g_xw = 0.0; g_wt = 0.0; n_g = 0
    r_xw = 0.0; r_wt = 0.0; n_r = 0
    near_g = 0; near_r = 0
    near_y = 2 * H // 3

    inv_w = 2.0 / max(1, W - 1)
    inv_h = 1.0 / max(1, H - 1 - y0)

    for y in range(y0, H):
        rw = 1.0 + 2.0 * (y - y0) * inv_h
        for x in range(W):
            rv = camera.imageGetRed(image, W, x, y)
            gv = camera.imageGetGreen(image, W, x, y)
            bv = camera.imageGetBlue(image, W, x, y)
            xn = x * inv_w - 1.0

            if gv > 1.15 * rv and gv > 1.15 * bv and gv > 55:
                n_g += 1
                g_xw += xn * rw
                g_wt += rw
                if y >= near_y:
                    near_g += 1
            elif rv > 1.15 * gv and rv > 1.15 * bv and rv > 55:
                n_r += 1
                r_xw += xn * rw
                r_wt += rw
                if y >= near_y:
                    near_r += 1

    gcx = g_xw / g_wt if g_wt > 0 else 0.0
    rcx = r_xw / r_wt if r_wt > 0 else 0.0
    fill = (n_g + n_r) / max(1, (H - y0) * W)
    return gcx, n_g, rcx, n_r, fill, near_g, near_r


# ── Main loop ────────────────────────────────────────────
while robot.step(timestep) != -1:
    gcx, n_green, rcx, n_red, fill, near_g, near_r = analyze_camera()

    ps = [s.getValue() for s in sensors]
    left_w = ps[5]
    right_w = ps[2]
    front_l = ps[7]
    front_r = ps[0]

    has_g = n_green >= GREEN_THRESH
    has_r = n_red >= RED_THRESH
    total_color = n_green + n_red

    # ── Stuck detection (grey wall ahead, no color) ──
    front_blocked = front_l > STUCK_SENSOR and front_r > STUCK_SENSOR
    if not has_g and not has_r and front_blocked:
        stuck_counter += 1
    else:
        stuck_counter = 0

    if stuck_counter > STUCK_FRAMES:
        reverse_counter = REVERSE_FRAMES
        stuck_counter = 0

    no_color_frames = 0 if (has_g or has_r) else no_color_frames + 1
    no_green_frames = 0 if has_g else no_green_frames + 1

    if state in (APPROACH, COMMIT):
        peak_color = max(peak_color, total_color)

    # ── State transitions ────────────────────────────────
    if state == CRUISE:
        if has_g or has_r:
            state = APPROACH
            peak_color = total_color
            no_color_frames = 0
        elif creep_counter > 0:
            creep_counter -= 1
            if creep_counter <= 0:
                state = SCAN
                scan_counter = 0
        elif no_color_frames > NO_COLOR_TIMEOUT:
            state = SCAN
            scan_counter = 0
            no_color_frames = 0

    elif state == APPROACH:
        if has_g and abs(gcx) < COMMIT_CX and near_g >= COMMIT_NEAR_G:
            state = COMMIT
            commit_counter = COMMIT_STEPS
        elif no_green_frames > SCAN_DELAY and not has_r:
            if peak_color > 15:
                state = COMMIT
                commit_counter = COMMIT_STEPS
            else:
                state = SCAN
                scan_counter = 0
        elif no_green_frames > SCAN_DELAY and has_r:
            # Only red visible, green must be elsewhere -- stop and scan
            state = SCAN
            scan_counter = 0
            peak_color = 0

    elif state == COMMIT:
        commit_counter -= 1
        if commit_counter <= 0:
            state = CLEAR_WALL
            clear_counter = CLEAR_STEPS
            peak_color = 0

    elif state == CLEAR_WALL:
        clear_counter -= 1
        if clear_counter <= 0:
            if has_g and abs(gcx) <= 0.15:
                state = APPROACH
                peak_color = total_color
            elif has_g:
                state = ALIGN
                align_counter = ALIGN_TIMEOUT
            else:
                state = SCAN
                scan_counter = 0
                peak_color = 0

    elif state == ALIGN:
        align_counter -= 1
        if abs(gcx) < 0.15 and has_g:
            state = APPROACH
            peak_color = total_color
        elif align_counter <= 0:
            if has_g:
                state = APPROACH
                peak_color = total_color
            else:
                state = SCAN
                scan_counter = 0
                peak_color = 0

    elif state == SCAN:
        scan_counter += 1
        if has_r:
            scan_saw_red = True
        if scan_counter > SCAN_LEFT and has_g:
            state = APPROACH
            peak_color = total_color
        elif scan_counter > SCAN_LEFT + SCAN_SWEEP + SCAN_360:
            if scan_saw_red:
                # Red everywhere -- too close to wall, back up then scan again
                reverse_counter = REVERSE_FRAMES
            else:
                # Saw nothing -- creep forward to get closer
                state = CRUISE
                creep_counter = CREEP_STEPS
            scan_saw_red = False
            peak_color = 0

    # ── Compute motor speeds ─────────────────────────────
    base = 0.85 * MAX_SPEED
    steer = 0.0

    if state == CRUISE:
        base = 0.65 * MAX_SPEED if creep_counter > 0 else MAX_SPEED

    elif state == APPROACH:
        base = max(0.50, 0.80 - fill * 3.0) * MAX_SPEED

        if has_g and abs(gcx) < 0.15:
            base = max(base, 0.80 * MAX_SPEED)
        elif has_g and abs(gcx) > 0.35:
            base *= 0.65

        if has_g:
            err = gcx
            d_err = (err - prev_error) if prev_had_green else 0.0
            sf = Kp * err + Kd * d_err
            sf = max(-1.0, min(1.0, sf))
            steer = sf * 0.55 * MAX_SPEED
            prev_error = err
        elif no_green_frames < 5:
            base = 0.65 * MAX_SPEED
            steer = 0.0
        elif has_r:
            steer = -RED_AVOID_K * rcx * 0.45 * MAX_SPEED
            steer = max(-0.55 * MAX_SPEED, min(0.55 * MAX_SPEED, steer))

    elif state == COMMIT:
        base = 0.90 * MAX_SPEED

    elif state == CLEAR_WALL:
        base = 0.0
        steer = 0.0

    elif state == ALIGN:
        base = 0.0
        if has_g:
            steer = gcx * 1.5 * MAX_SPEED
            steer = max(-0.45 * MAX_SPEED, min(0.45 * MAX_SPEED, steer))
        else:
            steer = 0.0

    elif state == SCAN:
        base = 0.0
        if scan_counter <= SCAN_LEFT:
            steer = -SCAN_SPEED * MAX_SPEED
        else:
            steer = SCAN_SPEED * MAX_SPEED

    # ── Side-wall centering (gentle, always-on except COMMIT/SCAN) ─
    if state not in (COMMIT, SCAN, CLEAR_WALL, ALIGN):
        sc = WALL_K * (left_w - right_w) * MAX_SPEED
        sc = max(-0.12 * MAX_SPEED, min(0.12 * MAX_SPEED, sc))
        steer += sc

    # Hard side-wall avoidance (disabled while tracking green)
    if state not in (COMMIT, CLEAR_WALL, ALIGN) and not (state == APPROACH and has_g):
        if left_w > WALL_HARD:
            steer = max(steer, 0.35 * MAX_SPEED)
            base = min(base, 0.45 * MAX_SPEED)
        if right_w > WALL_HARD:
            steer = min(steer, -0.35 * MAX_SPEED)
            base = min(base, 0.45 * MAX_SPEED)

    # ── Reverse override if stuck on grey wall ──────────
    if reverse_counter > 0:
        reverse_counter -= 1
        base = -0.40 * MAX_SPEED
        steer = 0.0
        if reverse_counter <= 0:
            state = SCAN
            scan_counter = 0
            peak_color = 0

    # ── Apply ────────────────────────────────────────────
    ls = max(-MAX_SPEED, min(MAX_SPEED, base + steer))
    rs = max(-MAX_SPEED, min(MAX_SPEED, base - steer))
    left_motor.setVelocity(ls)
    right_motor.setVelocity(rs)

    prev_had_green = has_g

    print(
        f"{STATE_NAMES[state]:7} gcx:{gcx:+.2f} G:{n_green:3} R:{n_red:3} "
        f"fill:{fill:.3f} nrG:{near_g:3} sc:{scan_counter:3}"
    )
