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
CRUISE, APPROACH, COMMIT, CLEAR_WALL, SCAN = range(5)
STATE_NAMES = ["CRUISE", "APPROACH", "COMMIT", "CLEAR", "SCAN"]

state = CRUISE
prev_error = 0.0
prev_had_green = False
no_green_frames = 0
clear_counter = 0
commit_counter = 0
scan_counter = 0
scan_direction = 1
peak_color = 0

# ── Tuning parameters ───────────────────────────────────
Kp = 2.2
Kd = 0.8
Kp_commit = 0.8
RED_AVOID_K = 1.8
GREEN_THRESH = 3
RED_THRESH = 5
COMMIT_CX = 0.20
COMMIT_NEAR_G = 3
CLEAR_STEPS = 5
COMMIT_STEPS = 15
SCAN_DELAY = 20
SCAN_360 = 75
WALL_K = 0.0012
WALL_HARD = 200


def analyze_camera():
    """Per-pixel weighted centroid for green and red regions."""
    image = camera.getImage()
    y0 = H // 4

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

    has_g = n_green >= GREEN_THRESH
    has_r = n_red >= RED_THRESH
    total_color = n_green + n_red

    no_green_frames = 0 if has_g else no_green_frames + 1

    if state in (APPROACH, COMMIT):
        peak_color = max(peak_color, total_color)

    # ── State transitions ────────────────────────────────
    if state == CRUISE:
        if has_g or has_r:
            state = APPROACH
            peak_color = total_color

    elif state == APPROACH:
        if has_g and abs(gcx) < COMMIT_CX and near_g >= COMMIT_NEAR_G:
            state = COMMIT
            commit_counter = COMMIT_STEPS
        elif no_green_frames > SCAN_DELAY and not has_r:
            if peak_color > 15:
                # Was near a wall -- green vanished at close range, push through
                state = COMMIT
                commit_counter = COMMIT_STEPS
            else:
                # Genuinely lost, no wall nearby -- stop and scan
                state = SCAN
                scan_counter = 0

    elif state == COMMIT:
        # Ignore the camera entirely -- just drive straight through
        commit_counter -= 1
        if commit_counter <= 0:
            state = CLEAR_WALL
            clear_counter = CLEAR_STEPS
            peak_color = 0

    elif state == CLEAR_WALL:
        # FULL STOP -- then check if next wall is already visible
        clear_counter -= 1
        if clear_counter <= 0:
            if has_g:
                # Next wall's green already visible ahead -- skip scan
                state = APPROACH
                peak_color = total_color
            else:
                # Nothing ahead -- rotate to find green
                state = SCAN
                scan_counter = 0
                scan_direction *= -1
                peak_color = 0

    elif state == SCAN:
        scan_counter += 1
        # Ignore first 15 frames (~100° of rotation) so we turn away
        # from the wall behind us before we start looking for green
        if scan_counter > 15 and has_g:
            # Found green -- head toward it
            state = APPROACH
            peak_color = total_color
        elif scan_counter > SCAN_360:
            # Full 360 done, no green found -- drive forward a bit
            state = CRUISE
            peak_color = 0

    # ── Compute motor speeds ─────────────────────────────
    base = 0.85 * MAX_SPEED
    steer = 0.0

    if state == CRUISE:
        base = 0.90 * MAX_SPEED

    elif state == APPROACH:
        base = max(0.35, 0.70 - fill * 3.0) * MAX_SPEED

        if has_g and abs(gcx) < 0.15:
            base = max(base, 0.60 * MAX_SPEED)
        elif has_g and abs(gcx) > 0.4:
            base *= 0.80

        if has_g:
            err = gcx
            d_err = (err - prev_error) if prev_had_green else 0.0
            sf = Kp * err + Kd * d_err
            sf = max(-1.0, min(1.0, sf))
            steer = sf * 0.50 * MAX_SPEED
            prev_error = err
        elif no_green_frames < 5:
            # Green just disappeared (very close to wall) -- hold heading
            base = 0.55 * MAX_SPEED
            steer = 0.0
        elif has_r:
            steer = -RED_AVOID_K * rcx * 0.40 * MAX_SPEED
            steer = max(-0.50 * MAX_SPEED, min(0.50 * MAX_SPEED, steer))

    elif state == COMMIT:
        base = 0.70 * MAX_SPEED
        # Zero steering -- drive perfectly straight, no scanning

    elif state == CLEAR_WALL:
        base = 0.0
        steer = 0.0

    elif state == SCAN:
        # Pure in-place rotation -- zero forward speed
        base = 0.0
        steer = scan_direction * 0.40 * MAX_SPEED

    # ── Side-wall centering (gentle, always-on except COMMIT/SCAN) ─
    if state not in (COMMIT, SCAN, CLEAR_WALL):
        sc = WALL_K * (left_w - right_w) * MAX_SPEED
        sc = max(-0.12 * MAX_SPEED, min(0.12 * MAX_SPEED, sc))
        steer += sc

    # Hard side-wall avoidance (disabled while tracking green)
    if state not in (COMMIT, CLEAR_WALL) and not (state == APPROACH and has_g):
        if left_w > WALL_HARD:
            steer = max(steer, 0.35 * MAX_SPEED)
            base = min(base, 0.45 * MAX_SPEED)
        if right_w > WALL_HARD:
            steer = min(steer, -0.35 * MAX_SPEED)
            base = min(base, 0.45 * MAX_SPEED)

    # ── Apply ────────────────────────────────────────────
    ls = max(-MAX_SPEED, min(MAX_SPEED, base + steer))
    rs = max(-MAX_SPEED, min(MAX_SPEED, base - steer))
    left_motor.setVelocity(ls)
    right_motor.setVelocity(rs)

    prev_had_green = has_g

    print(
        f"{STATE_NAMES[state]:7} gcx:{gcx:+.2f} G:{n_green:3} R:{n_red:3} "
        f"fill:{fill:.3f} L:{left_w:.0f} R:{right_w:.0f}"
    )
