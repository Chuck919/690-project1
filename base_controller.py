from controller import Robot

# -----------------------------
# Robot & Time Step
# -----------------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# -----------------------------
# Motors (Differential Drive)
# -----------------------------
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

MAX_SPEED = 6.28

# -----------------------------
# Camera
# -----------------------------
camera = robot.getDevice('camera')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()

# -----------------------------
# Distance Sensors
# -----------------------------
distance_sensors = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(timestep)
    distance_sensors.append(sensor)

# -----------------------------
# State Tracking
# -----------------------------
straighten_counter = 0
previous_total_green = 0
just_passed_wall = False
no_green_counter = 0
scan_mode = False
scan_timer = 0
forward_after_wall_counter = 0

# -----------------------------
# Vision: Improved color detection
# -----------------------------
def detect_colors_with_center():
    """
    Detects colors and specifically checks if green is centered (in front)
    """
    image = camera.getImage()
    
    # Look at bottom 2/3 of image for better wall detection
    y_start = height // 3
    y_end = height - 5
    
    green_left = 0
    green_center = 0
    green_right = 0
    red_left = 0
    red_center = 0
    red_right = 0
    
    # Define regions - narrower center for better centering
    left_bound = width // 3
    right_bound = 2 * width // 3
    
    for y in range(y_start, y_end, 2):  # Sample every 2 pixels
        for x in range(0, width, 2):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)
            
            # Green detection (slightly relaxed for arrows)
            if g > 1.2 * r and g > 1.2 * b and g > 70:
                if x < left_bound:
                    green_left += 1
                elif x > right_bound:
                    green_right += 1
                else:
                    green_center += 1
            
            # Red detection
            elif r > 1.2 * g and r > 1.2 * b and r > 70:
                if x < left_bound:
                    red_left += 1
                elif x > right_bound:
                    red_right += 1
                else:
                    red_center += 1
    
    total_green = green_left + green_center + green_right
    total_red = red_left + red_center + red_right
    
    return green_left, green_center, green_right, red_left, red_center, red_right, total_green, total_red

# -----------------------------
# Main Control Loop
# -----------------------------
while robot.step(timestep) != -1:
    
    # Get color detections
    green_left, green_center, green_right, red_left, red_center, red_right, total_green, total_red = detect_colors_with_center()
    
    # Read distance sensors
    ps_values = [sensor.getValue() for sensor in distance_sensors]
    front_dist = max(ps_values[0], ps_values[7])
    left_side = ps_values[5]
    right_side = ps_values[2]
    print("Left" ,left_side)
    print("Right", right_side)
    # -----------------------------
    # DETECT WALL PASSAGE
    # -----------------------------
    # Wall passed when: had green before, now no green, and not near obstacle
    if previous_total_green > 20 and total_green < 5 and front_dist < 70:
        just_passed_wall = True
        forward_after_wall_counter = 40  # Move forward more after passing wall
        scan_mode = False
        scan_timer = 0
    
    # Track no green frames
    if total_green < 5:
        no_green_counter += 1
    else:
        no_green_counter = 0
        if scan_mode:
            # Found green during scan - exit and straighten
            scan_mode = False
            scan_timer = 0
            straighten_counter = 10
    
    # Enter scan mode if no green after moving forward past wall
    if no_green_counter > 15 and straighten_counter == 0 and forward_after_wall_counter == 0 and not scan_mode:
        scan_mode = True
        scan_timer = 0
        no_green_counter = 0
    
    previous_total_green = total_green
    
    # BASE SPEED
    base_speed = 0.90 * MAX_SPEED
    left_speed = base_speed
    right_speed = base_speed
    
    # -----------------------------
    # FORWARD AFTER WALL (Priority 0 - Move past wall first)
    # -----------------------------
    if forward_after_wall_counter > 0:
        # Keep going straight to clear the wall
        left_speed = 0.92 * MAX_SPEED
        right_speed = 0.92 * MAX_SPEED
        forward_after_wall_counter -= 1
        
        if forward_after_wall_counter == 0:
            just_passed_wall = False
    
    # -----------------------------
    # SCAN MODE - 360° rotation (Priority 1)
    # -----------------------------
    elif scan_mode:
        scan_timer += 1
        
        # Rotate in place (right turn) - 360 degrees
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
        
        # After full rotation, if still no green, give up and go straight
        if scan_timer > 100:
            scan_mode = False
            scan_timer = 0
            straighten_counter = 20
    
    # -----------------------------
    # STRAIGHTENING MODE (Priority 2)
    # -----------------------------
    elif straighten_counter > 0:
        # Force straight movement to reset orientation
        left_speed = 0.92 * MAX_SPEED
        right_speed = 0.92 * MAX_SPEED
        straighten_counter -= 1
    
    # -----------------------------
    # GREEN CENTERED - GO THROUGH (Priority 3)
    # -----------------------------
    elif green_center > 20:
        # Green is centered
        left_speed = 0.96 * MAX_SPEED
        right_speed = 0.96 * MAX_SPEED
    
    # -----------------------------
    # STEER TOWARD GREEN - AIM FOR CENTER (Priority 4)
    # -----------------------------
    elif total_green > 10:
        green_bias = green_right - green_left
        red_bias = red_right - red_left
        
        steer = 0.0
        
        # Stronger turn to aim for center of green section
        TURN_STRENGTH = 0.011  # Increased from 0.009
        steer = TURN_STRENGTH * green_bias * MAX_SPEED
        
        # Bonus: if green is heavily on one side, turn harder to center it
        green_imbalance = abs(green_right - green_left)
        if green_imbalance > 30:
            steer *= 1.3  # Turn 30% harder when very off-center
        
        # Avoid red (stronger weight)
        if total_red > 10:
            AVOID_STRENGTH = 0.014
            steer -= AVOID_STRENGTH * red_bias * MAX_SPEED
        
        # Apply steering
        left_speed = base_speed + steer
        right_speed = base_speed - steer
    
    # -----------------------------
    # AVOID RED WHEN NO GREEN (Priority 5)
    # -----------------------------
    elif total_red > 15:
        red_bias = red_right - red_left
        AVOID_STRENGTH = 0.016
        steer = -AVOID_STRENGTH * red_bias * MAX_SPEED
        
        left_speed = base_speed + steer
        right_speed = base_speed - steer
    
    # -----------------------------
    # EMERGENCY SIDE WALL AVOIDANCE
    # -----------------------------

    if left_side > 200 and not scan_mode and forward_after_wall_counter == 0 and total_red == 0:
        # Too close to left wall - correct right
        left_speed = 0.95 * MAX_SPEED
        right_speed = 0.50 * MAX_SPEED
        straighten_counter = 15
        print("Left wall avoidance enabled")
        
    elif right_side > 200 and not scan_mode and forward_after_wall_counter == 0 and total_red == 0:
        # Too close to right wall - correct left
        left_speed = 0.50 * MAX_SPEED
        right_speed = 0.95 * MAX_SPEED
        straighten_counter = 15
        print("Right wall avoidance enabled")

    
    # -----------------------------
    # APPLY SPEEDS
    # -----------------------------
    # Clamp speeds
    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Debug (uncomment to see what's happening)
    print(f"TG:{total_green:3} GL:{green_left:3} GC:{green_center:3} GR:{green_right:3} | FWD:{forward_after_wall_counter:2} | Scan:{scan_mode}")
