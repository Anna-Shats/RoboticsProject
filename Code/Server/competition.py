# Cheat Sheet for motor wheels
# (rightfront, rightback, leftfront, leftback)
# 2000 moves fast backwards
# -2000 moves fast forwards
# 0 stops

import curses
import time
from motor import Ordinary_Car
from infrared import Infrared

def a_star(stdscr, pwm):
    # Set up curses for line tracking display
    stdscr.nodelay(True)  # Non-blocking input
    curses.curs_set(0)    # Hide cursor
    stdscr.clear()
    
    

def line_tracking(stdscr, pwm):
    # Set up curses for line tracking display
    stdscr.nodelay(True)  # Non-blocking input
    curses.curs_set(0)    # Hide cursor
    stdscr.clear()
    
    # Get terminal dimensions
    max_y, max_x = stdscr.getmaxyx()
    
    # Only add text if we have enough room
    if max_y > 0:
        stdscr.addstr(0, 0, "Line Tracking Mode (Q to return to menu)")
    stdscr.refresh()
    
    try:
        # Initialize infrared sensors
        ir = Infrared()
        
        # For tracking time between episodes
        car_record_time = time.time()
        
        while True:
            # Check for quit key
            key = stdscr.getch()
            if key == ord('q'):
                return  # Return to main menu
            
            # Only read sensors and adjust movement every 0.2 seconds
            if (time.time() - car_record_time) > 0.2:
                car_record_time = time.time()
                
                # Read all infrared sensors
                infrared_value = ir.read_all_infrared()
                
                # Read individual sensors and shift bits
                left_infrared = ir.read_one_infrared(1) << 2    # Left (value 4 when active)
                center_infrared = ir.read_one_infrared(2) << 1  # Center (value 2 when active)
                right_infrared = ir.read_one_infrared(3)        # Right (value 1 when active)
                
                # Visual representation of sensors
                left_symbol = "0" if left_infrared == 4 else "X"
                middle_symbol = "0" if center_infrared == 2 else "X"
                right_symbol = "0" if right_infrared == 1 else "X"
                sensor_visual = f"{left_symbol} | {middle_symbol} | {right_symbol}"
                
                # Display sensor information
                if max_y > 1:
                    stdscr.addstr(1, 0, f"IR Sensors: {sensor_visual}  (0=Line, X=No Line)" + " " * 5)
                if max_y > 2:
                    stdscr.addstr(2, 0, f"IR Value: {infrared_value} | L:{left_infrared} C:{center_infrared} R:{right_infrared}" + " " * 5)
                
                # Simple line following logic
                if left_infrared == 4:
                    # Turn left in episodes
                    if max_y > 3:
                        stdscr.addstr(3, 0, "Action: Turn Left          " + " " * 10)
                    pwm.set_motor_model(-1250, -1250, 1250, 1250)  # Turn left
                    time.sleep(0.15)  # Turn for a short time
                    pwm.set_motor_model(0, 0, 0, 0)  # Stop
                    time.sleep(0.1)  # Pause to check sensors
                
                elif right_infrared == 1:
                    # Turn right in episodes
                    if max_y > 3:
                        stdscr.addstr(3, 0, "Action: Turn Right         " + " " * 10)
                    pwm.set_motor_model(1250, 1250, -1250, -1250)  # Turn right
                    time.sleep(0.15)  # Turn for a short time
                    pwm.set_motor_model(0, 0, 0, 0)  # Stop
                    time.sleep(0.1)  # Pause to check sensors
                
                elif center_infrared == 2:
                    # Move forward in episodes with slight left-right slithering
                    if max_y > 3:
                        stdscr.addstr(3, 0, "Action: Forward (Slithering)" + " " * 10)
                    
                    # Alternate between slightly left and slightly right
                    # Use a timestamp-based alternation
                    if int(time.time() * 5) % 2 == 0:  # Changes every 0.2 seconds
                        # Slight left bias while moving forward
                        pwm.set_motor_model(-700, -700, -900, -900)
                    else:
                        # Slight right bias while moving forward
                        pwm.set_motor_model(-900, -900, -700, -700)
                    
                    time.sleep(0.15)
                    pwm.set_motor_model(0, 0, 0, 0)  # Stop
                    time.sleep(0.05)  # Shorter pause for smoother motion
                
                else:
                    # Line lost - track consecutive misses
                    if not hasattr(line_tracking, 'consecutive_misses'):
                        line_tracking.consecutive_misses = 0
                    line_tracking.consecutive_misses += 1
                    
                    if line_tracking.consecutive_misses <= 10:
                        # First few misses - move backward briefly
                        if max_y > 3:
                            stdscr.addstr(3, 0, "Action: Line Lost, Backing " + " " * 10)
                        pwm.set_motor_model(800, 800, 800, 800)  # Move backward (inverted values)
                        time.sleep(0.1)  # Shorter backward movement
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.05)
                    else:
                        # After several misses, try a sweeping search
                        sweep_direction = line_tracking.consecutive_misses % 2
                        if sweep_direction == 0:
                            # Try turning left to find line
                            if max_y > 3:
                                stdscr.addstr(3, 0, "Action: Searching Left    " + " " * 10)
                            pwm.set_motor_model(-800, -800, 1100, 1100)
                        else:
                            # Try turning right to find line
                            if max_y > 3:
                                stdscr.addstr(3, 0, "Action: Searching Right   " + " " * 10)
                            pwm.set_motor_model(1100, 1100, -800, -800)
                        
                        time.sleep(0.12)
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.05)
                        
                        # Reset counter after many attempts
                        if line_tracking.consecutive_misses > 12:
                            line_tracking.consecutive_misses = 0
                
                # Reset consecutive misses when we detect the line
                if left_infrared == 4 or center_infrared == 2 or right_infrared == 1:
                    if hasattr(line_tracking, 'consecutive_misses'):
                        line_tracking.consecutive_misses = 0
                
                # Robot visualization - only if screen is big enough
                if max_y > 6:
                    robot_vis = "    ↑    "  # Default forward position
                    
                    if left_infrared == 4:
                        robot_vis = "  ↖      "  # Left arrow
                    elif right_infrared == 1:
                        robot_vis = "      ↗  "  # Right arrow
                    elif center_infrared != 2 and left_infrared != 4 and right_infrared != 1:
                        robot_vis = "    ↓    "  # Down arrow (backing up)
                    
                    if max_y > 4:
                        stdscr.addstr(4, 0, "Robot position:" + " " * 10)
                    if max_y > 5:
                        stdscr.addstr(5, 0, f"  {robot_vis}  " + " " * 5)
                    if max_y > 6:
                        stdscr.addstr(6, 0, f"  {sensor_visual}  " + " " * 5)
                
                stdscr.refresh()
            
            # Short sleep to prevent CPU overload
            time.sleep(0.01)
            
    except Exception as e:
        # Safe error handling
        try:
            rows, cols = stdscr.getmaxyx()
            if rows > 8:
                stdscr.addstr(8, 0, f"Error occurred: {str(e)}" + " " * 10)
            stdscr.refresh()
            time.sleep(2)
        except:
            pass
    finally:
        # Clean up
        pwm.set_motor_model(0, 0, 0, 0)
        ir.close()

def manual_control(stdscr, pwm):
    # Set up curses for manual control
    stdscr.nodelay(True)  # Non-blocking input
    curses.curs_set(0)    # Hide cursor
    stdscr.clear()
    stdscr.addstr(0, 0, "Manual Control Mode - WASD keys to move (Q to return to menu)")
    stdscr.refresh()
    
    # Movement tracking
    last_key_time = 0
    current_key = None
    
    # Initialize infrared sensors
    ir = Infrared()
    
    # Initialize servo
    from servo import Servo
    servo = Servo()
    
    # Servo scanning variables
    servo_angles = [0, 30, 60, 90, 120, 150, 180]
    current_angle_index = 0
    last_servo_move_time = time.time()
    servo_interval = 0.8  # seconds between servo movements
    
    try:
        while True:
            key = stdscr.getch()
            current_time = time.time()
            
            # Read IR sensors
            left_infrared = ir.read_one_infrared(1)
            center_infrared = ir.read_one_infrared(2)
            right_infrared = ir.read_one_infrared(3)
            
            # Visual representation of sensors
            left_symbol = "0" if left_infrared else "X"
            middle_symbol = "0" if center_infrared else "X"
            right_symbol = "0" if right_infrared else "X"
            sensor_visual = f"{left_symbol} | {middle_symbol} | {right_symbol}"
            
            # Display IR sensor info
            stdscr.addstr(3, 0, f"IR Sensors: {sensor_visual}  (0=Line, X=No Line)" + " " * 5)
            stdscr.addstr(4, 0, f"L:{left_infrared} C:{center_infrared} R:{right_infrared}" + " " * 5)
            
            # Move servo at regular intervals
            if current_time - last_servo_move_time >= servo_interval:
                current_angle = servo_angles[current_angle_index]
                servo.set_servo_pwm('1', current_angle)
                stdscr.addstr(5, 0, f"Servo angle: {current_angle}°" + " " * 10)
                
                # Move to next angle
                current_angle_index = (current_angle_index + 1) % len(servo_angles)
                last_servo_move_time = current_time
            
            # Always register new key presses
            if key != -1:
                current_key = key
                last_key_time = current_time
                
                if key == ord('q'):
                    return  # Return to main menu
                    
                # Process movement commands
                if key == ord('w'):  # Forward
                    stdscr.addstr(1, 0, "Moving forward  ")
                    pwm.set_motor_model(-2000, -2000, -2000, -2000)
                elif key == ord('s'):  # Backward
                    stdscr.addstr(1, 0, "Moving backward ")
                    pwm.set_motor_model(1500, 1500, 1500, 1500)
                elif key == ord('a'):  # Left
                    stdscr.addstr(1, 0, "Turning left    ")
                    pwm.set_motor_model(-2000, -2000, 2000, 2000)
                elif key == ord('d'):  # Right
                    stdscr.addstr(1, 0, "Turning right   ")
                    pwm.set_motor_model(2000, 2000, -2000, -2000)
                elif key == ord(' '):  # Space to explicitly stop
                    stdscr.addstr(1, 0, "Stopped         ")
                    pwm.set_motor_model(0, 0, 0, 0)
                    current_key = None
            
            # Auto-stop if no key press for 0.1 seconds
            if current_key is not None and current_time - last_key_time > 0.1:
                stdscr.addstr(1, 0, "Stopped (auto)   ")
                pwm.set_motor_model(0, 0, 0, 0)
                current_key = None
            
            # Debug info
            stdscr.addstr(2, 0, f"Key: {chr(current_key) if current_key else 'None'} | Time since: {current_time - last_key_time:.3f}s")
            stdscr.refresh()
            
            # Very small delay for CPU efficiency but maintain responsiveness
            time.sleep(0.01)
    
    finally:
        # Make sure motors are stopped when exiting manual mode
        pwm.set_motor_model(0, 0, 0, 0)
        ir.close()
        # Reset servo position to center before exiting
        servo.set_servo_pwm('1', 90)

def maze_solving(stdscr, pwm):
    # Set up curses
    stdscr.nodelay(True)
    curses.curs_set(0)
    stdscr.clear()
    
    # Import additional modules
    from ultrasonic import Ultrasonic
    from buzzer import Buzzer
    
    # Define the sequence of turns (left=L, right=R)
    sequence = ["L", "R", "R", "L", "L", "L", "R", "L", "L", "R", "R", "R", "L", "L", "R", "R", "R", "L", "R", "L", "L", "R"]
    current_step = 0
    
    # Setup ultrasonic sensor and buzzer
    ultrasonic = Ultrasonic()
    buzzer = Buzzer()
    buzzer_enabled = True  # Can be toggled
    
    try:
        running = True
        forward_speed = -1000  # Forward speed (negative for forward)
        turning_speed = 1500   # Speed for turning
        turn_time = 0.65       # Time for ~90 degree turn
        wall_threshold = 10.0  # Increased threshold to stop earlier
        
        # Distance history for display
        distance_history = [0] * 5  # Keep last 5 readings
        
        max_y, max_x = stdscr.getmaxyx()
        
        # Initial display
        if max_y > 0:
            stdscr.addstr(0, 0, "Maze Solving Mode (Q to return to menu, B to toggle buzzer)")
        if max_y > 1:
            stdscr.addstr(1, 0, "Starting maze sequence")
        if max_y > 2:
            stdscr.addstr(2, 0, f"Next turn: {sequence[current_step]}")
        stdscr.refresh()
        
        # For gradual slowing
        approach_threshold = 25.0  # Start slowing down at this distance
        
        # Main loop
        while running:
            # Check for quit key
            key = stdscr.getch()
            if key == ord('q'):
                running = False
                break
            elif key == ord('b'):
                # Toggle buzzer
                buzzer_enabled = not buzzer_enabled
                if max_y > 4:
                    stdscr.addstr(4, 0, f"Buzzer: {'ON ' if buzzer_enabled else 'OFF'}" + " "*10)
                    stdscr.refresh()
            
            # Get distance - take multiple readings for better accuracy
            distances = []
            for _ in range(3):  # Take 3 readings
                dist = ultrasonic.get_distance()
                if dist is not None:
                    distances.append(dist)
                time.sleep(0.01)  # Very short delay between readings
            
            # Use median of readings if available
            if distances:
                distances.sort()
                distance = distances[len(distances)//2]  # Median value
                
                # Update history
                distance_history.pop(0)
                distance_history.append(distance)
            else:
                distance = None
            
            # Display current status and distance history
            if distance is not None and max_y > 3:
                stdscr.addstr(3, 0, f"Distance: {distance:.1f}cm, Step: {current_step+1}/{len(sequence)}" + " "*10)
                
                # Display distance history if there's room
                if max_y > 7:
                    stdscr.addstr(7, 0, "Recent distances: " + " "*20)
                    hist_str = " ".join([f"{d:.1f}" for d in distance_history])
                    stdscr.addstr(8, 0, hist_str + " "*20)
                
                stdscr.refresh()
            
            # Detect wall and perform turn
            if distance is not None:
                # Gradual speed adjustment based on distance
                if distance <= approach_threshold and distance > wall_threshold:
                    # Calculate a proportional speed reduction as we approach the wall
                    speed_factor = (distance - wall_threshold) / (approach_threshold - wall_threshold)
                    speed_factor = max(0.4, min(1.0, speed_factor))  # Clamp between 0.4 and 1.0
                    adjusted_speed = int(forward_speed * speed_factor)
                    
                    if max_y > 5:
                        stdscr.addstr(5, 0, f"Slowing: {adjusted_speed} ({speed_factor:.2f})" + " "*15)
                    
                    pwm.set_motor_model(adjusted_speed, adjusted_speed, adjusted_speed, adjusted_speed)
                
                # Stop and turn when we reach the threshold
                if distance <= wall_threshold:
                    # Sound buzzer if enabled
                    if buzzer_enabled:
                        buzzer.set_state(True)
                        time.sleep(0.1)
                        buzzer.set_state(False)
                    
                    # Emergency stop
                    pwm.set_motor_model(0, 0, 0, 0)
                    if max_y > 6:
                        stdscr.addstr(6, 0, f"Wall detected at {distance:.1f}cm!" + " "*15)
                        stdscr.refresh()
                    time.sleep(0.3)
                    
                    # Turn based on the sequence
                    if current_step < len(sequence):
                        turn_direction = sequence[current_step]
                        
                        if max_y > 5:
                            stdscr.addstr(5, 0, f"Turning {turn_direction}" + " "*15)
                            stdscr.refresh()
                        
                        if turn_direction == "L":
                            # Turn left (90 degrees)
                            pwm.set_motor_model(-turning_speed, -turning_speed, turning_speed, turning_speed)
                        else:  # "R"
                            # Turn right (90 degrees)
                            pwm.set_motor_model(turning_speed, turning_speed, -turning_speed, -turning_speed)
                        
                        time.sleep(turn_time)
                        
                        # Stop after turning
                        pwm.set_motor_model(0, 0, 0, 0)
                        time.sleep(0.3)
                        
                        # Move to next step in sequence
                        current_step += 1
                        if current_step < len(sequence):
                            if max_y > 2:
                                stdscr.addstr(2, 0, f"Next turn: {sequence[current_step]}" + " "*10)
                                stdscr.refresh()
                        else:
                            if max_y > 2:
                                stdscr.addstr(2, 0, "Sequence complete!" + " "*10)
                                stdscr.refresh()
                                time.sleep(2)
                                running = False
                                break
                    else:
                        # Sequence completed
                        if max_y > 2:
                            stdscr.addstr(2, 0, "Sequence complete!" + " "*10)
                            stdscr.refresh()
                        running = False
                        break
                elif distance > wall_threshold and distance <= approach_threshold:
                    # We're in the approach zone but not at the wall yet
                    continue  # Skip the normal forward movement below
                else:
                    # Move forward at full speed
                    pwm.set_motor_model(forward_speed, forward_speed, forward_speed, forward_speed)
            else:
                # No valid distance reading, move forward cautiously
                pwm.set_motor_model(forward_speed//2, forward_speed//2, forward_speed//2, forward_speed//2)
            
            # Short delay between iterations
            time.sleep(0.02)  # Faster loop for more responsive stopping
            
        # Final message
        stdscr.clear()
        if max_y > 0:
            stdscr.addstr(0, 0, "Maze solving completed or stopped")
        if max_y > 1:
            stdscr.addstr(1, 0, "Press any key to return to menu")
        stdscr.refresh()
        
        # Wait for key press
        stdscr.nodelay(False)
        stdscr.getch()
        
    except Exception as e:
        # Safe error handling
        try:
            if max_y > 9:
                stdscr.addstr(9, 0, f"Error: {str(e)}" + " "*20)
            stdscr.refresh()
            time.sleep(2)
        except:
            pass
    finally:
        # Clean up
        pwm.set_motor_model(0, 0, 0, 0)
        ultrasonic.close()
        buzzer.close()

def display_menu(stdscr):
    # Set up curses for menu display
    curses.curs_set(1)  # Show cursor
    stdscr.nodelay(False)  # Blocking input for menu
    stdscr.clear()
    
    # Menu options
    menu_items = [
        "1. Manual Control (WASD)",
        "2. Line Tracking",
        "3. Maze Solving",
        "q. Quit"
    ]
    
    # Display menu
    stdscr.addstr(0, 0, "Robot Control Menu")
    stdscr.addstr(1, 0, "-----------------")
    for i, item in enumerate(menu_items):
        stdscr.addstr(i+3, 0, item)
    
    stdscr.addstr(len(menu_items)+4, 0, "Enter your choice: ")
    stdscr.refresh()

def main(stdscr):
    # Initialize the car
    PWM = Ordinary_Car()
    
    try:
        while True:
            display_menu(stdscr)
            # Get user choice
            key = stdscr.getch()
            
            if key == ord('1') or key == ord('m'):
                manual_control(stdscr, PWM)
            elif key == ord('2') or key == ord('l'):
                line_tracking(stdscr, PWM)
            elif key == ord('3'):
                maze_solving(stdscr, PWM)
            elif key == ord('q'):
                break
            else:
                stdscr.clear()
                stdscr.addstr(0, 0, "Invalid option. Press any key to continue.")
                stdscr.refresh()
                stdscr.getch()
    
    except KeyboardInterrupt:
        pass
    finally:
        PWM.set_motor_model(0, 0, 0, 0)  # Ensure motors stop
        PWM.close()

if __name__ == '__main__':
    print('Starting Robot Control Program...')
    curses.wrapper(main)
