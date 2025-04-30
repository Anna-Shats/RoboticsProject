# Cheat Sheet for motor wheels
# (rightfront, rightback, leftfront, leftback)
# 2000 moves fast backwards
# -2000 moves fast forwards
# 0 stops

import curses
import time
from motor import Ordinary_Car
from infrared import Infrared

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
        
        # To track the last known sensor pattern that detected the line
        last_active_sensor = 2  # Default to center
        consecutive_empty = 0   # Count consecutive times no line was detected
        
        # Corner handling variables
        corner_handling_state = 0  # 0=none, 1=detected, 2=overshooting, 3=turning
        corner_turn_direction = 0  # 0=none, 1=left, 2=right
        corner_state_steps = 0     # Count steps in each corner handling state
        
        # Constants for corner handling
        OVERSHOOT_STEPS = 3       # How many forward movements to overshoot
        TURN_STEPS = 15            # How many turn movements to make
        
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
                
                # Corner handling sequence
                if corner_handling_state > 0:
                    # Already in a corner handling sequence
                    if corner_handling_state == 1:  # Just detected corner, start overshooting
                        if max_y > 3:
                            stdscr.addstr(3, 0, f"Action: Overshoot {corner_state_steps}/{OVERSHOOT_STEPS}" + " " * 10)
                        
                        # Move forward to overshoot the corner
                        pwm.set_motor_model(-1000, -1000, -1000, -1000)  # Forward at good speed
                        time.sleep(0.15)
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.05)
                        
                        corner_state_steps += 1
                        if corner_state_steps >= OVERSHOOT_STEPS:
                            # Move to turning state
                            corner_handling_state = 2
                            corner_state_steps = 0
                    
                    elif corner_handling_state == 2:  # Overshooting complete, now turn
                        # Execute turn based on saved direction
                        if corner_turn_direction == 1:  # Left turn
                            if max_y > 3:
                                stdscr.addstr(3, 0, f"Action: Sharp Left {corner_state_steps}/{TURN_STEPS}" + " " * 10)
                            # Sharper left turn
                            pwm.set_motor_model(-800, -800, 1200, 1200)
                        else:  # Right turn
                            if max_y > 3:
                                stdscr.addstr(3, 0, f"Action: Sharp Right {corner_state_steps}/{TURN_STEPS}" + " " * 10)
                            # Sharper right turn
                            pwm.set_motor_model(1200, 1200, -800, -800)
                        
                        time.sleep(0.15)  # Longer turn
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.05)
                        
                        corner_state_steps += 1
                        if corner_state_steps >= TURN_STEPS:
                            # Corner sequence completed
                            corner_handling_state = 0
                            corner_turn_direction = 0
                            corner_state_steps = 0
                            
                            if max_y > 3:
                                stdscr.addstr(3, 0, "Action: Corner Complete     " + " " * 10)
                
                else:
                    # Normal line following, including corner detection
                    
                    # Check for corner pattern: left and center sensors on
                    if left_infrared == 4 and center_infrared == 2 and not right_infrared == 1:
                        if max_y > 3:
                            stdscr.addstr(3, 0, "Action: LEFT CORNER DETECTED" + " " * 10)
                        # Start corner handling sequence
                        corner_handling_state = 1
                        corner_turn_direction = 1  # Left turn
                        corner_state_steps = 0
                        last_active_sensor = 1
                        consecutive_empty = 0
                    
                    # Check for corner pattern: center and right sensors on
                    elif center_infrared == 2 and right_infrared == 1 and not left_infrared == 4:
                        if max_y > 3:
                            stdscr.addstr(3, 0, "Action: RIGHT CORNER DETECTED" + " " * 10)
                        # Start corner handling sequence
                        corner_handling_state = 1
                        corner_turn_direction = 2  # Right turn
                        corner_state_steps = 0
                        last_active_sensor = 3
                        consecutive_empty = 0
                    
                    # Case: All three sensors on - could be a wide line or intersection
                    elif left_infrared == 4 and center_infrared == 2 and right_infrared == 1:
                        if max_y > 3:
                            stdscr.addstr(3, 0, "Action: All Sensors On     " + " " * 10)
                        # Go straight but slower
                        pwm.set_motor_model(-600, -600, -600, -600)  # Slow forward
                        time.sleep(0.2)
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.1)
                        consecutive_empty = 0
                    
                    # Normal line tracking
                    elif left_infrared == 4:  # Just left sensor on
                        if max_y > 3:
                            stdscr.addstr(3, 0, "Action: Turn Left          " + " " * 10)
                        pwm.set_motor_model(-900, -900, 1250, 1250)  # Turn left
                        time.sleep(0.15)
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.1)
                        last_active_sensor = 1
                        consecutive_empty = 0
                    
                    elif right_infrared == 1:  # Just right sensor on
                        if max_y > 3:
                            stdscr.addstr(3, 0, "Action: Turn Right         " + " " * 10)
                        pwm.set_motor_model(1250, 1250, -900, -900)  # Turn right
                        time.sleep(0.15)
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.1)
                        last_active_sensor = 3
                        consecutive_empty = 0
                    
                    elif center_infrared == 2:  # Just center sensor on
                        if max_y > 3:
                            stdscr.addstr(3, 0, "Action: Forward            " + " " * 10)
                        pwm.set_motor_model(-800, -800, -800, -800)  # Move forward
                        time.sleep(0.2)
                        pwm.set_motor_model(0, 0, 0, 0)  # Stop
                        time.sleep(0.1)
                        last_active_sensor = 2
                        consecutive_empty = 0
                    
                    else:  # No sensors - recovery behavior
                        consecutive_empty += 1
                        
                        # For the first few misses, keep going forward
                        if consecutive_empty <= 3:
                            if max_y > 3:
                                stdscr.addstr(3, 0, "Action: Continue Forward " + " " * 10)
                            pwm.set_motor_model(-600, -600, -600, -600)  # Slower forward
                            time.sleep(0.1)
                            pwm.set_motor_model(0, 0, 0, 0)  # Stop
                            time.sleep(0.05)
                        
                        # After several misses, try to recover based on last sensor
                        else:
                            if last_active_sensor == 1:  # Left was last active
                                if max_y > 3:
                                    stdscr.addstr(3, 0, "Action: Seek Left       " + " " * 10)
                                pwm.set_motor_model(-600, -600, -1000, -1000)  # Forward with left bias
                            elif last_active_sensor == 3:  # Right was last active
                                if max_y > 3:
                                    stdscr.addstr(3, 0, "Action: Seek Right      " + " " * 10)
                                pwm.set_motor_model(-1000, -1000, -600, -600)  # Forward with right bias
                            else:  # Center was last active
                                if max_y > 3:
                                    stdscr.addstr(3, 0, "Action: Small Slither   " + " " * 10)
                                # Alternate slight left and right movement
                                if consecutive_empty % 2 == 0:
                                    pwm.set_motor_model(-600, -600, -900, -900)  # Slight left bias
                                else:
                                    pwm.set_motor_model(-900, -900, -600, -600)  # Slight right bias
                            
                            time.sleep(0.1)  # Move for a short time
                            pwm.set_motor_model(0, 0, 0, 0)  # Stop
                            time.sleep(0.05)  # Short pause
                
                # Robot visualization - only if screen is big enough
                if max_y > 6:
                    robot_vis = "    ↑    "  # Default forward position
                    
                    if corner_handling_state == 1:
                        robot_vis = "    ⇑    "  # Overshooting (bold forward)
                    elif corner_handling_state == 2:
                        if corner_turn_direction == 1:
                            robot_vis = "  ↰      "  # Left turn after overshoot
                        else:
                            robot_vis = "      ↱  "  # Right turn after overshoot
                    elif last_active_sensor == 1:
                        robot_vis = "  ↖      "  # Left arrow
                    elif last_active_sensor == 3:
                        robot_vis = "      ↗  "  # Right arrow
                    
                    if max_y > 4:
                        stdscr.addstr(4, 0, "Robot position:" + " " * 10)
                    if max_y > 5:
                        stdscr.addstr(5, 0, f"  {robot_vis}  " + " " * 5)
                    if max_y > 6:
                        stdscr.addstr(6, 0, f"  {sensor_visual}  " + " " * 5)
                
                # Debug info only if screen is big enough
                if max_y > 7:
                    corner_state = "NONE"
                    if corner_handling_state == 1:
                        corner_state = f"OVERSHOOT-{corner_state_steps}/{OVERSHOOT_STEPS}"
                    elif corner_handling_state == 2:
                        turn_dir = "LEFT" if corner_turn_direction == 1 else "RIGHT"
                        corner_state = f"TURN {turn_dir}-{corner_state_steps}/{TURN_STEPS}"
                    
                    stdscr.addstr(7, 0, f"Corner: {corner_state} | Last: {last_active_sensor} | Empty: {consecutive_empty}" + " " * 5)
                
                stdscr.refresh()
            
            # Short sleep to prevent CPU overload
            time.sleep(0.01)
            
    except Exception as e:
        # Safe error handling
        try:
            rows, cols = stdscr.getmaxyx()
            if rows > 8:
                stdscr.addstr(8, 0, "Error occurred" + " " * 10)
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

def display_menu(stdscr):
    # Set up curses for menu display
    curses.curs_set(1)  # Show cursor
    stdscr.nodelay(False)  # Blocking input for menu
    stdscr.clear()
    
    # Menu options
    menu_items = [
        "1. Manual Control (WASD)",
        "2. Line Tracking",
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
