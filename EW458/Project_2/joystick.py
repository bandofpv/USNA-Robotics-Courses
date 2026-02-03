import pygame

pygame.init()

def main():
    joystick = None

    exit = False
    while not exit:
        # Event processing step.
        for event in pygame.event.get():

            if event.type == pygame.JOYBUTTONDOWN:
                # IF "A" button pressed
                if event.button == 0:
                    print("A button pressed")
                # If "B" button pressed
                if event.button == 1:
                    print("B button pressed")
                # If "X" button pressed
                if event.button == 2:
                    print("X button pressed")
                # If "Y" button pressed
                if event.button == 3:
                    print("Y button pressed")
                if event.button == 11:
                    print("START button pressed, exiting")
                    exit = True

            if event.type == pygame.JOYBUTTONUP:
                # If "A" button released
                if event.button == 0:
                    print("A button released")
                # If "B" button released
                if event.button == 1:
                    print("B button released")  
                # If "X" button released
                if event.button == 2:
                    print("X button released")
                # If "Y" button released
                if event.button == 3:
                    print("Y button released")
            
            if event.type == pygame.JOYAXISMOTION:
                axis = event.axis
                value = event.value
                print(f"Axis {axis} moved to {value:>6.3f}")

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                if joystick is None:
                    joystick = pygame.joystick.Joystick(event.device_index)
                    print(f"Joystick {joystick.get_name()} connected")

            if event.type == pygame.JOYDEVICEREMOVED:
                if joystick and joystick.get_instance_id() == event.instance_id:
                    print(f"Joystick {joystick.get_name()} disconnected")
                    joystick = None

        # Process joystick input
        if joystick:
            axes = joystick.get_numaxes()
            for i in range(axes):
                axis = joystick.get_axis(i)
                # print(f"Axis {i} value: {axis:>6.3f}")

            buttons = joystick.get_numbuttons()
            for i in range(buttons):
                button = joystick.get_button(i)
                print(f"Button {i:>2} value: {button}")

if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()