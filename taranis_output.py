import pygame
pygame.init() # Initializes pygame
#clock = pygame.time.Clock() #Ved ikke hvor nødvendigt clock er

print(f" Found {pygame.joystick.get_count()} joysticks") # We should only get one joystick
joystick = pygame.joystick.Joystick(0) # choose joystick 0
joystick.init() # Initializes joystick

while True:
    #clock.tick(60) #fuck da clock
    pygame.event.get()   

    ax0 = joystick.get_axis(0) #Thrust
    ax3 = joystick.get_axis(3) #Yaw
    ax2 = joystick.get_axis(2) #Pitch
    ax1 = joystick.get_axis(1) #Roll
    print(f"Axis 0: {ax0}, Axis 3: {ax3}, Axis 2: {ax2}, Axis 1: {ax1}")
    

    but0 = joystick.get_axis(4) #SG 3-way switch
    but1 = joystick.get_axis(5) #SC 3-way switch
    but2 = joystick.get_axis(6) #Right analog button
    but3 = joystick.get_axis(7) #SF 2-way switch
    print(f"Button 0: {but0}, Button 1: {but1}, Button 2: {but2}, Button 3: {but3}\n")
    

