#!/usr/bin/env python
"""
This simple example is used for the line-by-line tutorial
that comes with pygame. It is based on a 'popular' web banner.
Note there are comments here, but for the full explanation,
follow along in the tutorial.
"""

import my_navigation_code as nav
import sprite_routines as sp
# Import Modules
from definitions import *
import matplotlib.pyplot as plt


def create_world(background):

    nao1 = sp.nao_robot(background)
    nao1.set_pos(5, 5, 30*degree)
    target1 = sp.target(50, 20)
    wall1 = sp.obstacle(10.5, 20, 1, 21, 0)
    wall2 = sp.obstacle(5, 10, 10, 1, 0)
    wall3 = sp.obstacle(30, 10, 15, 15)
    wall4 = sp.obstacle(30, 30, 15, 2)
    person1=sp.person(50,10,30.0*degree)

    polygon1 = sp.polygon([[0, 0], [4, 0], [6, 6], [2, 8], [-2, 6]], (250, 250, 0))  # cm
    polygon1.set_pos(20, 20, 0)  # cm
    
    alltargets = sp.pygame.sprite.Group((target1,))
    allobstacles = sp.pygame.sprite.Group((wall1, wall2, wall3, wall4,))
    # alltargets = sp.pygame.sprite.Group((target1,person1))
    # allobstacles = sp.pygame.sprite.Group((wall1, wall2, wall3, wall4,polygon1))
    allrobots = sp.pygame.sprite.Group((nao1,))

    whiff_sound = sp.load_sound('whiff.wav')
    punch_sound = sp.load_sound('punch.wav')

    allsounds = {'whiff': whiff_sound, 'punch': punch_sound}

    return allrobots, allobstacles, alltargets, allsounds

def detect_collisions(robot, allobstacles, allsounds, sound_played):
    # detect collisions
    obstacle_hit = sp.pygame.sprite.spritecollide(robot, allobstacles, False, sp.pygame.sprite.collide_mask)
    if obstacle_hit:
        for g in obstacle_hit:
            if isinstance(g, sp.obstacle):
                robot.set_vel(0, 0)
                if not sound_played:
                    print("You hit the wall")
                    allsounds['punch'].play()
                    sound_played = True
                break
            elif isinstance(g, sp.polygon):
                robot.set_vel(0, 1)
                if not sound_played:
                    print("You stepped on an area")
                    allsounds['whiff'].play()
                    sound_played = True
                break
    else:
        sound_played = False
    return sound_played


def detect_target_reached(nao1, alltargets, allsounds):
#    global allsounds

    target_hit = sp.pygame.sprite.spritecollide(nao1, alltargets, False, sp.pygame.sprite.collide_mask)
    if target_hit:
        print("you reached the target! Hurray!")
        nao1.set_vel(0, 0)
        allsounds['whiff'].play()

    return target_hit


def show_menu():

    print("Menu:")
    print("ESC   exit game")
    print("UP    speed up robot")
    print("DOWN  slow down robot")
    print("LEFT  increase turnrate")
    print("RIGHT decrease turnrate")
    print("SPACE stop robot")
    print("a     toggle autonomous navigation")
    print("c     toggle collision detection")
    print("d     toggle draw sonar detections (in autonomous mode)")

def mainloop():
    """this function is called when the program starts.
       it initializes everything it needs, then runs in
       a loop until the function returns."""
    screen, background, clock = sp.init()
    allrobots, allobstacles, alltargets, allsounds  = create_world(background)

    # Main Loop
    going = True
    collided = False
    target_reached = False
    autonomous = False
    collision_detection = True

    nao1 = allrobots.sprites()[0]  # there is only one robot currently
    target = alltargets.sprites()[0] # and one target

    while going:
        clock.tick(60)
        # Handle Input Events
        for event in sp.pygame.event.get():
            if event.type == sp.QUIT:
                going = False
            elif event.type == sp.KEYDOWN:
                if event.key == sp.K_ESCAPE:
                    going = False
                elif event.key == sp.K_UP:
                    nao1.velocity += 1.0
                elif event.key == sp.K_DOWN:
                    nao1.velocity -= 1.0
                elif event.key == sp.K_LEFT:
                    nao1.turnrate += 0.1
                elif event.key == sp.K_RIGHT:
                    nao1.turnrate -= 0.1
                elif event.key == sp.K_SPACE:
                    if autonomous:
                        print("Autonomous mode turned off")
                        autonomous = False
                    nao1.set_vel(0, 0)
                elif event.key == sp.K_a:
                    autonomous = not autonomous
                    if autonomous:
                        print("Autonomous mode active")
                    else:
                        print("Autonomous mode turned off")
                elif event.key == sp.K_d:
                    nao1.draw_detections = not nao1.draw_detections
                    if nao1.draw_detections:
                        print("Drawing detections is active")
                    else:
                        print("Drawing detections is turned off")
                elif event.key == sp.K_c:
                    collision_detection = not collision_detection
                    if collision_detection:
                        print("Collision detection is active")
                    else:
                        print("Collision detection is turned off")
                else:
                    show_menu()
                    pass
            elif event.type == sp.MOUSEBUTTONDOWN:
                pos = sp.pygame.mouse.get_pos()
                xx, yy = sp.from_screen_coordinates(pos)
                target.set_pos(xx, yy)
                # thetarget.update_pos(pos[1],pos[0],-12,False)
            elif event.type == sp.MOUSEBUTTONUP:
                pass

        # this updates the parameters and the state of the robot (collided or not, going or not
        if autonomous:
            nav.scan_world(nao1, allobstacles, alltargets)
        if collision_detection:
            collided = detect_collisions(nao1, allobstacles, allsounds, collided)
        if not target_reached:
            target_reached = detect_target_reached(nao1, alltargets, allsounds)
        else:
            autonomous = False
            nao1.set_vel(0, 0)

        # this updates the position based on current velocity and turnrate

        # Draw Everything
        screen.blit(background, (0, 0))
        allobstacles.draw(screen)
        alltargets.draw(screen)
        allrobots.draw(screen)
        sp.pygame.display.flip()

        allrobots.update()
        allobstacles.update()
        alltargets.update()

    sp.pygame.quit()


# Game Over
def show_data():
    t = [x[0] for x in data_robot]
    y1 = [x[1] for x in data_robot]
    y2 = [x[2] for x in data_robot]
    #print(t)
    plt.plot(t, y1, 'r-', t, y2, 'b-')
    plt.xlabel('time (s)')
    plt.ylabel('sonar data (m)')
    plt.legend(['left', 'right'])
    plt.show()



# this calls the 'mainloop' function when this script is executed
if __name__ == '__main__':
    show_menu()
    mainloop()
    #show_data()
    