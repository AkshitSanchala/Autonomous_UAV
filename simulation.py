import pygame
import random
from settings import *
from drone import Drone
from pygame.locals import *

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    leader_election_triggered = False
    pygame.display.set_caption("Drone Swarm Simulation with Leader Election")
    clock = pygame.time.Clock()

    swarm = [Drone(random.randint(0, SCREEN_WIDTH), random.randint(0, SCREEN_HEIGHT)) for _ in range(NUM_DRONES)]

    target_pos = pygame.math.Vector2(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if LEADER_FOLLOWS_MOUSE and event.type == pygame.MOUSEMOTION:
                target_pos.x, target_pos.y = event.pos
            if event.type == KEYDOWN:
                if event.key == K_SPACE:
                    leader_election_triggered = True
        
        screen.fill(BACKGROUND_COLOR)

        for drone in swarm:
            drone.update(swarm, target_pos, leader_election_triggered)
            drone.draw(screen)


        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
