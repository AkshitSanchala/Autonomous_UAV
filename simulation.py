import pygame
import random
import settings
from drone import Drone
from pygame.locals import *

def main():
    pygame.init()
    screen = pygame.display.set_mode((settings.SCREEN_WIDTH, settings.SCREEN_HEIGHT))
    leader_election_triggered = False
    pygame.display.set_caption("Drone Swarm Simulation with Leader Election")
    clock = pygame.time.Clock()

    swarm = [Drone(random.randint(0, settings.SCREEN_WIDTH), random.randint(0, settings.SCREEN_HEIGHT)) for _ in range(settings.NUM_DRONES)]

    target_pos = pygame.math.Vector2(settings.SCREEN_WIDTH / 2, settings.SCREEN_HEIGHT / 2)

    # --- Feedback Message ---
    font = pygame.font.Font(None, 30)
    feedback_message = ""
    feedback_timer = 0

    last_key = None

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if settings.LEADER_FOLLOWS_MOUSE and event.type == pygame.MOUSEMOTION:
                target_pos.x, target_pos.y = event.pos
            if event.type == KEYDOWN:
                if event.key == K_SPACE:
                    leader_election_triggered = True
                elif event.key == K_c:
                    last_key = 'c'
                elif event.key == K_a:
                    last_key = 'a'
                elif event.key == K_s:
                    last_key = 's'
                elif event.key == K_l:
                    settings.LEADER_FOLLOWS_MOUSE = not settings.LEADER_FOLLOWS_MOUSE
                    feedback_message = f"Leader follows mouse: {settings.LEADER_FOLLOWS_MOUSE}"
                    feedback_timer = 120
                elif event.key == K_PLUS or event.key == K_KP_PLUS or event.key == K_EQUALS:
                    if last_key == 'c':
                        settings.COHESION_WEIGHT += 0.1
                        feedback_message = f"Cohesion weight: {settings.COHESION_WEIGHT:.2f}"
                    elif last_key == 'a':
                        settings.ALIGNMENT_WEIGHT += 0.1
                        feedback_message = f"Alignment weight: {settings.ALIGNMENT_WEIGHT:.2f}"
                    elif last_key == 's':
                        settings.SEPARATION_WEIGHT += 0.01
                        feedback_message = f"Separation weight: {settings.SEPARATION_WEIGHT:.2f}"
                    feedback_timer = 120
                elif event.key == K_MINUS or event.key == K_KP_MINUS:
                    if last_key == 'c':
                        settings.COHESION_WEIGHT -= 0.1
                        feedback_message = f"Cohesion weight: {settings.COHESION_WEIGHT:.2f}"
                    elif last_key == 'a':
                        settings.ALIGNMENT_WEIGHT -= 0.1
                        feedback_message = f"Alignment weight: {settings.ALIGNMENT_WEIGHT:.2f}"
                    elif last_key == 's':
                        settings.SEPARATION_WEIGHT -= 0.01
                        feedback_message = f"Separation weight: {settings.SEPARATION_WEIGHT:.2f}"
                    feedback_timer = 120

        screen.fill(settings.BACKGROUND_COLOR)

        for drone in swarm:
            drone.update(swarm, target_pos, leader_election_triggered)
            drone.draw(screen)

        # --- Display Feedback ---
        if feedback_timer > 0:
            text = font.render(feedback_message, True, (255, 255, 255))
            screen.blit(text, (10, 10))
            feedback_timer -= 1

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
