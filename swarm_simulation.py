import pygame
import random
import math

# --- Constants ---
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
NUM_DRONES = 15
DRONE_RADIUS = 5
MAX_SPEED = 3
PERCEPTION_RADIUS = 100

# Boids rule weights
COHESION_WEIGHT = 0.005
ALIGNMENT_WEIGHT = 0.09
SEPARATION_WEIGHT = 0.01

# Colors
BACKGROUND_COLOR = (10, 10, 40) # Dark Blue
DRONE_COLOR = (255, 255, 255) # White


class Drone:
    """
    Represents a single agent in the swarm.
    """
    def __init__(self, x, y):
        # Use pygame's Vector2 for easier vector math
        self.pos = pygame.math.Vector2(x, y)
        angle = random.uniform(0, 2 * math.pi)
        self.vel = pygame.math.Vector2(math.cos(angle), math.sin(angle)) * random.uniform(1, MAX_SPEED)
        self.acc = pygame.math.Vector2(0, 0)

    def apply_force(self, force):
        """Applies a steering force to the drone's acceleration."""
        self.acc += force

    def wrap_edges(self):
        """Wraps the drone around the screen edges."""
        if self.pos.x > SCREEN_WIDTH:
            self.pos.x = 0
        elif self.pos.x < 0:
            self.pos.x = SCREEN_WIDTH
        
        if self.pos.y > SCREEN_HEIGHT:
            self.pos.y = 0
        elif self.pos.y < 0:
            self.pos.y = SCREEN_HEIGHT

    def update(self, swarm):
        """
        Updates the drone's state based on the Boids rules.
        This is the core of the simulation.
        """
        # --- Boids Algorithm Implementation ---
        
        # Reset steering vectors and neighbor count for this frame
        steering_cohesion = pygame.math.Vector2(0, 0)
        steering_alignment = pygame.math.Vector2(0, 0)
        steering_separation = pygame.math.Vector2(0, 0)
        neighbor_count = 0

        # Loop through every other drone in the swarm
        for other_drone in swarm:
            if other_drone is not self:
                # Calculate distance to the other drone
                distance = self.pos.distance_to(other_drone.pos)
                
                # Check if the other drone is a neighbor (within perception radius)
                if 0 < distance < PERCEPTION_RADIUS:
                    neighbor_count += 1
                    
                    # 1. Cohesion: Add other drone's position to calculate the center of mass later
                    steering_cohesion += other_drone.pos
                    
                    # 2. Alignment: Add other drone's velocity to average it later
                    steering_alignment += other_drone.vel
                    
                    # 3. Separation: Calculate a vector pointing away from the other drone
                    diff = self.pos - other_drone.pos
                    if distance > 0:
                        diff /= distance  # The closer the drone, the stronger the push
                    steering_separation += diff
        
        # If there are neighbors, calculate the final steering forces
        if neighbor_count > 0:
            # Cohesion calculation
            steering_cohesion /= neighbor_count   # Get the average position (center of mass)
            steering_cohesion -= self.pos         # Get the vector from self to the center
            if steering_cohesion.magnitude() > 0:
                steering_cohesion.scale_to_length(MAX_SPEED)
            steering_cohesion -= self.vel
            
            # Alignment calculation
            steering_alignment /= neighbor_count # Get the average velocity
            if steering_alignment.magnitude() > 0:
                steering_alignment.scale_to_length(MAX_SPEED)
            steering_alignment -= self.vel
            
            # Separation calculation
            steering_separation /= neighbor_count # Get the average separation vector
            if steering_separation.magnitude() > 0:
                steering_separation.scale_to_length(MAX_SPEED)
            steering_separation -= self.vel

        # Apply the weighted forces to the drone's acceleration
        self.apply_force(steering_cohesion * COHESION_WEIGHT)
        self.apply_force(steering_alignment * ALIGNMENT_WEIGHT)
        self.apply_force(steering_separation * SEPARATION_WEIGHT)
        
        # --- Physics Update ---
        self.vel += self.acc
        # Limit the speed of the drone
        if self.vel.magnitude() > MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
            
        self.pos += self.vel
        self.acc *= 0 # Reset acceleration for the next frame
        
        # Handle screen boundaries
        self.wrap_edges()

    def draw(self, screen):
        """Draws the drone on the screen as a circle."""
        pygame.draw.circle(screen, DRONE_COLOR, self.pos, DRONE_RADIUS)


def main():
    """Main function to set up and run the simulation."""
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Boids Drone Swarm Simulation")
    clock = pygame.time.Clock()

    # Create the swarm of drones
    swarm = [Drone(random.randint(0, SCREEN_WIDTH), random.randint(0, SCREEN_HEIGHT)) for _ in range(NUM_DRONES)]

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # --- Update and Draw ---
        screen.fill(BACKGROUND_COLOR)

        for drone in swarm:
            drone.update(swarm)
            drone.draw(screen)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()