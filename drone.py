import pygame
import random
import math
import uuid
from settings import *

class Drone:
    """
    Represents a single agent in the swarm with state-based behavior.
    """
    def __init__(self, x, y):
        self.pos = pygame.math.Vector2(x, y)
        angle = random.uniform(0, 2 * math.pi)
        self.vel = pygame.math.Vector2(math.cos(angle), math.sin(angle)) * random.uniform(1, MAX_SPEED)
        self.acc = pygame.math.Vector2(0, 0)
        
        self.uid = uuid.uuid4().int
        self.state = 'PEER'
        self.leader_id = self.uid

    def apply_force(self, force):
        self.acc += force

    def wrap_edges(self):
        if self.pos.x > SCREEN_WIDTH: self.pos.x = 0
        elif self.pos.x < 0: self.pos.x = SCREEN_WIDTH
        if self.pos.y > SCREEN_HEIGHT: self.pos.y = 0
        elif self.pos.y < 0: self.pos.y = SCREEN_HEIGHT

    def steer_to(self, target):
        """Calculates the steering force to move towards a target."""
        desired = target - self.pos
        if desired.magnitude() == 0:
            return pygame.math.Vector2(0,0)
        
        desired.scale_to_length(MAX_SPEED)
        steer = desired - self.vel
        if steer.magnitude() > MAX_FORCE:
            steer.scale_to_length(MAX_FORCE)
        return steer
    
    def apply_boids(self, swarm):
        """Applies the three classic Boids rules."""
        steering_cohesion = pygame.math.Vector2(0, 0)
        steering_alignment = pygame.math.Vector2(0, 0)
        steering_separation = pygame.math.Vector2(0, 0)
        neighbor_count = 0

        for other_drone in swarm:
            if other_drone is not self:
                distance = self.pos.distance_to(other_drone.pos)
                if 0 < distance < PERCEPTION_RADIUS:
                    neighbor_count += 1
                    steering_cohesion += other_drone.pos
                    steering_alignment += other_drone.vel
                    diff = self.pos - other_drone.pos
                    if distance > 0: diff /= distance
                    steering_separation += diff
        
        if neighbor_count > 0:
            # Cohesion
            steering_cohesion /= neighbor_count
            steering_cohesion = self.steer_to(steering_cohesion)
            # Alignment
            steering_alignment /= neighbor_count
            if steering_alignment.magnitude() > 0: steering_alignment.scale_to_length(MAX_SPEED)
            steering_alignment -= self.vel
            # Separation
            steering_separation /= neighbor_count
            if steering_separation.magnitude() > 0: steering_separation.scale_to_length(MAX_SPEED)
            steering_separation -= self.vel
        
        self.apply_force(steering_cohesion * COHESION_WEIGHT)
        self.apply_force(steering_alignment * ALIGNMENT_WEIGHT)
        self.apply_force(steering_separation * SEPARATION_WEIGHT)

    def seek(self, target):
        """Leader's behavior: move towards a target."""
        self.apply_force(self.steer_to(target))

    def follow_leader(self, swarm):
        """Follower's behavior: modified Boids to follow the leader and other followers."""
        steering_cohesion = pygame.math.Vector2(0, 0)
        steering_alignment = pygame.math.Vector2(0, 0)
        steering_separation = pygame.math.Vector2(0, 0)
        total_neighbors = 0
        follower_neighbors = 0

        for other_drone in swarm:
            if other_drone is not self:
                distance = self.pos.distance_to(other_drone.pos)
                if 0 < distance < PERCEPTION_RADIUS:
                    total_neighbors += 1
                    steering_alignment += other_drone.vel
                    diff = self.pos - other_drone.pos
                    if distance > 0: diff /= distance
                    steering_separation += diff
                    
                    # Cohesion is calculated only with followers and the leader
                    if other_drone.state == 'FOLLOWER' or other_drone.state == 'LEADER':
                        steering_cohesion += other_drone.pos
                        follower_neighbors += 1
        
        if total_neighbors > 0:
            # Alignment
            steering_alignment /= total_neighbors
            if steering_alignment.magnitude() > 0: steering_alignment.scale_to_length(MAX_SPEED)
            steering_alignment -= self.vel
            # Separation
            steering_separation /= total_neighbors
            if steering_separation.magnitude() > 0: steering_separation.scale_to_length(MAX_SPEED)
            steering_separation -= self.vel
        
        if follower_neighbors > 0:
            # Cohesion
            steering_cohesion /= follower_neighbors
            steering_cohesion = self.steer_to(steering_cohesion)
        
        # Give cohesion a bit more weight for followers to encourage grouping
        self.apply_force(steering_cohesion * COHESION_WEIGHT * 2.0)
        self.apply_force(steering_alignment * ALIGNMENT_WEIGHT)
        self.apply_force(steering_separation * SEPARATION_WEIGHT)

    def update(self, swarm, target):
        # Stage 1: Leader Election Propagation
        for other_drone in swarm:
            if self.pos.distance_to(other_drone.pos) < PERCEPTION_RADIUS:
                if other_drone.leader_id > self.leader_id:
                    self.leader_id = other_drone.leader_id

        # Stage 2: State Determination
        if self.uid == self.leader_id:
            self.state = 'LEADER'
        else:
            # A drone becomes a follower if it's near the leader OR another follower.
            # This allows the "following" state to propagate through the swarm.
            is_near_a_follower = False
            for other_drone in swarm:
                if other_drone.state in ['LEADER', 'FOLLOWER']:
                    if self.pos.distance_to(other_drone.pos) < PERCEPTION_RADIUS:
                        is_near_a_follower = True
                        break
            
            if is_near_a_follower:
                self.state = 'FOLLOWER'
            else:
                self.state = 'PEER'

        # Stage 3: Apply Behavior Based on State
        if self.state == 'LEADER':
            if LEADER_FOLLOWS_MOUSE:
                self.seek(target)
            else:
                self.apply_boids(swarm)
        elif self.state == 'FOLLOWER':
            self.follow_leader(swarm)
        elif self.state == 'PEER':
            self.apply_boids(swarm)

        # Physics update
        self.vel += self.acc
        if self.vel.magnitude() > MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
        self.pos += self.vel
        self.acc *= 0
        self.wrap_edges()

    def draw(self, screen):
        # --- NEW: Draw the perception radius ---
        # Create a transparent surface
        radius_surface = pygame.Surface((PERCEPTION_RADIUS * 2, PERCEPTION_RADIUS * 2), pygame.SRCALPHA)
        # Draw the circle on the transparent surface
        pygame.draw.circle(radius_surface, RANGE_COLOR, (PERCEPTION_RADIUS, PERCEPTION_RADIUS), PERCEPTION_RADIUS)
        # Blit the surface onto the main screen
        screen.blit(radius_surface, (self.pos.x - PERCEPTION_RADIUS, self.pos.y - PERCEPTION_RADIUS))

        if self.state == 'LEADER':
            color = LEADER_COLOR
        elif self.state == 'FOLLOWER':
            color = FOLLOWER_COLOR
        else:
            color = PEER_COLOR

        # The angle of the drone is the angle of its velocity vector
        angle = self.vel.angle_to(pygame.math.Vector2(1, 0)) # Angle to the horizontal axis

        # Calculate the three points of the triangle
        # Point 1: The "nose" of the triangle
        p1 = self.pos + pygame.math.Vector2(TRIANGLE_HEIGHT / 2, 0).rotate(-angle)
        
        # Point 2: The "bottom-left" corner
        p2 = self.pos + pygame.math.Vector2(-TRIANGLE_HEIGHT / 2, TRIANGLE_BASE / 2).rotate(-angle)
        
        # Point 3: The "bottom-right" corner
        p3 = self.pos + pygame.math.Vector2(-TRIANGLE_HEIGHT / 2, -TRIANGLE_BASE / 2).rotate(-angle)

        pygame.draw.polygon(screen, color, [p1, p2, p3])
