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
        self.leader_pos = None
        self.uid = uuid.uuid4().int
        self.state = 'PEER'
        self.leader_id = self.uid
        self.color = PEER_COLOR

    def apply_force(self, force):
        self.acc += force

    def avoid_edges(self):
        """Applies a steering force to keep the drone within the screen boundaries."""
        turn_force = pygame.math.Vector2(0, 0)
        force_multiplier = 3
        if self.pos.x < TURN_MARGIN:
            turn_force.x = TURN_FORCE * force_multiplier
        elif self.pos.x > SCREEN_WIDTH - TURN_MARGIN:
            turn_force.x = -TURN_FORCE * force_multiplier
        
        if self.pos.y < TURN_MARGIN:
            turn_force.y = TURN_FORCE * force_multiplier
        elif self.pos.y > SCREEN_HEIGHT - TURN_MARGIN:
            turn_force.y = -TURN_FORCE * force_multiplier
            
        self.apply_force(turn_force)

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

    # drone.py -> replace the old follow_leader method with this one

    def follow_leader(self, swarm):
        """
        Follower's behavior: Steer towards the leader using the most reliable
        information available (lowest hop count) and apply Boids rules.
        """
        # --- Initialize forces and neighbor data ---
        steering_cohesion = pygame.math.Vector2(0, 0)
        steering_alignment = pygame.math.Vector2(0, 0)
        steering_separation = pygame.math.Vector2(0, 0)
        neighbor_count = 0

        # --- Data for finding the best leader position ---
        best_leader_pos_candidate = None
        min_hops = float('inf')

        # --- Single loop to scan neighbors for Boids rules and leader info ---
        for other_drone in swarm:
            if other_drone is self:
                continue
            
            distance = self.pos.distance_to(other_drone.pos)
            if 0 < distance < PERCEPTION_RADIUS:
                # --- Standard Boids calculations (applied to all neighbors) ---
                neighbor_count += 1
                steering_alignment += other_drone.vel
                diff = self.pos - other_drone.pos
                if distance > 0: diff /= distance
                steering_separation += diff

                # --- Leader Information Propagation Logic ---
                # Case 1: Neighbor IS the leader (0 hops). This is the best possible info.
                if other_drone.state == 'LEADER':
                    if 0 < min_hops: # Prioritize direct contact over relayed info
                        min_hops = 0
                        best_leader_pos_candidate = other_drone.pos
                
                # Case 2: Neighbor is another follower with valid, non-stale info.
                elif other_drone.leader_pos is not None and other_drone.leader_pos_hops is not float('inf'):
                    # The hop count for info from this neighbor is their count + 1.
                    hops_via_neighbor = other_drone.leader_pos_hops + 1
                    if hops_via_neighbor < min_hops:
                        min_hops = hops_via_neighbor
                        best_leader_pos_candidate = other_drone.leader_pos

        # --- Update own state based on the best information found ---
        self.leader_pos = best_leader_pos_candidate
        self.leader_pos_hops = min_hops

        # --- Finalize and Apply Forces ---
        # 1. Cohesion Force (only if a valid leader position was found)
        if self.leader_pos is not None and min_hops <= MAX_HOPS:
            steering_cohesion = self.steer_to(self.leader_pos)
            if self.leader_pos_hops > 0: # If it's a secondary follower
                self.color = SECONDARY_FOLLOWER_COLOR # This logic now works perfectly with hops

        # 2. Boids Forces (if there were any neighbors)
        if neighbor_count > 0:
            # Alignment
            steering_alignment /= neighbor_count
            if steering_alignment.magnitude() > 0: steering_alignment.scale_to_length(MAX_SPEED)
            steering_alignment -= self.vel
            # Separation
            steering_separation /= neighbor_count
            if steering_separation.magnitude() > 0: steering_separation.scale_to_length(MAX_SPEED)
            steering_separation -= self.vel

        # 3. Apply all calculated forces
        if self.leader_pos is not None:
            self.apply_force(steering_cohesion * min(float(min_hops+1),4) * COHESION_WEIGHT)
        
        self.apply_force(steering_alignment * ALIGNMENT_WEIGHT)
        self.apply_force(steering_separation * SEPARATION_WEIGHT)

    def update(self, swarm, target, leader_election_triggered):
        if leader_election_triggered:
            # Stage 1: Leader Election Propagation
            for other_drone in swarm:
                if self.pos.distance_to(other_drone.pos) < PERCEPTION_RADIUS:
                    if other_drone.leader_id > self.leader_id:
                        self.leader_id = other_drone.leader_id

            # Stage 2: State Determination
            if self.uid == self.leader_id:
                self.state = 'LEADER'
            else:
                # A drone becomes a follower ONLY if it's near the leader.
                is_near_leader = False
                for other_drone in swarm:
                    if other_drone.state == 'LEADER':
                        if self.pos.distance_to(other_drone.pos) < PERCEPTION_RADIUS:
                            is_near_leader = True
                            break
                
                if is_near_leader:
                    self.state = 'FOLLOWER'
                else:
                    self.state = 'PEER'
        else:
            self.state = 'PEER'

        # Stage 3: Apply Behavior Based on State
        if self.state == 'LEADER':
            self.color = LEADER_COLOR
            if LEADER_FOLLOWS_MOUSE:
                self.seek(target)
            else:
                self.apply_boids(swarm)
        elif self.state == 'FOLLOWER':
            self.color = FOLLOWER_COLOR
            self.follow_leader(swarm)
        elif self.state == 'PEER':
            self.color = PEER_COLOR
            if leader_election_triggered:
                self.follow_leader(swarm)
            else: 
                self.apply_boids(swarm)
        

        # Physics update
        self.vel += self.acc
        if self.vel.magnitude() > MAX_SPEED or self.vel.magnitude() < MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
        self.pos += self.vel
        self.acc *= 0
        self.avoid_edges()

    def draw(self, screen):
        # --- NEW: Draw the perception radius ---
        # Create a transparent surface
        radius_surface = pygame.Surface((PERCEPTION_RADIUS * 2, PERCEPTION_RADIUS * 2), pygame.SRCALPHA)
        # Draw the circle on the transparent surface
        pygame.draw.circle(radius_surface, RANGE_COLOR, (PERCEPTION_RADIUS, PERCEPTION_RADIUS), PERCEPTION_RADIUS)
        # Blit the surface onto the main screen
        screen.blit(radius_surface, (self.pos.x - PERCEPTION_RADIUS, self.pos.y - PERCEPTION_RADIUS))

        color = self.color

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
