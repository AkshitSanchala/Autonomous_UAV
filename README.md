# Drone Swarm Simulation

This simulation demonstrates the flocking behavior of autonomous agents using the Boid algorithm, with additional features for leader election and state-based behaviors.

## The Boid Algorithm

The Boid algorithm, developed by Craig Reynolds, simulates the flocking behavior of birds. It is based on three simple rules that individual agents, or "boids," follow. The complex, emergent behavior of the entire flock arises from these simple, localized interactions.

The three fundamental rules are:

1.  **Separation**: Each boid steers to avoid crowding its local flockmates. This prevents collisions and keeps the drones from clustering too tightly.
2.  **Alignment**: Each boid steers towards the average heading of its local flockmates. This rule ensures that the drones move in the same general direction, creating a sense of coordinated movement.
3.  **Cohesion**: Each boid steers to move toward the average position of its local flockmates. This rule keeps the swarm together and prevents it from dispersing.

## Simulation Implementation

Here’s how we’ve implemented and expanded upon the Boid algorithm in this simulation:

### 1. Core Implementation (`drone.py`)

The `apply_boids` method in the `Drone` class is the heart of the Boid algorithm implementation. For each drone, it:

-   **Identifies Neighbors**: It iterates through all other drones to find those within its `PERCEPTION_RADIUS`.
-   **Calculates Steering Forces**:
    -   **Separation**: It computes a vector pointing away from each neighbor and sums them to create a repulsive force.
    -   **Alignment**: It averages the velocity vectors of its neighbors to determine the flock's general direction.
    -   **Cohesion**: It calculates the average position of its neighbors (the center of the local flock) and creates a force to move toward it.
-   **Applies Forces**: These three forces are scaled by their respective weights (`SEPARATION_WEIGHT`, `ALIGNMENT_WEIGHT`, `COHESION_WEIGHT`) and applied to the drone's acceleration.

### 3. Boid Rule Weights

The behavior of the swarm is highly sensitive to the weights assigned to the three boid rules. There are no "correct" values, as the ideal weights depend on the desired emergent behavior. However, here is a general guide to their ranges and effects:

-   **`COHESION_WEIGHT`**: Typically a small value (e.g., `0.01` - `0.1`).
    -   **Too low**: The swarm will be loosely gathered and may break apart.
    -   **Too high**: The swarm will collapse into a tight, ball-like cluster.

-   **`ALIGNMENT_WEIGHT`**: Usually the most influential weight (e.g., `0.1` - `0.5`).
    -   **Too low**: Drones will not form a cohesive direction and will appear disorganized.
    -   **Too high**: The swarm will move rigidly, like a single unit, with little individual variation.

-   **`SEPARATION_WEIGHT`**: This weight is often very small, as it can easily overpower the others (e.g., `0.001` - `0.01`).
    -   **Too low**: Drones will not avoid each other effectively and may overlap.
    -   **Too high**: The swarm will be pushed apart and may disperse entirely.

Finding the right balance between these weights is key to achieving the desired flocking behavior.

### 2. Simulation Enhancements

We've added several features to create a more advanced and visually intuitive simulation:

-   **Leader Election**: A simple leader election mechanism is in place where the drone with the highest UID becomes the leader. This introduces a hierarchical structure to the swarm.
-   **State-Based Behavior**: Drones can be in one of three states, each with a distinct color and behavior:
    -   `LEADER` (Green): The leader follows the standard boids rules, guiding the swarm.
    -   `FOLLOWER` (Orange): Followers are influenced by the leader and other followers. Their cohesion is directed toward other followers and the leader, allowing the leader's influence to propagate through the swarm, even to drones outside its direct perception.
    -   `PEER` (White): Peers are drones that are not close enough to the leader or any followers. They exhibit standard flocking behavior among themselves.
-   **Visual Feedback**: To make the simulation easier to understand, we've added:
    -   Distinct colors for each state.
    -   A transparent circle around each drone to visualize its perception radius.

By combining the foundational Boid algorithm with these enhancements, we've created a dynamic simulation that demonstrates emergent behavior, hierarchical control, and the propagation of influence through a swarm.
