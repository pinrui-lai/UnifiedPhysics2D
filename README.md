[English](README.md) | [日本語](README.ja.md)
# Unified Physics 2D

![Opening GIF](assets/Opening.gif)  
A 2D physics engine developed in C++ that supports interactions between rigid bodies and soft bodies.

The engine implements collision detection using GJK/EPA, persistent contact tracking with Persistent Manifolds, and constraint solving with Sequential Impulse and XPBD. Key areas of focus include data-oriented design, hierarchical collision detection, parallelization, memory-management optimization, and the analysis and handling of numerical degeneracies.

**[Download Demo (Windows x64)](https://github.com/pinrui-lai/UnifiedPhysics2D/releases/latest)**

## Tech Stack

- **Language:** C++20
- **Collision Detection:** Morton Codes, BVH, AABB, GJK, EPA, Clipping, SDF
- **Constraint Solvers:** Sequential Impulse, XPBD, Warm Starting, Split Impulse
- **Constraint Types:** Joint, Angular, Area, Pressure, Contact, Collision, Distance
- **Performance:** SoA / AoS, `std::pmr`, `thread_local`, `std::atomic`, multithreading
- **Visualization:** Dear ImGui

## Physics Architecture

| Area | Implementation |
|---|---|
| Broad Phase | A global BVH built using Morton codes. Each soft body is represented by its overall AABB. |
| Rigid × Rigid | GJK / EPA / Clipping, with Feature ID–based Persistent Manifolds |
| Rigid × Soft | SDF-based collision detection against the surface particles of soft bodies |
| Soft × Soft | A local BVH for each soft body, with surface-particle AABBs at each node. After comparing AABBs, the engine checks for intersections between surface particles and surface edges. |
| Rigid × Rigid Solver | Sequential Impulse, Warm Starting, Split Impulse |
| Soft × Soft Solver | XPBD-based constraint solving |
| Rigid × Soft Solver | Converts XPBD's $\Delta\lambda$ into an impulse applied to the rigid body to resolve interactions |

## Engineering Highlights

### Data-Oriented Design: Choosing Between SoA and AoS

SoA is used for operations that benefit from sequential data access. AoS is used in operations that require more random access, such as the narrow phase and constraint solvers, to help reduce cache misses.

Rather than enforcing a single layout throughout the engine, data is organized according to each operation's memory-access patterns.

### Hierarchical Collision Detection for Soft Bodies

<table>
  <tr>
    <th>Local BVHs</th>
    <th>Global BVH</th>
  </tr>
  <tr>
    <td width="50%">
      <img src="assets/LocalBVH.gif" alt="LocalBVH.gif">
    </td>
    <td width="50%">
      <img src="assets/GlobalBVH.jpg" alt="GlobalBVH.jpg">
    </td>
  </tr>
</table>

In the broad phase, a global BVH is built using each soft body's overall AABB. In the narrow phase, a local BVH is built for each soft body using AABBs for its surface particles. This narrows down the particle–edge pairs that require more detailed collision checks.

Local BVHs are updated only when the broad phase detects an overlap with another soft body, avoiding unnecessary update work.

### Persistent Contact Tracking

Each contact point is assigned a Feature ID, allowing the Persistent Manifold to track corresponding contacts across frames.

Accumulated impulses from matching contact points are reused through Warm Starting, helping maintain constraint-solver stability in scenes with persistent contact.

### Memory and Computational Optimization

- Use `std::pmr::monotonic_buffer_resource` to reduce temporary memory-allocation overhead.
- Use `std::execution` to parallelize work outside the constraint-solving stage.
- Use `thread_local` in multithreaded code to reduce memory-allocation overhead.
- Use `std::atomic` in multithreaded code to ensure safe access to shared data across threads.
- For XPBD angular constraints, precompute the target angle as a direction vector. During constraint evaluation, the cross product of the anchor-to-particle vector and the target direction vector is used as the constraint function $C$, avoiding the cost of computing the angle with inverse trigonometric functions.
- Precompute reciprocals for values repeatedly used as divisors, such as `delta_time`, and use multiplication in subsequent calculations to reduce division operations.
- Cache the sine and cosine of rigid-body rotation angles.
- Use sub-stepping to subdivide the simulation time step.
- Use `std::array` to store temporary data for the GJK simplex, EPA polytope, and BVH-query stack, reducing the need for dynamic allocations through `std::vector`.

<table>
  <tr>
    <th colspan="2">Soft Circle Stress Test & Box Stress Test</th>
  </tr>
  <tr>
    <td>
        <img src="assets/BoxStressWide.jpg" alt="BoxStressWide.jpg">
    </td>
  </tr>
  <tr>
    <td>
        <img src="assets/SoftCircleStressWide.jpg" alt="SoftCircleStressWide.jpg">
    </td>
  </tr>
</table>

The following table shows the performance results for the **Soft Circle Stress Test** shown in the image above.

<table>
  <tr>
    <th colspan="3">Soft Circle Stress Test / 68 Soft Circles / 6800 particles / ≈ 165 soft-soft pairs / ≈ 1200 particle-edge contacts / CPU: AMD R5 5600</th>
  </tr>
  <tr>
    <th>Soft-Soft Detection</th>
    <th>Multithread</th>
    <th>Single thread</th>
  </tr>
  <tr>
    <th>Local BVH</th>
    <th>≈ 0.75ms</th>
    <th>≈ 2.90ms</th>
  </tr>
  <tr>
    <th>Brute Force</th>
    <th>≈ 1.47ms</th>
    <th>≈ 8.30ms</th>
  </tr>
</table>

## Technical Challenges

### 1. Simulation Freeze Caused by an Infinite Loop in GJK

#### Investigation and Root Cause

Step-by-step tracing revealed a failure case in which the edges of two boxes were exactly touching. The simplex edge length became extremely small (a degenerate configuration), causing the next search-direction vector to approach zero. This exposed numerical instability near the limits of floating-point precision. The search direction was not updated properly, causing GJK to enter an infinite loop and freeze the entire simulation.

#### Solution

Added an epsilon-based threshold check. When the search-direction magnitude falls below the threshold, a fallback path terminates the iteration, preventing the loop from running indefinitely and improving simulation robustness.

### 2. Unstable Box Stacking

#### Investigation and Root Cause

A custom Dear ImGui debug panel was implemented to visualize contact normals and incident/reference edges in real time, making it possible to inspect the collision-detection process.

The investigation identified a failure during grazing contact: the simplex generated by GJK became degenerate, leading to incorrect contact normals and penetration depths. The constraint solver consequently received inaccurate contact information and calculated incorrect impulses.

The incorrect contact normal also affected clipping, causing the engine to select the wrong incident and reference edges. As a result, the current frame's Feature IDs did not correctly match those from the previous frame, so accumulated impulses could not be carried over to the corresponding contact points. This made box stacking unstable.

#### Solution

Applied the same fallback strategy used for the GJK infinite-loop issue when the simplex becomes degenerate, ensuring the accuracy of the resulting contact information.

<table>
  <tr>
    <th>Contact Info</th>
    <th>Spawn Bodies / Apply a force to left</th>
  </tr>
  <tr>
    <td width="55%">
      <img src="assets/ContactInfo.jpg" alt="ContactInfo.jpg">
    </td>
    <td>
      <img src="assets/Spawn.gif" alt="Spawn.gif">
      <img src="assets/SoftDemo.gif" alt="SoftDemo.gif">
    </td">
  </tr>
</table>

## Debugging and Visualization

Implemented a runtime tuning and debugging UI with Dear ImGui, providing:

- Real-time adjustment of physics parameters
- Inspection of physics-processing execution times
- Visualization of the BVH hierarchy
- Rendering of contact points and contact normals
- Rendering of incident and reference edges

By visualizing intermediate states in collision detection, the debugging tools make it possible to investigate numerical anomalies at each stage of the algorithm.
