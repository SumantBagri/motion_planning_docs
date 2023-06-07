# Gilbert Johnson Keerthi Algorithm (GJK)

$\underline{\text{Objective}}$: Check if given two shapes (convex/concave) are intersecting

$\text{\underline{Key Ideas}}$:
1. Every concave shape can be broken into a group of convex shapes $\Rightarrow$ all problems are reduced to convex shape intersection
2. Given two points from two, distinct, object point clouds, if the difference between two points is zero, then we have an intersection

$\underline{\text{Minkowski Difference} (A \ominus B)}$: Take every point from one shape and add(subtract) to every other point in the other shape (vector add / vector subtract)

$$(A \ominus B) = \left\{ a + (-b) | a \in A, b \in B \right\}$$

$\underline{\text{Properties of Minkowski difference}}$
1. If $A$ and $B$ convex $\rightarrow A \ominus B$ is convex
2. If $A$ and $B$ intersect $\rightarrow (0,0) \in A \ominus B$

$\underline{\text{Task}}$: Compute the Minkowski difference of the given two shapes and check if it contains the origin

$\underline{\text{Problem}}$: Calculating the full Minkowski difference is infeasible

$\underline{\text{Simplex}}$: The simplest possible polytope (a geometric object with flat sides) in any give dimension (d-0: point, d-1: line, d-2: triangle, d-3: tetrahedron, etc). $k$-Simplex is a shape that is guaranteed to enclose a point in $k$-dimensional space.

$\underline{\text{Idea}}$: Instead of checking if the entire Minkowski difference contains the origin, can we find a simplex that encloses the origin using the points on the "boundary" of the Minkowski difference ? (_this idea stems from the first property of the minkowski difference_)

$\underline{\text{Problems}}$:
1. There are infinite set of points _on_ the Minkowski difference. Need a scheme to pick points
2. Said scheme should maximize our chances of enclosing the origin quickly (the efficiency of the algorithm depends on how fast we can figure out if enclosing the origin is possible or not)

$\underline{\text{Key Idea}}$: For a convex shape, given a direction vector, there is a unique point _on_ the shape that is furthest in that direction vector. These points form the _convex hull_ of the shape

$\underline{\text{Support Points and Support Functions}}$: Support points are the finite set of points on the convex shape, $A$, forming the convex hull, $CH(A)$: $\left\{ p | p \in CH(A)\right\}$. A support function, $s_A$, maps a direction vector $\overrightarrow{d}$ to a unique support point furthest in that direction.

$$s_B(\overrightarrow{d}) = v = \argmax_{v \in B} \{v^T \cdot \overrightarrow{d}\}$$

$\underline{\text{Key Ideas}}$:
1. The support points of a Minkowski difference can be found by subtracting the support points of the individual shapes in opposite directions:

$$s_{A \ominus B}(\overrightarrow{d}) = s_A(\overrightarrow{d}) - s_B(-\overrightarrow{d})$$

2. If each shape has a defined way of computing support points, we dont need to know about the type of shapes we are dealing with directly. 

$\underline{\text{Achievements}}$:
1. We can now get the points on the "boundary" of the Minkowski difference without actually computing the full difference.
2. Since we can pick points based on direction, we can quickly check if there is a simplex enclosing the origin by always picking points in the direction of the origin: either from a given support point or from the normal to a line-segment. (_starting with a randomly selected support point_)

$\underline{\text{Algorithm}}$:
(_all discussion confined to the Minkowski difference of the given two shapes_)
1. Pick a support point randomly (0-simplex)
2. From the chosen support point, get the next support point in the direction of the origin
3. Check if the new support point "passed" the origin
4. If yes, get the next support point from the normal to the 1-simplex else report "not-intersecting"
5. Repeat steps 3-4 _"k-1"_ times for $k$-dimensions
6. Check if the $k$-simplex contains the origin
7. If yes, report "intersecting", else pick a new support point from the normal to the $k-1$-simplex "closest" to the origin
8. Check if the new support point is not part of the $k-1$-simplex
9. If not, then "update" the $k$-simplex and repeat steps 6-8, else report "not-intersecting"


# FCL (Flexible Collision Checking)
3 types of proximity queries on a pair of geometric models composed of triangles
- Collision detection
- Distance computation
- Tolerance verification

Supported object shapes:
- box
- sphere
- ellipsoid
- mesh
- octrees (using octomap library)


## Steps involved in checking for discrete collisions
1. Create a the collision objects
    - Create a geometry of an object as a mesh soup (vertices + triangles)
    - Create the transform of an object (rotation + translation)
    - Pass them as arguments to `CollisionObjectf`
2. Set the collision request data structure(`CollisionRequest`) and initialize the collision result data structure(`CollisionResult`)
3. Run the collision function by passing in the objects, request and result data structures

> **_NOTE:_**  By setting the collision request, the user can easily choose whether to return contact information (which is slower) or just return binary collision results (which is faster).

## Steps involved in continuos collision checking

For continuous collision, FCL requires the goal transform to be provided (the initial transform is included in the collision object data structure).

```cpp
// Given two objects o1 and o2
CollisionObjectf* o1 = ...
CollisionObjectf* o2 = ...
// The goal transforms for o1 and o2
Transform3f tf_goal_o1 = ...
Transform3f tf_goal_o2 = ...
// set the continuous collision request structure, here we just use the default
// setting
ContinuousCollisionRequest request;
// result will be returned via the continuous collision result structure
ContinuousCollisionResult result;
// perform continuous collision test
continuousCollide(o1, tf_goal_o1, o2, tf_goal_o2, request, result);
```
## CollisionManager, CollisionData and CollisionFunction
FCL uses a CollisionManager structure to manage all the objects involving the collision or distance operations. In-order to query for collision between objects, the manager calls the `collide()` method with a callback to default (or a user-defined) `CollisionFunction` callback passed as an argument along-with the `CollisionData` data structure to retrieve the results post computation.

---

# MoveIt2 collision checking

Collision checking in MoveIt is configured inside a Planning Scene using the `CollisionWorld` object. Fortunately, MoveIt is set up so that users never really have to worry about how collision checking is happening. Collision checking in MoveIt is mainly carried out using the FCL package - the primary collision checking library of MoveIt.

## Changing the collision detector

```cpp
// define the robot model
robot_model::RobotModelPtr robot_model = moveit::core::loadTestingRobotModel("panda");
// define the planning scene using above robot model
auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
// set the active collision detector for defined planning scene
planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
```

# Add Obstacle in MoveIt

1. Define `CollisionObject`

```c++
moveit_msgs::msg::CollisionObject collision_object;
collision_object
```