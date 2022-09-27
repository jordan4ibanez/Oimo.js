module constants;

/*
 * A list of constants built-in for
 * the physics engine.
 */

auto REVISION = "1.0.9";

// BroadPhase
auto BR_NULL = 0;
auto BR_BRUTE_FORCE = 1;
auto BR_SWEEP_AND_PRUNE = 2;
auto BR_BOUNDING_VOLUME_TREE = 3;

// Body type
auto BODY_NULL = 0;
auto BODY_DYNAMIC = 1;
auto BODY_STATIC = 2;
auto BODY_KINEMATIC = 3;
auto BODY_GHOST = 4;

// Shape type
auto SHAPE_NULL = 0;
auto SHAPE_SPHERE = 1;
auto SHAPE_BOX = 2;
auto SHAPE_CYLINDER = 3;
auto SHAPE_PLANE = 4;
auto SHAPE_PARTICLE = 5;
auto SHAPE_TETRA = 6;

// Joint type
auto JOINT_NULL = 0;
auto JOINT_DISTANCE = 1;
auto JOINT_BALL_AND_SOCKET = 2;
auto JOINT_HINGE = 3;
auto JOINT_WHEEL = 4;
auto JOINT_SLIDER = 5;
auto JOINT_PRISMATIC = 6;

// AABB aproximation
auto AABB_PROX = 0.005;