module constants;

/*
 * A list of constants built-in for
 * the physics engine.
 */

const auto REVISION = "1.0.9";

// BroadPhase
const auto BR_NULL = 0;
const auto BR_BRUTE_FORCE = 1;
const auto BR_SWEEP_AND_PRUNE = 2;
const auto BR_BOUNDING_VOLUME_TREE = 3;

// Body type
const auto BODY_NULL = 0;
const auto BODY_DYNAMIC = 1;
const auto BODY_STATIC = 2;
const auto BODY_KINEMATIC = 3;
const auto BODY_GHOST = 4;

// Shape type
const auto SHAPE_NULL = 0;
const auto SHAPE_SPHERE = 1;
const auto SHAPE_BOX = 2;
const auto SHAPE_CYLINDER = 3;
const auto SHAPE_PLANE = 4;
const auto SHAPE_PARTICLE = 5;
const auto SHAPE_TETRA = 6;

// Joint type
const auto JOINT_NULL = 0;
const auto JOINT_DISTANCE = 1;
const auto JOINT_BALL_AND_SOCKET = 2;
const auto JOINT_HINGE = 3;
const auto JOINT_WHEEL = 4;
const auto JOINT_SLIDER = 5;
const auto JOINT_PRISMATIC = 6;

// AABB aproximation
const auto AABB_PROX = 0.005;