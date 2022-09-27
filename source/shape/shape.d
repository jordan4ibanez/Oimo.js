module shape.shape;


import constants;
import math.math;
import math.vec3;
import math.mat33;
import math.aabb;
import shape.shape_config;

import std.algorithm.comparison : equal;
import std.container : DList;

private int count = 0;

int shapeIdCount() {
    return count++;
}

/**
 * A shape is used to detect collisions of rigid bodies.
 *
 * @author saharan
 * @author lo-th
 */

class Shape {

    int type;

    // global identification of the shape should be unique to the shape.
    int id;

    // previous shape in parent rigid body. Used for fast interations.
    Shape prev;

    // next shape in parent rigid body. Used for fast interations.
    Shape next;

    // proxy of the shape used for broad-phase collision detection.
    BasicProxy proxy;

    // parent rigid body of the shape.
    RigidBody parent;

    // linked list of the contacts with the shape.
    DList!Shape contactLink;

    // number of the contacts with the shape.
    int numContacts;

    // center of gravity of the shape in world coordinate system.
    Vec3 position;

    // rotation matrix of the shape in world coordinate system.
    Mat33 rotation;

    // position of the shape in parent's coordinate system.
    Vec3 relativePosition;

    // rotation matrix of the shape in parent's coordinate system.
    Mat33 relativeRotation;

    // axis-aligned bounding box of the shape.
    AABB aabb;

    // density of the shape.
    float density;

    // coefficient of friction of the shape.
    float friction;

    // coefficient of restitution of the shape.
    float restitution;

    // bits of the collision groups to which the shape belongs.
    int belongsTo;

    // bits of the collision groups with which the shape collides.
    int collidesWith;

    this(ShapeConfig config) {

        this.type = SHAPE_NULL;

        // global identification of the shape should be unique to the shape.
        this.id = ShapeIdCount();

        // previous shape in parent rigid body. Used for fast interations.
        this.prev = null;

        // next shape in parent rigid body. Used for fast interations.
        this.next = null;

        // proxy of the shape used for broad-phase collision detection.
        this.proxy = null;

        // parent rigid body of the shape.
        this.parent = null;

        // linked list of the contacts with the shape.
        this.contactLink = null;

        // number of the contacts with the shape.
        this.numContacts = 0;

        // center of gravity of the shape in world coordinate system.
        this.position = new Vec3();

        // rotation matrix of the shape in world coordinate system.
        this.rotation = new Mat33();

        // position of the shape in parent's coordinate system.
        this.relativePosition = new Vec3().copy( config.relativePosition );

        // rotation matrix of the shape in parent's coordinate system.
        this.relativeRotation = new Mat33().copy( config.relativeRotation );

        // axis-aligned bounding box of the shape.
        this.aabb = new AABB();

        // density of the shape.
        this.density = config.density;

        // coefficient of friction of the shape.
        this.friction = config.friction;

        // coefficient of restitution of the shape.
        this.restitution = config.restitution;

        // bits of the collision groups to which the shape belongs.
        this.belongsTo = config.belongsTo;

        // bits of the collision groups with which the shape collides.
        this.collidesWith = config.collidesWith;

    }

    /// D translation notes: These are overrides

    // Calculate the mass information of the shape.

    void calculateMassInfo ( Shape output ){

        throw new Exception("Shape Inheritance error.");

    }

    // Update the proxy of the shape.

    void updateProxy(){

        throw new Exception("Shape Inheritance error.");

    }
}