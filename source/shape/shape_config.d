module shape.shape_config;

import math.vec3;
import math.mat33;
/**
 * A shape configuration holds common configuration data for constructing a shape.
 * These configurations can be reused safely.
 *
 * @author saharan
 * @author lo-th
 */
 
public class ShapeConfig{
    // position of the shape in parent's coordinate system.
    Vec3 relativePosition;
    // rotation matrix of the shape in parent's coordinate system.
    Mat33 relativeRotation;
    // coefficient of friction of the shape.
    float friction;
    // coefficient of restitution of the shape.
    float restitution;
    // density of the shape.
    int density;
    // bits of the collision groups to which the shape belongs.
    int belongsTo;
    // bits of the collision groups with which the shape collides.
    int collidesWith;


    this() {
        // position of the shape in parent's coordinate system.
        this.relativePosition = new Vec3();
        // rotation matrix of the shape in parent's coordinate system.
        this.relativeRotation = new Mat33();
        // coefficient of friction of the shape.
        this.friction = 0.2; // 0.4
        // coefficient of restitution of the shape.
        this.restitution = 0.2;
        // density of the shape.
        this.density = 1;
        // bits of the collision groups to which the shape belongs.
        this.belongsTo = 1;
        // bits of the collision groups with which the shape collides.
        this.collidesWith = 0xffffffff;
    }
}