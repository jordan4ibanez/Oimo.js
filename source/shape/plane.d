module shape.plane;

import constants;
import shape.shape;
import shape.shape_config;
import math.math;
import math.vec3;

/**
 * Plane shape.
 * @author lo-th
 */

public class Plane : Shape {
    Vec3 normal;
    this( ShapeConfig config, float normal ) {

        super(config);

        this.type = SHAPE_PLANE;

        // radius of the shape.
        this.normal = new Vec3( 0, 1, 0 );

    }

    float volume () {
        return float.max;
    }

    override
    void calculateMassInfo ( Shape output ) {

        output.mass = this.density;//0.0001;
        float inertia = 1;
        output.inertia.set( inertia, 0, 0, 0, inertia, 0, 0, 0, inertia );

    }

    override
    void updateProxy () {

        float p = AABB_PROX;

        float min = -_Math.INF;
        float max = _Math.INF;
        Vec3 n = this.normal;
        // The plane AABB is infinite, except if the normal is pointing along any axis
        this.aabb.set(
            n.x == -1 ? this.position.x - p : min, n.x == 1 ? this.position.x + p : max,
            n.y == -1 ? this.position.y - p : min, n.y == 1 ? this.position.y + p : max,
            n.z == -1 ? this.position.z - p : min, n.z == 1 ? this.position.z + p : max
        );

        if ( this.proxy !is null ) this.proxy.update();

    }

}