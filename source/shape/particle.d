module shape.particle;

import constants;
import shape.shape;
import math.math;
import math.vec3;
import shape.shape_config;

/**
 * A Particule shape
 * @author lo-th
 */

public class Particle : Shape {

    this( ShapeConfig config, float normal ) {
        super(config);

        this.type = SHAPE_PARTICLE;
    }

    float volume () {
        return float.max;
    }

    void calculateMassInfo ( Shape output ) {
        var inertia = 0;
        output.inertia.set( inertia, 0, 0, 0, inertia, 0, 0, 0, inertia );
    }

    void updateProxy () {

        var p = 0;//AABB_PROX;

        this.aabb.set(
            this.position.x - p, this.position.x + p,
            this.position.y - p, this.position.y + p,
            this.position.z - p, this.position.z + p
        );

        if ( this.proxy != null ) this.proxy.update();

    }

}