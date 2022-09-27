module shape.tetra;

import constants;
import shape.shape;
import shape.shape_config;
import math.vec3;

/**
 * A tetra shape.
 * @author xprogram
 */

public class Tetra : Shape {

    float[] verts;
    /// This was originally an anonymous function with a,b,c instead of x,y,z
    Vec3[] faces;

    this( ShapeConfig config, float p1, float p2, float p3, float p4 ){

        super(config);

        this.type = SHAPE_TETRA;

        // Vertices and faces of tetra
        this.verts = [ p1, p2, p3, p4 ];
        this.faces = [ new Vec3(0, 1, 2), new Vec3(1, 2, 3), new Vec3(2, 3, 4), new Vec3(4, 0, 1) ];

    }

    void calculateMassInfo ( Shape output ){
        // I guess you could calculate box mass and split it
        // in half for the tetra...
        this.aabb.setFromPoints(this.verts);
        var p = this.aabb.elements;
        var x = p[3] - p[0];
        var y = p[4] - p[1];
        var z = p[5] - p[2];
        var mass = x * y * z * this.density;
        var divid = 1/12;
        output.mass = mass;
        output.inertia.set(
            mass * ( 2*y*2*y + 2*z*2*z ) * divid, 0, 0,
            0, mass * ( 2*x*2*x + 2*z*2*z ) * divid, 0,
            0, 0, mass * ( 2*y*2*y + 2*x*2*x ) * divid
        );

    }

    void updateProxy() {

        this.aabb.setFromPoints(this.verts);
        this.aabb.expandByScalar(AABB_PROX);

        if(this.proxy != null) this.proxy.update();

    }

}