module shape.tetra;

import constants;
import shape.shape;
import shape.shape_config;
import math.vec3;
import shape.mass_info;

/**
 * A tetra shape.
 * @author xprogram
 */

public class Tetra : Shape {

    Vec3[] verts;
    /// This was originally an anonymous function with a,b,c instead of x,y,z
    Vec3[] faces;

    this( ShapeConfig config, Vec3 p1, Vec3 p2, Vec3 p3, Vec3 p4 ){

        super(config);

        this.type = SHAPE_TETRA;

        // Vertices and faces of tetra
        this.verts = [ p1, p2, p3, p4 ];
        this.faces = [ new Vec3(0, 1, 2), new Vec3(1, 2, 3), new Vec3(2, 3, 4), new Vec3(4, 0, 1) ];

    }
    
    override
    void calculateMassInfo ( MassInfo output ){
        // I guess you could calculate box mass and split it
        // in half for the tetra...
        this.aabb.setFromPoints(this.verts);
        float[] p = this.aabb.elements;
        float x = p[3] - p[0];
        float y = p[4] - p[1];
        float z = p[5] - p[2];
        float mass = x * y * z * this.density;
        float divid = 1/12;
        output.mass = mass;
        output.inertia.set(
            mass * ( 2*y*2*y + 2*z*2*z ) * divid, 0, 0,
            0, mass * ( 2*x*2*x + 2*z*2*z ) * divid, 0,
            0, 0, mass * ( 2*y*2*y + 2*x*2*x ) * divid
        );

    }
    
    override
    void updateProxy() {

        this.aabb.setFromPoints(this.verts);
        this.aabb.expandByScalar(AABB_PROX);

        if(this.proxy !is null) this.proxy.update();

    }

}