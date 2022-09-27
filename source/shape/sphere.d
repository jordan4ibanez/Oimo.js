module shape.sphere;

import constants;
import shape.shape;
import shape.shape_config;
import math.math;
import shape.mass_info;

/**
 * Sphere shape
 * @author saharan
 * @author lo-th
 */

public class Sphere : Shape {

    float radius;

    this( ShapeConfig config, float radius ) {

        super(config);

        this.type = SHAPE_SPHERE;

        // radius of the shape.
        this.radius = radius;
    }

	float volume () {

		return _Math.PI * this.radius * 1.333333;

	}

    override
	void calculateMassInfo ( MassInfo output ) {

		float mass = this.volume() * this.radius * this.radius * this.density; //1.333 * _Math.PI * this.radius * this.radius * this.radius * this.density;
		output.mass = mass;
		float inertia = mass * this.radius * this.radius * 0.4;
		output.inertia.set( inertia, 0, 0, 0, inertia, 0, 0, 0, inertia );
	}

    override
	void updateProxy () {

		float p = AABB_PROX;

		this.aabb.set(
			this.position.x - this.radius - p, this.position.x + this.radius + p,
			this.position.y - this.radius - p, this.position.y + this.radius + p,
			this.position.z - this.radius - p, this.position.z + this.radius + p
		);

		if ( this.proxy !is null ) this.proxy.update();

	}

}