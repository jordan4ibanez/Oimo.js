module shape.sphere;

import constants;
import shape.shape;
import shape.shape_config;
import math.math;

/**
 * Sphere shape
 * @author saharan
 * @author lo-th
 */

public class Sphere : Shape {

    float radius;

    this( ShapeConfig config, float radius ) {

        super(config);

        Shape.call( this, config );

        this.type = SHAPE_SPHERE;

        // radius of the shape.
        this.radius = radius;
    }

	float volume () {

		return _Math.PI * this.radius * 1.333333;

	}

	void calculateMassInfo ( Shape output ) {

		var mass = this.volume() * this.radius * this.radius * this.density; //1.333 * _Math.PI * this.radius * this.radius * this.radius * this.density;
		output.mass = mass;
		var inertia = mass * this.radius * this.radius * 0.4;
		output.inertia.set( inertia, 0, 0, 0, inertia, 0, 0, 0, inertia );
	}

	void updateProxy () {

		var p = AABB_PROX;

		this.aabb.set(
			this.position.x - this.radius - p, this.position.x + this.radius + p,
			this.position.y - this.radius - p, this.position.y + this.radius + p,
			this.position.z - this.radius - p, this.position.z + this.radius + p
		);

		if ( this.proxy != null ) this.proxy.update();

	}

}