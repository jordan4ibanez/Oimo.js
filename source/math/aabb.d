module math.aabb;

import math.math;
import math.vec3;


/**
 * An axis-aligned bounding box.
 *
 * @author saharan
 * @author lo-th
 */

struct AABB {

    float[] elements = [
        0,0,0,
        0,0,0
    ];

    this(float minX, float maxX, float minY, float maxY, float minZ, float maxZ ){
        this.elements[0] = minX || 0;
        this.elements[1] = minY || 0;
        this.elements[2] = minZ || 0;
        this.elements[3] = maxX || 0;
        this.elements[4] = maxY || 0;
        this.elements[5] = maxZ || 0;
    }

	AABB set(float minX, float maxX, float minY, float maxY, float minZ, float maxZ){
		this.elements[0] = minX;
		this.elements[3] = maxX;
		this.elements[1] = minY;
		this.elements[4] = maxY;
		this.elements[2] = minZ;
		this.elements[5] = maxZ;
		return this;
	}

	bool intersectTest ( AABB aabb ) {
		float[] te = this.elements;
		float[] ue = aabb.elements;
		return te[0] > ue[3] || te[1] > ue[4] || te[2] > ue[5] || te[3] < ue[0] || te[4] < ue[1] || te[5] < ue[2] ? true : false;
	}

	bool intersectTestTwo ( AABB aabb ) {

		float[] te = this.elements;
		float[] ue = aabb.elements;
		return te[0] < ue[0] || te[1] < ue[1] || te[2] < ue[2] || te[3] > ue[3] || te[4] > ue[4] || te[5] > ue[5] ? true : false;

	}

	AABB clone () {

		return AABB().fromArray( this.elements );

	}

	AABB copy ( AABB aabb, float margin ) {

		float m = margin || 0;
		float[] me = aabb.elements;
		this.set( me[ 0 ]-m, me[ 3 ]+m, me[ 1 ]-m, me[ 4 ]+m, me[ 2 ]-m, me[ 5 ]+m );
		return this;

	}

	AABB fromArray ( float[] array ) {
		this.elements = array;
		return this;
	}

	// Set this AABB to the combined AABB of aabb1 and aabb2.

	AABB combine( AABB aabb1, AABB aabb2 ) {

		float[] a = aabb1.elements;
		float[] b = aabb2.elements;
		float[] te = this.elements;

		te[0] = a[0] < b[0] ? a[0] : b[0];
		te[1] = a[1] < b[1] ? a[1] : b[1];
		te[2] = a[2] < b[2] ? a[2] : b[2];

		te[3] = a[3] > b[3] ? a[3] : b[3];
		te[4] = a[4] > b[4] ? a[4] : b[4];
		te[5] = a[5] > b[5] ? a[5] : b[5];

		return this;

	}


	// Get the surface area.

	float surfaceArea () {

		float[] te = this.elements;
		float a = te[3] - te[0];
		float h = te[4] - te[1];
		float d = te[5] - te[2];
		return 2 * (a * (h + d) + h * d );

	}


	// Get whether the AABB intersects with the point or not.

	bool intersectsWithPoint (float x,float y,float z){

		float[] te = this.elements;
		return x >= te[0] && x <= te[3] && y >= te[1] && y <= te[4] && z >= te[2] && z <= te[5];
	}

	/**
	 * Set the AABB from an array
	 * of vertices. From THREE.
	 * @author WestLangley
	 * @author xprogram
	 */

	void setFromPoints( Vec3[] arr){
		this.makeEmpty();
		for(int i = 0; i < arr.length; i++){
			this.expandByPoint(arr[i]);
		}
	}

	void makeEmpty(){
		this.set(-float.infinity, -float.infinity, -float.infinity, float.infinity, float.infinity, float.infinity);
	}

	void expandByPoint(Vec3 pt){
		float[] te = this.elements;
		this.set(
			_Math.min(te[ 0 ], pt.x), _Math.min(te[ 1 ], pt.y), _Math.min(te[ 2 ], pt.z),
			_Math.max(te[ 3 ], pt.x), _Math.max(te[ 4 ], pt.y), _Math.max(te[ 5 ], pt.z)
		);
	}

	void expandByScalar(float s){

		float[] te = this.elements;
		te[0] += -s;
		te[1] += -s;
		te[2] += -s;
		te[3] += s;
		te[4] += s;
		te[5] += s;
        this.elements = te;
	}
}