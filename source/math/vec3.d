module math.vec3;

import math.math;
import math.quat;
import math.mat33;

class Vec3 {
    float x = 0;
    float y = 0;
    float z = 0;

    this(){}

    this( float x, float y, float z ) {
        this.x = x || 0;
        this.y = y || 0;
        this.z = z || 0;
    }


    Vec3 set( float x, float y, float z ) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    Vec3 add ( Vec3 a ) {
        this.x += a.x;
        this.y += a.y;
        this.z += a.z;
        return this;
    }

    Vec3 addVectors ( Vec3 a, Vec3 b ) {
        this.x = a.x + b.x;
        this.y = a.y + b.y;
        this.z = a.z + b.z;
        return this;
    }

    Vec3 addEqual ( Vec3 v ) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }

    Vec3 sub ( Vec3 a) {
        this.x -= a.x;
        this.y -= a.y;
        this.z -= a.z;
        return this;
    }

    Vec3 subVectors ( Vec3 a, Vec3 b ) {
        this.x = a.x - b.x;
        this.y = a.y - b.y;
        this.z = a.z - b.z;
        return this;

    }

    Vec3 subEqual ( Vec3 v ) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;

    }

    Vec3 scale ( Vec3 v, float s ) {
        this.x = v.x * s;
        this.y = v.y * s;
        this.z = v.z * s;
        return this;
    }

    Vec3 scaleEqual ( float s ) {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        return this;
    }

    Vec3 multiply( Vec3 v ){
        this.x *= v.x;
        this.y *= v.y;
        this.z *= v.z;
        return this;
    }

    /*scaleV( v ){

        this.x *= v.x;
        this.y *= v.y;
        this.z *= v.z;
        return this;

    },

    scaleVectorEqual( v ){

        this.x *= v.x;
        this.y *= v.y;
        this.z *= v.z;
        return this;

    },*/

    Vec3 addScaledVector ( Vec3 v, float s ) {
        this.x += v.x * s;
        this.y += v.y * s;
        this.z += v.z * s;
        return this;
    }

    Vec3 subScaledVector ( Vec3 v, float s ) {
        this.x -= v.x * s;
        this.y -= v.y * s;
        this.z -= v.z * s;
        return this;
    }

    /*addTime ( v, t ) {

        this.x += v.x * t;
        this.y += v.y * t;
        this.z += v.z * t;
        return this;

    },
    
    addScale ( v, s ) {

        this.x += v.x * s;
        this.y += v.y * s;
        this.z += v.z * s;
        return this;

    },

    subScale ( v, s ) {

        this.x -= v.x * s;
        this.y -= v.y * s;
        this.z -= v.z * s;
        return this;

    },*/

    Vec3 cross( Vec3 a ) {

        float x1 = this.x;
        float y1 = this.y;
        float z1 = this.z;

        this.x = y1 * a.z - z1 * a.y;
        this.y = z1 * a.x - x1 * a.z;
        this.z = x1 * a.y - y1 * a.x;

        return this;
    }

    Vec3 crossVectors ( Vec3 a, Vec3 b ) {

        float ax = a.x;
        float ay = a.y;
        float az = a.z;
        float bx = b.x;
        float by = b.y;
        float bz = b.z;

        this.x = ay * bz - az * by;
        this.y = az * bx - ax * bz;
        this.z = ax * by - ay * bx;

        return this;
    }

    Vec3 tangent ( Vec3 a ) {

        float ax = a.x, ay = a.y, az = a.z;

        this.x = ay * ax - az * az;
        this.y = - az * ay - ax * ax;
        this.z = ax * az + ay * ay;

        return this;
    }    

    Vec3 invert ( Vec3 v ) {
        this.x=-v.x;
        this.y=-v.y;
        this.z=-v.z;
        return this;
    }

    Vec3 negate () {

        this.x = - this.x;
        this.y = - this.y;
        this.z = - this.z;

        return this;

    }

    float dot ( Vec3 v ) {

        return this.x * v.x + this.y * v.y + this.z * v.z;
    }

    float addition () {

        return this.x + this.y + this.z;

    }

    float lengthSq () {

        return this.x * this.x + this.y * this.y + this.z * this.z;

    }

    float length () {
        return _Math.sqrt( this.x * this.x + this.y * this.y + this.z * this.z );
    }

    Vec3 copy ( Vec3 v ) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
        return this;
    }

    /*mul( b, a, m ){

        return this.mulMat( m, a ).add( b );

    },

    mulMat( m, a ){

        var e = m.elements;
        var x = a.x, y = a.y, z = a.z;

        this.x = e[ 0 ] * x + e[ 1 ] * y + e[ 2 ] * z;
        this.y = e[ 3 ] * x + e[ 4 ] * y + e[ 5 ] * z;
        this.z = e[ 6 ] * x + e[ 7 ] * y + e[ 8 ] * z;
        return this;

    },*/

    Vec3 applyMatrix3 ( Mat33 m, bool transpose ) {

        //if( transpose ) m = m.clone().transpose();
        float x1 = this.x, y1 = this.y, z1 = this.z;
        float[] e = m.elements;

        if( transpose ){
            
            this.x = e[ 0 ] * x1 + e[ 1 ] * y1 + e[ 2 ] * z1;
            this.y = e[ 3 ] * x1 + e[ 4 ] * y1 + e[ 5 ] * z1;
            this.z = e[ 6 ] * x1 + e[ 7 ] * y1 + e[ 8 ] * z1;

        } else {
    
            this.x = e[ 0 ] * x1 + e[ 3 ] * y1 + e[ 6 ] * z1;
            this.y = e[ 1 ] * x1 + e[ 4 ] * y1 + e[ 7 ] * z1;
            this.z = e[ 2 ] * x1 + e[ 5 ] * y1 + e[ 8 ] * z1;
        }

        return this;

    }

    Vec3 applyQuaternion ( Quat q ) {

        float x1 = this.x;
        float y1 = this.y;
        float z1 = this.z;

        float qx = q.x;
        float qy = q.y;
        float qz = q.z;
        float qw = q.w;

        // calculate quat * vector

        float ix =  qw * x1 + qy * z1 - qz * y1;
        float iy =  qw * y1 + qz * x1 - qx * z1;
        float iz =  qw * z1 + qx * y1 - qy * x1;
        float iw = - qx * x1 - qy * y1 - qz * z1;

        // calculate result * inverse quat

        this.x = ix * qw + iw * - qx + iy * - qz - iz * - qy;
        this.y = iy * qw + iw * - qy + iz * - qx - ix * - qz;
        this.z = iz * qw + iw * - qz + ix * - qy - iy * - qx;

        return this;

    }

    bool testZero () {

        if(this.x != 0 || this.y != 0 || this.z != 0) return true;
        else return false;

    }

    bool testDiff( Vec3 v ){

        return this.equals( v ) ? false : true;

    }

    bool equals ( Vec3 v ) {

        return v.x == this.x && v.y == this.y && v.z == this.z;

    }

    Vec3 clone () {

        return new Vec3( this.x, this.y, this.z );

    }


    Vec3 multiplyScalar ( float scalar ) {

        if ( scalar != float.infinity ) {
            this.x *= scalar;
            this.y *= scalar;
            this.z *= scalar;
        } else {
            this.x = 0;
            this.y = 0;
            this.z = 0;
        }

        return this;
    }

    Vec3 divideScalar ( float scalar ) {

        return this.multiplyScalar( 1 / scalar );

    }

    Vec3 normalize () {

        return this.divideScalar( this.length() );

    }

    void toArray ( float[] array, int offset ) {
        array[ offset ] = this.x;
        array[ offset + 1 ] = this.y;
        array[ offset + 2 ] = this.z;
    }

    Vec3 fromArray( float[] array, int offset ){
        this.x = array[ offset ];
        this.y = array[ offset + 1 ];
        this.z = array[ offset + 2 ];
        return this;
    }
}