module math.quat;

import math.math;
import math.vec3;
import math.mat33;
import math.aabb;

class Quat {

    float x = 0;
    float y = 0;
    float z = 0;
    float w = 1;

    float _x = 0;
    float _y = 0;
    float _z = 0;
    float _w = 1;

    this(){}

    this( float x, float y, float z, float w ){
        this.x = x || 0;
        this.y = y || 0;
        this.z = z || 0;
        this.w = w || 1;
    }

    Quat set ( float x, float y, float z, float w ) {
        
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;

        return this;

    }

    Quat addTime( Vec3 v, float t ){

        float ax = v.x;
        float ay = v.y;
        float az = v.z;
        float qw = this.w;
        float qx = this.x;
        float qy = this.y;
        float qz = this.z;
        t *= 0.5;    
        this.x += t * (  ax*qw + ay*qz - az*qy );
        this.y += t * (  ay*qw + az*qx - ax*qz );
        this.z += t * (  az*qw + ax*qy - ay*qx );
        this.w += t * ( -ax*qx - ay*qy - az*qz );
        this.normalize();
        return this;

    }

    /*mul( q1, q2 ){

        var ax = q1.x, ay = q1.y, az = q1.z, as = q1.w,
        bx = q2.x, by = q2.y, bz = q2.z, bs = q2.w;
        this.x = ax * bs + as * bx + ay * bz - az * by;
        this.y = ay * bs + as * by + az * bx - ax * bz;
        this.z = az * bs + as * bz + ax * by - ay * bx;
        this.w = as * bs - ax * bx - ay * by - az * bz;
        return this;

    }*/

    Quat multiply ( Quat q ) {

        return this.multiplyQuaternions( this, q );

    }

    Quat multiplyQuaternions ( Quat a, Quat b ) {

        float qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
        float qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;

        this.x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
        this.y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
        this.z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
        this.w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;
        return this;

    }

    Quat setFromUnitVectors( Vec3 v1, Vec3 v2 ) {

        Vec3 vx = new Vec3();
        float r = v1.dot( v2 ) + 1;

        if ( r < _Math.EPZ2 ) {

            r = 0;
            if ( _Math.abs( v1.x ) > _Math.abs( v1.z ) ) vx.set( - v1.y, v1.x, 0 );
            else vx.set( 0, - v1.z, v1.y );

        } else {

            vx.crossVectors( v1, v2 );

        }

        this._x = vx.x;
        this._y = vx.y;
        this._z = vx.z;
        this._w = r;

        return this.normalize();

    }

    Quat arc( Vec3 v1, Vec3 v2 ){

        float  x1 = v1.x;
        float  y1 = v1.y;
        float  z1 = v1.z;
        float  x2 = v2.x;
        float  y2 = v2.y;
        float  z2 = v2.z;
        float  d = x1*x2 + y1*y2 + z1*z2;
        if( d==-1 ){
            x2 = y1*x1 - z1*z1;
            y2 = -z1*y1 - x1*x1;
            z2 = x1*z1 + y1*y1;
            d = 1 / _Math.sqrt( x2*x2 + y2*y2 + z2*z2 );
            this.w = 0;
            this.x = x2*d;
            this.y = y2*d;
            this.z = z2*d;
            return this;
        }
        float  cx = y1*z2 - z1*y2;
        float  cy = z1*x2 - x1*z2;
        float  cz = x1*y2 - y1*x2;
        this.w = _Math.sqrt( ( 1 + d) * 0.5 );
        d = 0.5 / this.w;
        this.x = cx * d;
        this.y = cy * d;
        this.z = cz * d;
        return this;

    }

    Quat normalize(){

        float l = this.length();
        if ( l == 0 ) {
            this.set( 0, 0, 0, 1 );
        } else {
            l = 1 / l;
            this.x = this.x * l;
            this.y = this.y * l;
            this.z = this.z * l;
            this.w = this.w * l;
        }
        return this;

    }

    Quat inverse () {

        return this.conjugate().normalize();

    }

    Quat invert ( Quat q ) {

        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
        this.conjugate().normalize();
        return this;

    }

    Quat conjugate () {

        this.x *= - 1;
        this.y *= - 1;
        this.z *= - 1;
        return this;

    }

    float length(){

        return _Math.sqrt( this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w  );

    }

    float lengthSq () {

        return this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w;

    }
    
    Quat copy( Quat q ){
        
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
        return this;

    }

    Quat clone( Quat q ){

        return new Quat( this.x, this.y, this.z, this.w );

    }

    bool testDiff ( Quat q ) {

        return this.equals( q ) ? false : true;

    }

    bool equals ( Quat q ) {

        return this.x == q.x && this.y == q.y && this.z == q.z && this.w == q.w;

    }

    Quat setFromEuler ( float x, float y, float z ){

        float  c1 = _Math.cos( x * 0.5 );
        float  c2 = _Math.cos( y * 0.5 );
        float  c3 = _Math.cos( z * 0.5 );
        float  s1 = _Math.sin( x * 0.5 );
        float  s2 = _Math.sin( y * 0.5 );
        float  s3 = _Math.sin( z * 0.5 );

        // XYZ
        this.x = s1 * c2 * c3 + c1 * s2 * s3;
        this.y = c1 * s2 * c3 - s1 * c2 * s3;
        this.z = c1 * c2 * s3 + s1 * s2 * c3;
        this.w = c1 * c2 * c3 - s1 * s2 * s3;

        return this;

    }
    
    Quat setFromAxis ( Vec3 axis, float rad ) {

        axis.normalize();
        rad = rad * 0.5;
        float s = _Math.sin( rad );
        this.x = s * axis.x;
        this.y = s * axis.y;
        this.z = s * axis.z;
        this.w = _Math.cos( rad );
        return this;

    }

    Quat setFromMat33 ( Mat33 m ) {

        float trace = m.elements[0] + m.elements[4] + m.elements[8];
        float s;

        if ( trace > 0 ) {

            s = _Math.sqrt( trace + 1.0 );
            this.w = 0.5 / s;
            s = 0.5 / s;
            this.x = ( m.elements[5] - m.elements[7] ) * s;
            this.y = ( m.elements[6] - m.elements[2] ) * s;
            this.z = ( m.elements[1] - m.elements[3] ) * s;

        } else {

            float[] out_ = new float[16];
            int i = 0;
            if ( m.elements[4] > m.elements[0] ) i = 1;
            if ( m.elements[8] > m.elements[i*3+i] ) i = 2;

            int j = (i+1)%3;
            int k = (i+2)%3;
            
            s = _Math.sqrt( m.elements[i*3+i] - m.elements[j*3+j] - m.elements[k*3+k] + 1.0 );
            // This used to be fRoot?
            out_[i] = 0.5 * 1;
            s = 0.5 / 1;
            this.w = ( m.elements[j*3+k] - m.elements[k*3+j] ) * s;
            out_[j] = ( m.elements[j*3+i] + m.elements[i*3+j] ) * s;
            out_[k] = ( m.elements[k*3+i] + m.elements[i*3+k] ) * s;

            this.x = out_[1];
            this.y = out_[2];
            this.z = out_[3];

        }

        return this;

    }

    void toArray ( float[] array, int offset ) {

        offset = offset || 0;

        array[ offset ] = this.x;
        array[ offset + 1 ] = this.y;
        array[ offset + 2 ] = this.z;
        array[ offset + 3 ] = this.w;

    }

    Quat fromArray( float[] array, int offset ){

        offset = offset || 0;
        this.set( array[ offset ], array[ offset + 1 ], array[ offset + 2 ], array[ offset + 3 ] );
        return this;

    }
}