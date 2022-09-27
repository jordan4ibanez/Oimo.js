module math.math;

import IMath = std.math;
import std.random;
import std.uuid;
import Comparison = std.algorithm.comparison;
import raylib.raylib_types;

private static struct LMath {
    public static:
    float random() {
        Random rand = Random(unpredictableSeed());
        return uniform(0.0, 1.0, rand);
    }

    float degtorad = 0.0174532925199432957;
    float radtodeg = 57.295779513082320876;
    float PI       = 3.141592653589793;
    float TwoPI    = 6.283185307179586;
    float PI90     = 1.570796326794896;
    float PI270    = 4.712388980384689;

    float INF      = float.infinity;
    float EPZ      = 0.00001;
    float EPZ2     = 0.000001;

    alias sqrt   = IMath.sqrt;
    alias abs    = IMath.abs;
    alias floor  = IMath.floor;
    alias cos    = IMath.cos;
    alias sin    = IMath.sin;
    alias acos   = IMath.acos;
    alias asin   = IMath.asin;
    alias atan2  = IMath.atan2;
    alias round  = IMath.round;
    alias pow    = IMath.pow;
    alias max    = Comparison.max;
    alias min    = Comparison.min;


    float lerp( float x, float y, float t ) { 
        return ( 1 - t ) * x + t * y;
    }

    int randInt ( int low, int high ) {
        return low + cast(int)(IMath.floor( random() * cast(float)( high - low + 1 ) ));
    }

    float rand( float low, float high ) { 
        return low + random() * ( high - low ); 
    }

    string generateUUID() {
        // http://www.broofa.com/Tools/Math.uuid.htm
        UUID uuid = randomUUID();
        return uuid.toString();
    }

    /// Casts a float to an int rounded down
    int intify( float x ) { 
        return cast(int)IMath.floor(x); 
    }

    /// Trims the precision of a float
    float fix( float x, int n ) { 
        int exponent = IMath.pow(10, n);
        float i = x * cast(float)(exponent);
        int d = cast(int)IMath.floor(i);
        float e = cast(float)d / cast(float)(exponent);
        return e;
    }

    float clamp( float value, float min, float max ) { 
        return Comparison.clamp(value,min,max);
    }

    //clamp: function ( x, a, b ) { return ( x < a ) ? a : ( ( x > b ) ? b : x ); },
    float distance( float[] p1, float[] p2 ){

        float xd = p2[0]-p1[0];
        float yd = p2[1]-p1[1];
        float zd = p2[2]-p1[2];
        return IMath.sqrt(xd*xd + yd*yd + zd*zd);
    }

    float distance( Vector3 p1, Vector3 p2 ){
        float xd = p2.x - p1.x;
        float yd = p2.y - p1.y;
        float zd = p2.z - p1.z;
        return IMath.sqrt(xd*xd + yd*yd + zd*zd);
    }

    /*unwrapDegrees: function ( r ) {

        r = r % 360;
        if (r > 180) r -= 360;
        if (r < -180) r += 360;
        return r;

    },

    unwrapRadian: function( r ){

        r = r % _IMath.TwoPI;
        if (r > _IMath.PI) r -= _IMath.TwoPI;
        if (r < -_IMath.PI) r += _IMath.TwoPI;
        return r;

    },*/

    float acosClamp( float cos ) {
        if ( cos > 1 ) return 0;
        else if ( cos < -1 ) return IMath.PI;
        else return IMath.acos(cos);
    }

    float distanceVector( Vector3 v1, Vector3 v2 ){
        float xd = v1.x - v2.x;
        float yd = v1.y - v2.y;
        float zd = v1.z - v2.z;
        return xd * xd + yd * yd + zd * zd;
    }

    float dotVectors( Vector3 a, Vector3 b ) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
}

alias _Math = LMath;