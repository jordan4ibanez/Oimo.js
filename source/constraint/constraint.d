module constraint.constraint;

import core.rigid_body;
import math.vec3;

/**
 * The base class of all type of the constraints.
 *
 * @author saharan
 * @author lo-th
 */

public class Constraint {

    World parent;
    RigidBody body1;
    RigidBody body2;
    bool addedToIsland;
    
    this(){

        // parent world of the constraint.
        this.parent = null;

        // first body of the constraint.
        this.body1 = null;

        // second body of the constraint.
        this.body2 = null;

        // Internal
        this.addedToIsland = false;
        
    }

    // Prepare for solving the constraint
    void preSolve( float timeStep, float invTimeStep ){

        throw new Exception("Constraint Inheritance error.");

    }

    // Solve the constraint. This is usually called iteratively.
    void solve(){

        throw new Exception("Constraint Inheritance error.");

    }

    // Do the post-processing.
    void postSolve(){

        throw new Exception("Constraint", "Inheritance error.");

    }
}