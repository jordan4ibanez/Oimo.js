module constraint.joint.ball_and_joint_socket;

import { JOINT_BALL_AND_SOCKET } from '../../constants';
import { Joint } from './Joint';
import { LinearConstraint } from './base/LinearConstraint';

import constants;

/**
 * A ball-and-socket joint limits relative translation on two anchor points on rigid bodies.
 *
 * @author saharan
 * @author lo-th
 */

public class BallAndSocketJoint {
function BallAndSocketJoint ( config ){

    Joint.call( this, config );

    this.type = JOINT_BALL_AND_SOCKET;
    
    this.lc = new LinearConstraint( this );

};

    preSolve ( timeStep, invTimeStep ) {

        this.updateAnchorPoints();

        // preSolve

        this.lc.preSolve( timeStep, invTimeStep );

    },

    solve () {

        this.lc.solve();

    },

    postSolve () {

    }

}

export { BallAndSocketJoint };