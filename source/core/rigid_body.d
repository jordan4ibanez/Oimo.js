module core.rigid_body;

import constants;
import shape.mass_info;
import math.math;
import math.mat33;
import math.quat;
import math.vec3;
import shape.shape;
import std.algorithm.mutation: remove;
import std.conv: to;


/**
* The class of rigid body.
* Rigid body has the shape of a single or multiple collision processing,
* I can set the parameters individually.
* @author saharan
* @author lo-th
*/


public class RigidBody {

    Vec3 position;
    float scale = 1;
    float invScale = 1;

    int id = 0;
    string name = "";
    // The maximum number of shapes that can be added to a one rigid.
    //this.MAX_SHAPES = 64;//64;

    // An internal linked list was in here, it made no sense because the rigidbody
    // Already contains an array

    // I represent the kind of rigid body.
    // Please do not change from the outside this variable.
    // If you want to change the type of rigid body, always
    // Please specify the type you want to set the arguments of setupMass method.
    int type;

    MassInfo massInfo;

    Vec3 newPosition;
    bool controlPos = false;
    Quat newOrientation;
    Vec3 newRotation;
    Vec3 currentRotation;
    bool controlRot = false;
    bool controlRotInTime = false;

    Quat quaternion;
    Vec3 pos;

    Quat orientation;

    // Is the translational velocity.
    Vec3 linearVelocity;
    // Is the angular velocity.
    Vec3 angularVelocity;

    //--------------------------------------------
    //  Please do not change from the outside this variables.
    //--------------------------------------------

    // It is a world that rigid body has been added.
    World parent = null;
    ContactLink contactLink = null;
    int numContacts = 0;

    // An array of shapes that are included in the rigid body.
    Shape[] shapes;
    // The number of shapes that are included in the rigid body.
    int numShapes = 0;

    // It is the link array of joint that is connected to the rigid body.
    JointLink[] jointLink;
    // The number of joints that are connected to the rigid body.
    int numJoints = 0;

    // It is the world coordinate of the center of gravity in the sleep just before.
    Vec3 sleepPosition;
    // It is a quaternion that represents the attitude of sleep just before.
    Quat sleepOrientation;
    // I will show this rigid body to determine whether it is a rigid body static.
    bool isStatic = false;
    // I indicates that this rigid body to determine whether it is a rigid body dynamic.
    bool isDynamic = false;

    bool isKinematic = false;

    // It is a rotation matrix representing the orientation.
    Mat33 rotation = new Mat33();

    //--------------------------------------------
    // It will be recalculated automatically from the shape, which is included.
    //--------------------------------------------

    // This is the weight.
    float mass = 0;
    // It is the reciprocal of the mass.
    float inverseMass = 0;
    // It is the inverse of the inertia tensor in the world system.
    Mat33 inverseInertia;
    // It is the inertia tensor in the initial state.
    Mat33 localInertia;
    // It is the inverse of the inertia tensor in the initial state.
    Mat33 inverseLocalInertia;

    Mat33 tmpInertia;


    // I indicates rigid body whether it has been added to the simulation Island.
    bool addedToIsland = false;
    // It shows how to sleep rigid body.
    bool allowSleep = true;
    // This is the time from when the rigid body at rest.
    float sleepTime = 0;
    // I shows rigid body to determine whether it is a sleep state.
    bool sleeping = false;

    this ( Vec3 Position, Quat Rotation ) {

        this.position = Position;
        this.orientation = Rotation;

        this.type = BODY_NULL;

        this.massInfo = new MassInfo();

        this.newPosition = new Vec3();
        this.controlPos = false;
        this.newOrientation = new Quat();
        this.newRotation = new Vec3();
        this.currentRotation = new Vec3();
        this.controlRot = false;
        this.controlRotInTime = false;

        this.quaternion = new Quat();
        this.pos = new Vec3();



        // Is the translational velocity.
        this.linearVelocity = new Vec3();
        // Is the angular velocity.
        this.angularVelocity = new Vec3();
        // It is the world coordinate of the center of gravity in the sleep just before.
        this.sleepPosition = new Vec3();
        // It is a quaternion that represents the attitude of sleep just before.
        this.sleepOrientation = new Quat();

        // It is a rotation matrix representing the orientation.
        this.rotation = new Mat33();

        // It is the inverse of the inertia tensor in the world system.
        this.inverseInertia = new Mat33();
        // It is the inertia tensor in the initial state.
        this.localInertia = new Mat33();
        // It is the inverse of the inertia tensor in the initial state.
        this.inverseLocalInertia = new Mat33();

        this.tmpInertia = new Mat33();
    }


    void setParent ( World world ) {

        this.parent = world;
        this.scale = this.parent.scale;
        this.invScale = this.parent.invScale;
        this.id = this.parent.numRigidBodies;
        if( !this.name ) this.name = to!string(this.id);

    }

    /**
     * I'll add a shape to rigid body.
     * If you add a shape, please call the setupMass method to step up to the start of the next.
     * @param   shape shape to Add
     */
    void addShape(Shape shape){

        if(shape.parent){
			throw new Exception("RigidBody It is not possible that you add a shape which already has an associated body.");
		}

        bool found = false;
        foreach (Shape thisShape; this.shapes) {
            if (thisShape.id == shape.id) {
                found = true;
            }
        }
        if (!found) {
            shape.parent = this;
            this.shapes ~= shape;
            if(this.parent) this.parent.addShape( shape );
            this.numShapes++;
        }
    }
    /**
     * I will delete the shape from the rigid body.
     * If you delete a shape, please call the setupMass method to step up to the start of the next.
     * @param shape {Shape} to delete
     * @return void
     */

    void removeShape( Shape shape){


        if(shape.parent != this)
            return;

        int index = 0;
        bool got = false;
        foreach (Shape gottenShape; this.shapes) {
            if (gottenShape is shape) {
                got = true;
                break;
            }
            index++;
        }
        if (got) {
            this.shapes = this.shapes.remove(index);
            this.numShapes--;
        }                       

    }

    void remove () {
        this.dispose();
    }

    void dispose () {
        this.parent.removeRigidBody( this );
    }

    void checkContact( string name ) {
        this.parent.checkContact( this.name, name );
    }

    /**
     * Calulates mass datas(center of gravity, mass, moment inertia, etc...).
     * If the parameter type is set to BODY_STATIC, the rigid body will be fixed to the space.
     * If the parameter adjustPosition is set to true, the shapes' relative positions and
     * the rigid body's position will be adjusted to the center of gravity.
     * @param type
     * @param adjustPosition
     * @return void
     */
    void setupMass ( int type, bool AdjustPosition ) {

        bool adjustPosition = AdjustPosition || true;

        this.type = type || BODY_STATIC;
        this.isDynamic = this.type == BODY_DYNAMIC;
        this.isStatic = this.type == BODY_STATIC;

        this.mass = 0;
        this.localInertia.set(0,0,0,0,0,0,0,0,0);


        Mat33 tmpM = new Mat33();
        Vec3 tmpV = new Vec3();

        if (this.shapes.length > 0){
            Shape shape = this.shapes[0];
            while (shape !is null) {
                shape = shape.next;

                shape.calculateMassInfo( this.massInfo );
                float shapeMass = this.massInfo.mass;
                tmpV.addScaledVector(shape.relativePosition, shapeMass);
                this.mass += shapeMass;
                this.rotateInertia( shape.relativeRotation, this.massInfo.inertia, tmpM );
                this.localInertia.add( tmpM );

                // add offset inertia
                this.localInertia.addOffset( shapeMass, shape.relativePosition );
            }
        }

        this.inverseMass = 1 / this.mass;
        tmpV.scaleEqual( this.inverseMass );

        if( adjustPosition ){
            this.position.add( tmpV );
            
            if (this.shapes.length > 0){
                Shape shape = this.shapes[0];
                while (shape !is null) {
                    shape = shape.next;
                    shape.relativePosition.subEqual(tmpV);
                }
            }

            // subtract offset inertia
            this.localInertia.subOffset( this.mass, tmpV );

        }

        this.inverseLocalInertia.invert( this.localInertia );

        //}

        if( this.type == BODY_STATIC ){
            this.inverseMass = 0;
            this.inverseLocalInertia.set(0,0,0,0,0,0,0,0,0);
        }

        this.syncShapes();
        this.awake();

    }
    /**
     * Awake the rigid body.
     */
    void awake(){

        if( !this.allowSleep || !this.sleeping ) return;
        this.sleeping = false;
        this.sleepTime = 0;
        // awake connected constraints
        ContactLink cs = this.contactLink;
        while(cs !is null){
            cs.body.sleepTime = 0;
            cs.body.sleeping = false;
            cs = cs.next;
        }
        JointLink js = this.jointLink;
        while(js !is null){
            js.body.sleepTime = 0;
            js.body.sleeping = false;
            js = js.next;
        }
        if (this.shapes.length > 0){
            Shape shape = this.shapes[0];
            while (shape !is null) {
                shape = shape.next;
                shape.updateProxy();
            }
        }

    }
    /**
     * Sleep the rigid body.
     */
    void sleep(){

        if( !this.allowSleep || this.sleeping ) return;

        this.linearVelocity.set(0,0,0);
        this.angularVelocity.set(0,0,0);
        this.sleepPosition.copy( this.position );
        this.sleepOrientation.copy( this.orientation );

        this.sleepTime = 0;
        this.sleeping = true;
        if (this.shapes.length > 0){
            Shape shape = this.shapes[0];
            while (shape !is null) {
                shape = shape.next;
                shape.updateProxy();
            }
        }
    }

    void testWakeUp(){

        if( this.linearVelocity.testZero() || 
        this.angularVelocity.testZero() || 
        this.position.testDiff( this.sleepPosition ) || 
        this.orientation.testDiff( this.sleepOrientation )
        ) this.awake(); // awake the body

    }

    /**
     * Get whether the rigid body has not any connection with others.
     * @return {void}
     */
    bool isLonely () {
        return this.numJoints==0 && this.numContacts==0;
    }

    /**
     * The time integration of the motion of a rigid body, you can update the information such as the shape.
     * This method is invoked automatically when calling the step of the World,
     * There is no need to call from outside usually.
     * @param  timeStep time
     * @return {void}
     */

    void updatePosition ( float timeStep ) {
        switch(this.type){
            case BODY_STATIC:
                this.linearVelocity.set(0,0,0);
                this.angularVelocity.set(0,0,0);

                // ONLY FOR TEST
                if(this.controlPos){
                    this.position.copy(this.newPosition);
                    this.controlPos = false;
                }
                if(this.controlRot){
                    this.orientation.copy(this.newOrientation);
                    this.controlRot = false;
                }
                /*this.linearVelocity.x=0;
                this.linearVelocity.y=0;
                this.linearVelocity.z=0;
                this.angularVelocity.x=0;
                this.angularVelocity.y=0;
                this.angularVelocity.z=0;*/
            break;
            case BODY_DYNAMIC:

                if( this.isKinematic ){

                    this.linearVelocity.set(0,0,0);
                    this.angularVelocity.set(0,0,0);

                }

                if(this.controlPos){

                    this.linearVelocity.subVectors( this.newPosition, this.position ).multiplyScalar(1/timeStep);
                    this.controlPos = false;

                }
                if(this.controlRot){

                    this.angularVelocity.copy( this.getAxis() );
                    this.orientation.copy( this.newOrientation );
                    this.controlRot = false;

                }

                this.position.addScaledVector(this.linearVelocity, timeStep);
                this.orientation.addTime(this.angularVelocity, timeStep);

            break;
            default: throw new Exception("RigidBody Invalid type.");
        }

        this.syncShapes();

    }

    Vec3 getAxis () {

        return new Vec3( 0,1,0 ).applyMatrix3( this.inverseLocalInertia, true ).normalize();

    }

    void rotateInertia ( Mat33 rot, Mat33 inertia, Mat33 output ) {

        this.tmpInertia.multiplyMatrices( rot, inertia, false );
        output.multiplyMatrices( this.tmpInertia, rot, true );

    }

    void syncShapes () {

        this.rotation.setQuat( this.orientation );
        this.rotateInertia( this.rotation, this.inverseLocalInertia, this.inverseInertia );
        
        if (this.shapes.length > 0){
            Shape shape = this.shapes[0];
            while (shape !is null) {
                shape = shape.next;

                shape.position.copy( shape.relativePosition ).applyMatrix3( this.rotation, true ).add( this.position );
                // add by QuaziKb
                shape.rotation.multiplyMatrices( this.rotation, shape.relativeRotation, false);
                shape.updateProxy();
            }
        }
    }


    //---------------------------------------------
    // APPLY IMPULSE FORCE
    //---------------------------------------------

    void applyImpulse(Vec3 position, Vec3 force){
        this.linearVelocity.addScaledVector(force, this.inverseMass);
        Vec3 rel = new Vec3().copy( position ).sub( this.position ).cross( force ).applyMatrix3( this.inverseInertia, true );
        this.angularVelocity.add( rel );
    }


    //---------------------------------------------
    // SET DYNAMIQUE POSITION AND ROTATION
    //---------------------------------------------

    void setPosition(Vec3 pos){
        this.newPosition.copy( pos ).multiplyScalar( this.invScale );
        this.controlPos = true;
        if( !this.isKinematic ) this.isKinematic = true;
    }

    void setQuaternion(Quat q){
        this.newOrientation.set(q.x, q.y, q.z, q.w);
        this.controlRot = true;
        if( !this.isKinematic ) this.isKinematic = true;
    }

    void setRotation ( Vec3 rot ) {

        this.newOrientation = new Quat().setFromEuler( rot.x * _Math.degtorad, rot.y * _Math.degtorad, rot.z * _Math.degtorad );//this.rotationVectToQuad( rot );
        this.controlRot = true;

    }

    //---------------------------------------------
    // RESET DYNAMIQUE POSITION AND ROTATION
    //---------------------------------------------

    void resetPosition(float x, float y, float z){

        this.linearVelocity.set( 0, 0, 0 );
        this.angularVelocity.set( 0, 0, 0 );
        this.position.set( x, y, z ).multiplyScalar( this.invScale );
        //this.position.set( x*OIMO.WorldScale.invScale, y*OIMO.WorldScale.invScale, z*OIMO.WorldScale.invScale );
        this.awake();
    }

    void resetQuaternion( Quat q ){

        this.angularVelocity.set(0,0,0);
        this.orientation = new Quat( q.x, q.y, q.z, q.w );
        this.awake();

    }

    void resetRotation(float x, float y, float z){

        this.angularVelocity.set(0,0,0);
        this.orientation = new Quat().setFromEuler( x * _Math.degtorad, y * _Math.degtorad,  z * _Math.degtorad );//this.rotationVectToQuad( new Vec3(x,y,z) );
        this.awake();

    }

    //---------------------------------------------
    // GET POSITION AND ROTATION
    //---------------------------------------------

    Vec3 getPosition () {

        return this.pos;

    }

    Quat getQuaternion () {

        return this.quaternion;

    }

}