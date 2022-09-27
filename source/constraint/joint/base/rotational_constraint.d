module constraint.joint.base.rotational_constraint;

/**
* A rotational constraint for various joints.
* @author saharan
*/

public class RotationalConstraint {
    float cfm;
    float i1e00;
    float i1e01;
    float i1e02;
    float i1e10;
    float i1e11;
    float i1e12;
    float i1e20;
    float i1e21;
    float i1e22;
    float i2e00;
    float i2e01;
    float i2e02;
    float i2e10;
    float i2e11;
    float i2e12;
    float i2e20;
    float i2e21;
    float i2e22;
    float motorDenom;
    float invMotorDenom;
    float invDenom;
    float ax;
    float ay;
    float az;
    float a1x;
    float a1y;
    float a1z;
    float a2x;
    float a2y;
    float a2z;
    bool enableLimit=false;
    float lowerLimit;
    float upperLimit;
    float limitVelocity;
    int limitState=0; // -1: at lower, 0: locked, 1: at upper, 2: free
    float enableMotor=false;
    float motorSpeed;
    float maxMotorForce;
    float maxMotorImpulse;

    int limitImpulse=0;
    int motorImpulse=0;
    this ( Joint joint, LimitMotor limitMotor ){

        this.limitMotor=limitMotor;
        this.b1=joint.body1;
        this.b2=joint.body2;
        this.a1=this.b1.angularVelocity;
        this.a2=this.b2.angularVelocity;
        this.i1=this.b1.inverseInertia;
        this.i2=this.b2.inverseInertia;
        
    }

    void preSolve(timeStep,invTimeStep){
        this.ax=this.limitMotor.axis.x;
        this.ay=this.limitMotor.axis.y;
        this.az=this.limitMotor.axis.z;
        this.lowerLimit=this.limitMotor.lowerLimit;
        this.upperLimit=this.limitMotor.upperLimit;
        this.motorSpeed=this.limitMotor.motorSpeed;
        this.maxMotorForce=this.limitMotor.maxMotorForce;
        this.enableMotor=this.maxMotorForce>0;

        float[] ti1 = this.i1.elements;
        float[] ti2 = this.i2.elements;
        this.i1e00=ti1[0];
        this.i1e01=ti1[1];
        this.i1e02=ti1[2];
        this.i1e10=ti1[3];
        this.i1e11=ti1[4];
        this.i1e12=ti1[5];
        this.i1e20=ti1[6];
        this.i1e21=ti1[7];
        this.i1e22=ti1[8];

        this.i2e00=ti2[0];
        this.i2e01=ti2[1];
        this.i2e02=ti2[2];
        this.i2e10=ti2[3];
        this.i2e11=ti2[4];
        this.i2e12=ti2[5];
        this.i2e20=ti2[6];
        this.i2e21=ti2[7];
        this.i2e22=ti2[8];

        float frequency=this.limitMotor.frequency;
        bool enableSpring=frequency>0;
        bool enableLimit=this.lowerLimit<=this.upperLimit;
        float angle=this.limitMotor.angle;
        if(enableLimit){
            if(this.lowerLimit==this.upperLimit){
                if(this.limitState!=0){
                    this.limitState=0;
                    this.limitImpulse=0;
                }
                this.limitVelocity=this.lowerLimit-angle;
            }else if(angle<this.lowerLimit){
                if(this.limitState!=-1){
                    this.limitState=-1;
                    this.limitImpulse=0;
                }
                this.limitVelocity=this.lowerLimit-angle;
            }else if(angle>this.upperLimit){
                if(this.limitState!=1){
                    this.limitState=1;
                    this.limitImpulse=0;
                }
                this.limitVelocity=this.upperLimit-angle;
            }else{
                this.limitState=2;
                this.limitImpulse=0;
                this.limitVelocity=0;
            }
            if(!enableSpring){
                if(this.limitVelocity>0.02)this.limitVelocity-=0.02;
                else if(this.limitVelocity<-0.02)this.limitVelocity+=0.02;
                else this.limitVelocity=0;
            }
        }else{
            this.limitState=2;
            this.limitImpulse=0;
        }
        if(this.enableMotor&&(this.limitState!=0||enableSpring)){
            this.maxMotorImpulse=this.maxMotorForce*timeStep;
        }else{
            this.motorImpulse=0;
            this.maxMotorImpulse=0;
        }

        this.a1x=this.ax*this.i1e00+this.ay*this.i1e01+this.az*this.i1e02;
        this.a1y=this.ax*this.i1e10+this.ay*this.i1e11+this.az*this.i1e12;
        this.a1z=this.ax*this.i1e20+this.ay*this.i1e21+this.az*this.i1e22;
        this.a2x=this.ax*this.i2e00+this.ay*this.i2e01+this.az*this.i2e02;
        this.a2y=this.ax*this.i2e10+this.ay*this.i2e11+this.az*this.i2e12;
        this.a2z=this.ax*this.i2e20+this.ay*this.i2e21+this.az*this.i2e22;
        this.motorDenom=this.ax*(this.a1x+this.a2x)+this.ay*(this.a1y+this.a2y)+this.az*(this.a1z+this.a2z);
        this.invMotorDenom=1/this.motorDenom;

        if(enableSpring&&this.limitState!=2){
            float omega=6.2831853*frequency;
            float k=omega*omega*timeStep;
            vfloatar dmp=invTimeStep/(k+2*this.limitMotor.dampingRatio*omega);
            this.cfm=this.motorDenom*dmp;
            this.limitVelocity*=k*dmp;
        }else{
            this.cfm=0;
            this.limitVelocity*=invTimeStep*0.05;
        }

        this.invDenom=1/(this.motorDenom+this.cfm);
        
        this.limitImpulse*=0.95;
        this.motorImpulse*=0.95;
        float totalImpulse=this.limitImpulse+this.motorImpulse;
        this.a1.x+=totalImpulse*this.a1x;
        this.a1.y+=totalImpulse*this.a1y;
        this.a1.z+=totalImpulse*this.a1z;
        this.a2.x-=totalImpulse*this.a2x;
        this.a2.y-=totalImpulse*this.a2y;
        this.a2.z-=totalImpulse*this.a2z;
    }

    void solve(){

        float rvn=this.ax*(this.a2.x-this.a1.x)+this.ay*(this.a2.y-this.a1.y)+this.az*(this.a2.z-this.a1.z);

        // motor part
        float newMotorImpulse;
        if(this.enableMotor){
            newMotorImpulse=(rvn-this.motorSpeed)*this.invMotorDenom;
            float oldMotorImpulse=this.motorImpulse;
            this.motorImpulse+=newMotorImpulse;
            if(this.motorImpulse>this.maxMotorImpulse)this.motorImpulse=this.maxMotorImpulse;
            else if(this.motorImpulse<-this.maxMotorImpulse)this.motorImpulse=-this.maxMotorImpulse;
            newMotorImpulse=this.motorImpulse-oldMotorImpulse;
            rvn-=newMotorImpulse*this.motorDenom;
        }else newMotorImpulse=0;

        // limit part
        float newLimitImpulse;
        if(this.limitState!=2){
            newLimitImpulse=(rvn-this.limitVelocity-this.limitImpulse*this.cfm)*this.invDenom;
            float oldLimitImpulse=this.limitImpulse;
            this.limitImpulse+=newLimitImpulse;
            if(this.limitImpulse*this.limitState<0)this.limitImpulse=0;
            newLimitImpulse=this.limitImpulse-oldLimitImpulse;
        }else newLimitImpulse=0;

        float totalImpulse=newLimitImpulse+newMotorImpulse;
        this.a1.x+=totalImpulse*this.a1x;
        this.a1.y+=totalImpulse*this.a1y;
        this.a1.z+=totalImpulse*this.a1z;
        this.a2.x-=totalImpulse*this.a2x;
        this.a2.y-=totalImpulse*this.a2y;
        this.a2.z-=totalImpulse*this.a2z;

    }

}