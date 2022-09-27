module constraint.joint.base.translational_3_constraints;

/**
* A three-axis translational constraint for various joints.
* @author saharan
*/
public class Translational3Constraint {
    float m1;
    float m2;
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
    float ax1;
    float ay1;
    float az1;
    float ax2;
    float ay2;
    float az2;
    float ax3;
    float ay3;
    float az3;
    float r1x;
    float r1y;
    float r1z;
    float r2x;
    float r2y;
    float r2z;
    float t1x1;// jacobians
    float t1y1;
    float t1z1;
    float t2x1;
    float t2y1;
    float t2z1;
    float l1x1;
    float l1y1;
    float l1z1;
    float l2x1;
    float l2y1;
    float l2z1;
    float a1x1;
    float a1y1;
    float a1z1;
    float a2x1;
    float a2y1;
    float a2z1;
    float t1x2;
    float t1y2;
    float t1z2;
    float t2x2;
    float t2y2;
    float t2z2;
    float l1x2;
    float l1y2;
    float l1z2;
    float l2x2;
    float l2y2;
    float l2z2;
    float a1x2;
    float a1y2;
    float a1z2;
    float a2x2;
    float a2y2;
    float a2z2;
    float t1x3;
    float t1y3;
    float t1z3;
    float t2x3;
    float t2y3;
    float t2z3;
    float l1x3;
    float l1y3;
    float l1z3;
    float l2x3;
    float l2y3;
    float l2z3;
    float a1x3;
    float a1y3;
    float a1z3;
    float a2x3;
    float a2y3;
    float a2z3;
    float lowerLimit1;
    float upperLimit1;
    float limitVelocity1;
    int limitState1=0; // -1: at lower, 0: locked, 1: at upper, 2: unlimited
    bool enableMotor1=false;
    float motorSpeed1;
    float maxMotorForce1;
    float maxMotorImpulse1;
    float lowerLimit2;
    float upperLimit2;
    float limitVelocity2;
    int limitState2=0; // -1: at lower, 0: locked, 1: at upper, 2: unlimited
    bool enableMotor2=false;
    float motorSpeed2;
    float maxMotorForce2;
    float maxMotorImpulse2;
    float lowerLimit3;
    float upperLimit3;
    float limitVelocity3;
    int limitState3=0; // -1: at lower, 0: locked, 1: at upper, 2: unlimited
    bool enableMotor3=false;
    float motorSpeed3;
    float maxMotorForce3;
    float maxMotorImpulse3;
    float k00; // K = J*M*JT
    float k01;
    float k02;
    float k10;
    float k11;
    float k12;
    float k20;
    float k21;
    float k22;
    float kv00; // diagonals without CFMs
    float kv11;
    float kv22;
    float dv00; // ...inverted
    float dv11;
    float dv22;
    float d00; // K^-1
    float d01;
    float d02;
    float d10;
    float d11;
    float d12;
    float d20;
    float d21;
    float d22;

    float limitImpulse1=0;
    float motorImpulse1=0;
    float limitImpulse2=0;
    float motorImpulse2=0;
    float limitImpulse3=0;
    float motorImpulse3=0;
    float cfm1=0;// Constraint Force Mixing
    float cfm2=0;
    float cfm3=0;
    float weight=-1;

    this ( Joint joint, LimitMotor limitMotor1, LimitMotor limitMotor2, LimitMotor limitMotor3){
        this.limitMotor1=limitMotor1;
        this.limitMotor2=limitMotor2;
        this.limitMotor3=limitMotor3;
        this.b1=joint.body1;
        this.b2=joint.body2;
        this.p1=joint.anchorPoint1;
        this.p2=joint.anchorPoint2;
        this.r1=joint.relativeAnchorPoint1;
        this.r2=joint.relativeAnchorPoint2;
        this.l1=this.b1.linearVelocity;
        this.l2=this.b2.linearVelocity;
        this.a1=this.b1.angularVelocity;
        this.a2=this.b2.angularVelocity;
        this.i1=this.b1.inverseInertia;
        this.i2=this.b2.inverseInertia;
    }

    void preSolve(float timeStep,float invTimeStep){
        this.ax1=this.limitMotor1.axis.x;
        this.ay1=this.limitMotor1.axis.y;
        this.az1=this.limitMotor1.axis.z;
        this.ax2=this.limitMotor2.axis.x;
        this.ay2=this.limitMotor2.axis.y;
        this.az2=this.limitMotor2.axis.z;
        this.ax3=this.limitMotor3.axis.x;
        this.ay3=this.limitMotor3.axis.y;
        this.az3=this.limitMotor3.axis.z;
        this.lowerLimit1=this.limitMotor1.lowerLimit;
        this.upperLimit1=this.limitMotor1.upperLimit;
        this.motorSpeed1=this.limitMotor1.motorSpeed;
        this.maxMotorForce1=this.limitMotor1.maxMotorForce;
        this.enableMotor1=this.maxMotorForce1>0;
        this.lowerLimit2=this.limitMotor2.lowerLimit;
        this.upperLimit2=this.limitMotor2.upperLimit;
        this.motorSpeed2=this.limitMotor2.motorSpeed;
        this.maxMotorForce2=this.limitMotor2.maxMotorForce;
        this.enableMotor2=this.maxMotorForce2>0;
        this.lowerLimit3=this.limitMotor3.lowerLimit;
        this.upperLimit3=this.limitMotor3.upperLimit;
        this.motorSpeed3=this.limitMotor3.motorSpeed;
        this.maxMotorForce3=this.limitMotor3.maxMotorForce;
        this.enableMotor3=this.maxMotorForce3>0;
        this.m1=this.b1.inverseMass;
        this.m2=this.b2.inverseMass;

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

        float dx=this.p2.x-this.p1.x;
        float dy=this.p2.y-this.p1.y;
        float dz=this.p2.z-this.p1.z;
        float d1=dx*this.ax1+dy*this.ay1+dz*this.az1;
        float d2=dx*this.ax2+dy*this.ay2+dz*this.az2;
        float d3=dx*this.ax3+dy*this.ay3+dz*this.az3;
        float frequency1=this.limitMotor1.frequency;
        float frequency2=this.limitMotor2.frequency;
        float frequency3=this.limitMotor3.frequency;
        bool enableSpring1=frequency1>0;
        bool enableSpring2=frequency2>0;
        bool enableSpring3=frequency3>0;
        bool enableLimit1=this.lowerLimit1<=this.upperLimit1;
        bool enableLimit2=this.lowerLimit2<=this.upperLimit2;
        bool enableLimit3=this.lowerLimit3<=this.upperLimit3;

        // for stability
        if(enableSpring1&&d1>20||d1<-20){
            enableSpring1=false;
        }
        if(enableSpring2&&d2>20||d2<-20){
            enableSpring2=false;
        }
        if(enableSpring3&&d3>20||d3<-20){
            enableSpring3=false;
        }

        if(enableLimit1){
            if(this.lowerLimit1==this.upperLimit1){
                if(this.limitState1!=0){
                    this.limitState1=0;
                    this.limitImpulse1=0;
                }
                this.limitVelocity1=this.lowerLimit1-d1;
                if(!enableSpring1)d1=this.lowerLimit1;
            }else if(d1<this.lowerLimit1){
                if(this.limitState1!=-1){
                    this.limitState1=-1;
                    this.limitImpulse1=0;
                }
                this.limitVelocity1=this.lowerLimit1-d1;
                if(!enableSpring1)d1=this.lowerLimit1;
            }else if(d1>this.upperLimit1){
                if(this.limitState1!=1){
                    this.limitState1=1;
                    this.limitImpulse1=0;
                }
                this.limitVelocity1=this.upperLimit1-d1;
                if(!enableSpring1)d1=this.upperLimit1;
            }else{
                this.limitState1=2;
                this.limitImpulse1=0;
                this.limitVelocity1=0;
            }
            if(!enableSpring1){
                if(this.limitVelocity1>0.005)this.limitVelocity1-=0.005;
                else if(this.limitVelocity1<-0.005)this.limitVelocity1+=0.005;
                else this.limitVelocity1=0;
            }
        }else{
            this.limitState1=2;
            this.limitImpulse1=0;
        }

        if(enableLimit2){
            if(this.lowerLimit2==this.upperLimit2){
                if(this.limitState2!=0){
                    this.limitState2=0;
                    this.limitImpulse2=0;
                }
                this.limitVelocity2=this.lowerLimit2-d2;
                if(!enableSpring2)d2=this.lowerLimit2;
            }else if(d2<this.lowerLimit2){
                if(this.limitState2!=-1){
                    this.limitState2=-1;
                    this.limitImpulse2=0;
                }
                this.limitVelocity2=this.lowerLimit2-d2;
                if(!enableSpring2)d2=this.lowerLimit2;
            }else if(d2>this.upperLimit2){
                if(this.limitState2!=1){
                    this.limitState2=1;
                    this.limitImpulse2=0;
                }
                this.limitVelocity2=this.upperLimit2-d2;
                if(!enableSpring2)d2=this.upperLimit2;
            }else{
                this.limitState2=2;
                this.limitImpulse2=0;
                this.limitVelocity2=0;
            }
            if(!enableSpring2){
                if(this.limitVelocity2>0.005)this.limitVelocity2-=0.005;
                else if(this.limitVelocity2<-0.005)this.limitVelocity2+=0.005;
                else this.limitVelocity2=0;
            }
        }else{
            this.limitState2=2;
            this.limitImpulse2=0;
        }

        if(enableLimit3){
            if(this.lowerLimit3==this.upperLimit3){
                if(this.limitState3!=0){
                    this.limitState3=0;
                    this.limitImpulse3=0;
                }
                this.limitVelocity3=this.lowerLimit3-d3;
                if(!enableSpring3)d3=this.lowerLimit3;
                }else if(d3<this.lowerLimit3){
                if(this.limitState3!=-1){
                    this.limitState3=-1;
                    this.limitImpulse3=0;
                }
                this.limitVelocity3=this.lowerLimit3-d3;
                if(!enableSpring3)d3=this.lowerLimit3;
            }else if(d3>this.upperLimit3){
                if(this.limitState3!=1){
                    this.limitState3=1;
                    this.limitImpulse3=0;
                }
                this.limitVelocity3=this.upperLimit3-d3;
                if(!enableSpring3)d3=this.upperLimit3;
            }else{
                this.limitState3=2;
                this.limitImpulse3=0;
                this.limitVelocity3=0;
            }
            if(!enableSpring3){
                if(this.limitVelocity3>0.005)this.limitVelocity3-=0.005;
                else if(this.limitVelocity3<-0.005)this.limitVelocity3+=0.005;
                else this.limitVelocity3=0;
            }
        }else{
            this.limitState3=2;
            this.limitImpulse3=0;
        }

        if(this.enableMotor1&&(this.limitState1!=0||enableSpring1)){
            this.maxMotorImpulse1=this.maxMotorForce1*timeStep;
        }else{
            this.motorImpulse1=0;
            this.maxMotorImpulse1=0;
        }

        if(this.enableMotor2&&(this.limitState2!=0||enableSpring2)){
            this.maxMotorImpulse2=this.maxMotorForce2*timeStep;
        }else{
            this.motorImpulse2=0;
            this.maxMotorImpulse2=0;
        }

        if(this.enableMotor3&&(this.limitState3!=0||enableSpring3)){
            this.maxMotorImpulse3=this.maxMotorForce3*timeStep;
        }else{
            this.motorImpulse3=0;
            this.maxMotorImpulse3=0;
        }
        
        float rdx=d1*this.ax1+d2*this.ax2+d3*this.ax2;
        float rdy=d1*this.ay1+d2*this.ay2+d3*this.ay2;
        float rdz=d1*this.az1+d2*this.az2+d3*this.az2;
        float w1=this.m2/(this.m1+this.m2);
        if(this.weight>=0)w1=this.weight; // use given weight
        float w2=1-w1;
        this.r1x=this.r1.x+rdx*w1;
        this.r1y=this.r1.y+rdy*w1;
        this.r1z=this.r1.z+rdz*w1;
        this.r2x=this.r2.x-rdx*w2;
        this.r2y=this.r2.y-rdy*w2;
        this.r2z=this.r2.z-rdz*w2;

        // build jacobians
        this.t1x1=this.r1y*this.az1-this.r1z*this.ay1;
        this.t1y1=this.r1z*this.ax1-this.r1x*this.az1;
        this.t1z1=this.r1x*this.ay1-this.r1y*this.ax1;
        this.t2x1=this.r2y*this.az1-this.r2z*this.ay1;
        this.t2y1=this.r2z*this.ax1-this.r2x*this.az1;
        this.t2z1=this.r2x*this.ay1-this.r2y*this.ax1;
        this.l1x1=this.ax1*this.m1;
        this.l1y1=this.ay1*this.m1;
        this.l1z1=this.az1*this.m1;
        this.l2x1=this.ax1*this.m2;
        this.l2y1=this.ay1*this.m2;
        this.l2z1=this.az1*this.m2;
        this.a1x1=this.t1x1*this.i1e00+this.t1y1*this.i1e01+this.t1z1*this.i1e02;
        this.a1y1=this.t1x1*this.i1e10+this.t1y1*this.i1e11+this.t1z1*this.i1e12;
        this.a1z1=this.t1x1*this.i1e20+this.t1y1*this.i1e21+this.t1z1*this.i1e22;
        this.a2x1=this.t2x1*this.i2e00+this.t2y1*this.i2e01+this.t2z1*this.i2e02;
        this.a2y1=this.t2x1*this.i2e10+this.t2y1*this.i2e11+this.t2z1*this.i2e12;
        this.a2z1=this.t2x1*this.i2e20+this.t2y1*this.i2e21+this.t2z1*this.i2e22;

        this.t1x2=this.r1y*this.az2-this.r1z*this.ay2;
        this.t1y2=this.r1z*this.ax2-this.r1x*this.az2;
        this.t1z2=this.r1x*this.ay2-this.r1y*this.ax2;
        this.t2x2=this.r2y*this.az2-this.r2z*this.ay2;
        this.t2y2=this.r2z*this.ax2-this.r2x*this.az2;
        this.t2z2=this.r2x*this.ay2-this.r2y*this.ax2;
        this.l1x2=this.ax2*this.m1;
        this.l1y2=this.ay2*this.m1;
        this.l1z2=this.az2*this.m1;
        this.l2x2=this.ax2*this.m2;
        this.l2y2=this.ay2*this.m2;
        this.l2z2=this.az2*this.m2;
        this.a1x2=this.t1x2*this.i1e00+this.t1y2*this.i1e01+this.t1z2*this.i1e02;
        this.a1y2=this.t1x2*this.i1e10+this.t1y2*this.i1e11+this.t1z2*this.i1e12;
        this.a1z2=this.t1x2*this.i1e20+this.t1y2*this.i1e21+this.t1z2*this.i1e22;
        this.a2x2=this.t2x2*this.i2e00+this.t2y2*this.i2e01+this.t2z2*this.i2e02;
        this.a2y2=this.t2x2*this.i2e10+this.t2y2*this.i2e11+this.t2z2*this.i2e12;
        this.a2z2=this.t2x2*this.i2e20+this.t2y2*this.i2e21+this.t2z2*this.i2e22;

        this.t1x3=this.r1y*this.az3-this.r1z*this.ay3;
        this.t1y3=this.r1z*this.ax3-this.r1x*this.az3;
        this.t1z3=this.r1x*this.ay3-this.r1y*this.ax3;
        this.t2x3=this.r2y*this.az3-this.r2z*this.ay3;
        this.t2y3=this.r2z*this.ax3-this.r2x*this.az3;
        this.t2z3=this.r2x*this.ay3-this.r2y*this.ax3;
        this.l1x3=this.ax3*this.m1;
        this.l1y3=this.ay3*this.m1;
        this.l1z3=this.az3*this.m1;
        this.l2x3=this.ax3*this.m2;
        this.l2y3=this.ay3*this.m2;
        this.l2z3=this.az3*this.m2;
        this.a1x3=this.t1x3*this.i1e00+this.t1y3*this.i1e01+this.t1z3*this.i1e02;
        this.a1y3=this.t1x3*this.i1e10+this.t1y3*this.i1e11+this.t1z3*this.i1e12;
        this.a1z3=this.t1x3*this.i1e20+this.t1y3*this.i1e21+this.t1z3*this.i1e22;
        this.a2x3=this.t2x3*this.i2e00+this.t2y3*this.i2e01+this.t2z3*this.i2e02;
        this.a2y3=this.t2x3*this.i2e10+this.t2y3*this.i2e11+this.t2z3*this.i2e12;
        this.a2z3=this.t2x3*this.i2e20+this.t2y3*this.i2e21+this.t2z3*this.i2e22;

        // build an impulse matrix
        float m12=this.m1+this.m2;
        this.k00=(this.ax1*this.ax1+this.ay1*this.ay1+this.az1*this.az1)*m12;
        this.k01=(this.ax1*this.ax2+this.ay1*this.ay2+this.az1*this.az2)*m12;
        this.k02=(this.ax1*this.ax3+this.ay1*this.ay3+this.az1*this.az3)*m12;
        this.k10=(this.ax2*this.ax1+this.ay2*this.ay1+this.az2*this.az1)*m12;
        this.k11=(this.ax2*this.ax2+this.ay2*this.ay2+this.az2*this.az2)*m12;
        this.k12=(this.ax2*this.ax3+this.ay2*this.ay3+this.az2*this.az3)*m12;
        this.k20=(this.ax3*this.ax1+this.ay3*this.ay1+this.az3*this.az1)*m12;
        this.k21=(this.ax3*this.ax2+this.ay3*this.ay2+this.az3*this.az2)*m12;
        this.k22=(this.ax3*this.ax3+this.ay3*this.ay3+this.az3*this.az3)*m12;

        this.k00+=this.t1x1*this.a1x1+this.t1y1*this.a1y1+this.t1z1*this.a1z1;
        this.k01+=this.t1x1*this.a1x2+this.t1y1*this.a1y2+this.t1z1*this.a1z2;
        this.k02+=this.t1x1*this.a1x3+this.t1y1*this.a1y3+this.t1z1*this.a1z3;
        this.k10+=this.t1x2*this.a1x1+this.t1y2*this.a1y1+this.t1z2*this.a1z1;
        this.k11+=this.t1x2*this.a1x2+this.t1y2*this.a1y2+this.t1z2*this.a1z2;
        this.k12+=this.t1x2*this.a1x3+this.t1y2*this.a1y3+this.t1z2*this.a1z3;
        this.k20+=this.t1x3*this.a1x1+this.t1y3*this.a1y1+this.t1z3*this.a1z1;
        this.k21+=this.t1x3*this.a1x2+this.t1y3*this.a1y2+this.t1z3*this.a1z2;
        this.k22+=this.t1x3*this.a1x3+this.t1y3*this.a1y3+this.t1z3*this.a1z3;

        this.k00+=this.t2x1*this.a2x1+this.t2y1*this.a2y1+this.t2z1*this.a2z1;
        this.k01+=this.t2x1*this.a2x2+this.t2y1*this.a2y2+this.t2z1*this.a2z2;
        this.k02+=this.t2x1*this.a2x3+this.t2y1*this.a2y3+this.t2z1*this.a2z3;
        this.k10+=this.t2x2*this.a2x1+this.t2y2*this.a2y1+this.t2z2*this.a2z1;
        this.k11+=this.t2x2*this.a2x2+this.t2y2*this.a2y2+this.t2z2*this.a2z2;
        this.k12+=this.t2x2*this.a2x3+this.t2y2*this.a2y3+this.t2z2*this.a2z3;
        this.k20+=this.t2x3*this.a2x1+this.t2y3*this.a2y1+this.t2z3*this.a2z1;
        this.k21+=this.t2x3*this.a2x2+this.t2y3*this.a2y2+this.t2z3*this.a2z2;
        this.k22+=this.t2x3*this.a2x3+this.t2y3*this.a2y3+this.t2z3*this.a2z3;

        this.kv00=this.k00;
        this.kv11=this.k11;
        this.kv22=this.k22;

        this.dv00=1/this.kv00;
        this.dv11=1/this.kv11;
        this.dv22=1/this.kv22;

        if(enableSpring1&&this.limitState1!=2){
            float omega=6.2831853*frequency1;
            float k=omega*omega*timeStep;
            float dmp=invTimeStep/(k+2*this.limitMotor1.dampingRatio*omega);
            this.cfm1=this.kv00*dmp;
            this.limitVelocity1*=k*dmp;
        }else{
            this.cfm1=0;
            this.limitVelocity1*=invTimeStep*0.05;
        }
        if(enableSpring2&&this.limitState2!=2){
            omega=6.2831853*frequency2;
            k=omega*omega*timeStep;
            dmp=invTimeStep/(k+2*this.limitMotor2.dampingRatio*omega);
            this.cfm2=this.kv11*dmp;
            this.limitVelocity2*=k*dmp;
        }else{
            this.cfm2=0;
            this.limitVelocity2*=invTimeStep*0.05;
        }
        if(enableSpring3&&this.limitState3!=2){
            omega=6.2831853*frequency3;
            k=omega*omega*timeStep;
            dmp=invTimeStep/(k+2*this.limitMotor3.dampingRatio*omega);
            this.cfm3=this.kv22*dmp;
            this.limitVelocity3*=k*dmp;
        }else{
            this.cfm3=0;
            this.limitVelocity3*=invTimeStep*0.05;
        }
        this.k00+=this.cfm1;
        this.k11+=this.cfm2;
        this.k22+=this.cfm3;

        float inv=1/(
        this.k00*(this.k11*this.k22-this.k21*this.k12)+
        this.k10*(this.k21*this.k02-this.k01*this.k22)+
        this.k20*(this.k01*this.k12-this.k11*this.k02)
        );
        this.d00=(this.k11*this.k22-this.k12*this.k21)*inv;
        this.d01=(this.k02*this.k21-this.k01*this.k22)*inv;
        this.d02=(this.k01*this.k12-this.k02*this.k11)*inv;
        this.d10=(this.k12*this.k20-this.k10*this.k22)*inv;
        this.d11=(this.k00*this.k22-this.k02*this.k20)*inv;
        this.d12=(this.k02*this.k10-this.k00*this.k12)*inv;
        this.d20=(this.k10*this.k21-this.k11*this.k20)*inv;
        this.d21=(this.k01*this.k20-this.k00*this.k21)*inv;
        this.d22=(this.k00*this.k11-this.k01*this.k10)*inv;

        // warm starting
        float totalImpulse1=this.limitImpulse1+this.motorImpulse1;
        float totalImpulse2=this.limitImpulse2+this.motorImpulse2;
        float totalImpulse3=this.limitImpulse3+this.motorImpulse3;
        this.l1.x+=totalImpulse1*this.l1x1+totalImpulse2*this.l1x2+totalImpulse3*this.l1x3;
        this.l1.y+=totalImpulse1*this.l1y1+totalImpulse2*this.l1y2+totalImpulse3*this.l1y3;
        this.l1.z+=totalImpulse1*this.l1z1+totalImpulse2*this.l1z2+totalImpulse3*this.l1z3;
        this.a1.x+=totalImpulse1*this.a1x1+totalImpulse2*this.a1x2+totalImpulse3*this.a1x3;
        this.a1.y+=totalImpulse1*this.a1y1+totalImpulse2*this.a1y2+totalImpulse3*this.a1y3;
        this.a1.z+=totalImpulse1*this.a1z1+totalImpulse2*this.a1z2+totalImpulse3*this.a1z3;
        this.l2.x-=totalImpulse1*this.l2x1+totalImpulse2*this.l2x2+totalImpulse3*this.l2x3;
        this.l2.y-=totalImpulse1*this.l2y1+totalImpulse2*this.l2y2+totalImpulse3*this.l2y3;
        this.l2.z-=totalImpulse1*this.l2z1+totalImpulse2*this.l2z2+totalImpulse3*this.l2z3;
        this.a2.x-=totalImpulse1*this.a2x1+totalImpulse2*this.a2x2+totalImpulse3*this.a2x3;
        this.a2.y-=totalImpulse1*this.a2y1+totalImpulse2*this.a2y2+totalImpulse3*this.a2y3;
        this.a2.z-=totalImpulse1*this.a2z1+totalImpulse2*this.a2z2+totalImpulse3*this.a2z3;
    }

    void solve(){
        float rvx=this.l2.x-this.l1.x+this.a2.y*this.r2z-this.a2.z*this.r2y-this.a1.y*this.r1z+this.a1.z*this.r1y;
        float rvy=this.l2.y-this.l1.y+this.a2.z*this.r2x-this.a2.x*this.r2z-this.a1.z*this.r1x+this.a1.x*this.r1z;
        float rvz=this.l2.z-this.l1.z+this.a2.x*this.r2y-this.a2.y*this.r2x-this.a1.x*this.r1y+this.a1.y*this.r1x;
        float rvn1=rvx*this.ax1+rvy*this.ay1+rvz*this.az1;
        float rvn2=rvx*this.ax2+rvy*this.ay2+rvz*this.az2;
        float rvn3=rvx*this.ax3+rvy*this.ay3+rvz*this.az3;
        float oldMotorImpulse1=this.motorImpulse1;
        float oldMotorImpulse2=this.motorImpulse2;
        float oldMotorImpulse3=this.motorImpulse3;
        float dMotorImpulse1=0;
        float dMotorImpulse2=0;
        float dMotorImpulse3=0;
        if(this.enableMotor1){
            dMotorImpulse1=(rvn1-this.motorSpeed1)*this.dv00;
            this.motorImpulse1+=dMotorImpulse1;
            if(this.motorImpulse1>this.maxMotorImpulse1){ // clamp motor impulse
                this.motorImpulse1=this.maxMotorImpulse1;
            }else if(this.motorImpulse1<-this.maxMotorImpulse1){
                this.motorImpulse1=-this.maxMotorImpulse1;
            }
            dMotorImpulse1=this.motorImpulse1-oldMotorImpulse1;
        }
        if(this.enableMotor2){
            dMotorImpulse2=(rvn2-this.motorSpeed2)*this.dv11;
            this.motorImpulse2+=dMotorImpulse2;
            if(this.motorImpulse2>this.maxMotorImpulse2){ // clamp motor impulse
                this.motorImpulse2=this.maxMotorImpulse2;
            }else if(this.motorImpulse2<-this.maxMotorImpulse2){
                this.motorImpulse2=-this.maxMotorImpulse2;
            }
            dMotorImpulse2=this.motorImpulse2-oldMotorImpulse2;
        }
        if(this.enableMotor3){
            dMotorImpulse3=(rvn3-this.motorSpeed3)*this.dv22;
            this.motorImpulse3+=dMotorImpulse3;
            if(this.motorImpulse3>this.maxMotorImpulse3){ // clamp motor impulse
                this.motorImpulse3=this.maxMotorImpulse3;
            }else if(this.motorImpulse3<-this.maxMotorImpulse3){
                this.motorImpulse3=-this.maxMotorImpulse3;
            }
            dMotorImpulse3=this.motorImpulse3-oldMotorImpulse3;
        }

        // apply motor impulse to relative velocity
        rvn1+=dMotorImpulse1*this.kv00+dMotorImpulse2*this.k01+dMotorImpulse3*this.k02;
        rvn2+=dMotorImpulse1*this.k10+dMotorImpulse2*this.kv11+dMotorImpulse3*this.k12;
        rvn3+=dMotorImpulse1*this.k20+dMotorImpulse2*this.k21+dMotorImpulse3*this.kv22;

        // subtract target velocity and applied impulse
        rvn1-=this.limitVelocity1+this.limitImpulse1*this.cfm1;
        rvn2-=this.limitVelocity2+this.limitImpulse2*this.cfm2;
        rvn3-=this.limitVelocity3+this.limitImpulse3*this.cfm3;

        float oldLimitImpulse1=this.limitImpulse1;
        float oldLimitImpulse2=this.limitImpulse2;
        float oldLimitImpulse3=this.limitImpulse3;

        float dLimitImpulse1=rvn1*this.d00+rvn2*this.d01+rvn3*this.d02;
        float dLimitImpulse2=rvn1*this.d10+rvn2*this.d11+rvn3*this.d12;
        float dLimitImpulse3=rvn1*this.d20+rvn2*this.d21+rvn3*this.d22;

        this.limitImpulse1+=dLimitImpulse1;
        this.limitImpulse2+=dLimitImpulse2;
        this.limitImpulse3+=dLimitImpulse3;

        // clamp
        int clampState=0;
        if(this.limitState1==2||this.limitImpulse1*this.limitState1<0){
            dLimitImpulse1=-oldLimitImpulse1;
            rvn2+=dLimitImpulse1*this.k10;
            rvn3+=dLimitImpulse1*this.k20;
            clampState|=1;
        }
        if(this.limitState2==2||this.limitImpulse2*this.limitState2<0){
            dLimitImpulse2=-oldLimitImpulse2;
            rvn1+=dLimitImpulse2*this.k01;
            rvn3+=dLimitImpulse2*this.k21;
            clampState|=2;
        }
        if(this.limitState3==2||this.limitImpulse3*this.limitState3<0){
            dLimitImpulse3=-oldLimitImpulse3;
            rvn1+=dLimitImpulse3*this.k02;
            rvn2+=dLimitImpulse3*this.k12;
            clampState|=4;
        }

        // update un-clamped impulse
        // TODO: isolate division
        float det;
        switch(clampState){
            case 1:// update 2 3
            det=1/(this.k11*this.k22-this.k12*this.k21);
            dLimitImpulse2=(this.k22*rvn2+-this.k12*rvn3)*det;
            dLimitImpulse3=(-this.k21*rvn2+this.k11*rvn3)*det;
            break;
            case 2:// update 1 3
            det=1/(this.k00*this.k22-this.k02*this.k20);
            dLimitImpulse1=(this.k22*rvn1+-this.k02*rvn3)*det;
            dLimitImpulse3=(-this.k20*rvn1+this.k00*rvn3)*det;
            break;
            case 3:// update 3
            dLimitImpulse3=rvn3/this.k22;
            break;
            case 4:// update 1 2
            det=1/(this.k00*this.k11-this.k01*this.k10);
            dLimitImpulse1=(this.k11*rvn1+-this.k01*rvn2)*det;
            dLimitImpulse2=(-this.k10*rvn1+this.k00*rvn2)*det;
            break;
            case 5:// update 2
            dLimitImpulse2=rvn2/this.k11;
            break;
            case 6:// update 1
            dLimitImpulse1=rvn1/this.k00;
            break;
        }

        this.limitImpulse1=oldLimitImpulse1+dLimitImpulse1;
        this.limitImpulse2=oldLimitImpulse2+dLimitImpulse2;
        this.limitImpulse3=oldLimitImpulse3+dLimitImpulse3;

        float dImpulse1=dMotorImpulse1+dLimitImpulse1;
        float dImpulse2=dMotorImpulse2+dLimitImpulse2;
        float dImpulse3=dMotorImpulse3+dLimitImpulse3;

        // apply impulse
        this.l1.x+=dImpulse1*this.l1x1+dImpulse2*this.l1x2+dImpulse3*this.l1x3;
        this.l1.y+=dImpulse1*this.l1y1+dImpulse2*this.l1y2+dImpulse3*this.l1y3;
        this.l1.z+=dImpulse1*this.l1z1+dImpulse2*this.l1z2+dImpulse3*this.l1z3;
        this.a1.x+=dImpulse1*this.a1x1+dImpulse2*this.a1x2+dImpulse3*this.a1x3;
        this.a1.y+=dImpulse1*this.a1y1+dImpulse2*this.a1y2+dImpulse3*this.a1y3;
        this.a1.z+=dImpulse1*this.a1z1+dImpulse2*this.a1z2+dImpulse3*this.a1z3;
        this.l2.x-=dImpulse1*this.l2x1+dImpulse2*this.l2x2+dImpulse3*this.l2x3;
        this.l2.y-=dImpulse1*this.l2y1+dImpulse2*this.l2y2+dImpulse3*this.l2y3;
        this.l2.z-=dImpulse1*this.l2z1+dImpulse2*this.l2z2+dImpulse3*this.l2z3;
        this.a2.x-=dImpulse1*this.a2x1+dImpulse2*this.a2x2+dImpulse3*this.a2x3;
        this.a2.y-=dImpulse1*this.a2y1+dImpulse2*this.a2y2+dImpulse3*this.a2y3;
        this.a2.z-=dImpulse1*this.a2z1+dImpulse2*this.a2z2+dImpulse3*this.a2z3;
    }
    
}