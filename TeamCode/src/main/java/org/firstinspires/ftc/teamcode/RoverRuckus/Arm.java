package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.ColorSensor;

import evlib.hardware.motors.Motor;
import evlib.hardware.motors.MotorEnc;
import evlib.hardware.sensors.DigitalSensor;
import evlib.hardware.servos.ServoControl;
import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.electronvolts.util.PIDController;

/**
 * Created by ftc7393 on 10/27/2018.
 */

public class Arm {
    double extensionLength;
    double mass;
    double gravity;
    double angle;
    double torque;
    double extensionForce;
    private MotorEnc extension;
    private MotorEnc rotation;


    private final DigitalInputEdgeDetector extensionLimit,rotationLimit;
    private PIDController rotationPID;
    private PIDController extensionPID;
    private double rotationEncoder=0;
    private double extensionEncoder=0;

    int maxRotationPosition =4696;
    int dumpRotationPosition=600;

    int dumpExtensionPosition=600;
    int maxExtensionPosition=2685;
    int minRotationPosition=0;
    int minExtensionPosition=0;
    double rotationSetPoint=0;
    double rotationPower=0;
    double extensionSetPoint=0;
    double extensionPower=0;


    public Arm(MotorEnc extension, DigitalSensor extensionLimit, MotorEnc rotation, DigitalSensor rotationLimit) {
        this.extension = extension;
        this.extensionLimit= new DigitalInputEdgeDetector(extensionLimit);
        this.rotation = rotation;
        this.rotationLimit= new DigitalInputEdgeDetector(rotationLimit);

        this.rotationPID=new PIDController(0.001,0.0002,.00006,1);
        this.extensionPID=new PIDController(0.003,0,0,1);

    }

    public void act() {

        rotationEncoder=rotation.getEncoderPosition();
        extensionEncoder=extension.getEncoderPosition();
//        if( rotationLimit.isPressed()==true){
//            rotation.resetEncoder();
//
//        }
//
//        if( extensionLimit.isPressed()==true){
//            extension.resetEncoder();
//
//        }
        rotationPower=rotationPID.computeCorrection(rotationSetPoint,rotationEncoder);
        extensionPower=extensionPID.computeCorrection(extensionSetPoint,extensionEncoder);

//        rotation.setSpeed(rotationPower);
//        if(rotationSetPoint>maxRotationPosition){
//            rotationSetPoint=maxRotationPosition;
//        }
//        if(rotationSetPoint<minRotationPosition){
//            rotationSetPoint=minRotationPosition;
//        }
        if(extensionSetPoint>maxExtensionPosition){
            extensionSetPoint=maxExtensionPosition;
        }
        if(extensionSetPoint<minExtensionPosition){
            extensionSetPoint=minExtensionPosition;
        }
//        rotation.setPower(rotationPower);
        extension.setPower(extensionPower);
        rotation.setPosition((int)rotationSetPoint,rotationPower);



//        torque=-1*extensionLength*mass*gravity*java.lang.Math.cos(angle);
//        extensionForce=-1*mass*gravity*java.lang.Math.sin(angle);
        extension.update();
        rotation.update();


    }

    public void zeroRotation(){rotation.setPosition((int)rotationSetPoint,1);}
    public void endRotation(){rotationSetPoint=maxRotationPosition;}
    public void dumpRotation(){rotation.setPosition(dumpRotationPosition,1);}

    public void controlRotation(double y){rotationSetPoint=rotationSetPoint+(y*100);}
//    public void controlRotation(double y){rotation.setPower(y);}
//    public void controlExtension(double x){extension.setPower(x);}
    public void controlExtension(double x){extensionSetPoint=extensionSetPoint+(x*100);}

    public void zeroExtension(){extension.setPosition(minExtensionPosition,1);}
    public void endExtension(){extension.setPosition(maxExtensionPosition,1);}


    public void dumpExtension(){extension.setPosition(dumpExtensionPosition,1);}
    public double getRotationEncoder(){return rotationEncoder;}
    public double getExtensionEncoder(){return extensionEncoder;}

}
