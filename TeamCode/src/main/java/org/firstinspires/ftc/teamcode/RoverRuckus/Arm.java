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

    int maxRotationPosition =600;
    int dumpRotationPosition=600;

    int dumpExtensionPosition=600;
    int maxExtensionPosition=600;
    int minRotationPosition=0;
    int minExtensionPosition=0;
    int rotationSetPoint=0;
    double rotationPower=0;


    public Arm(MotorEnc extension, DigitalSensor extensionLimit, MotorEnc rotation, DigitalSensor rotationLimit) {
        this.extension = extension;
        this.extensionLimit= new DigitalInputEdgeDetector(extensionLimit);
        this.rotation = rotation;
        this.rotationLimit= new DigitalInputEdgeDetector(rotationLimit);

        this.rotationPID=new PIDController(0,0,0,0);
        this.extensionPID=new PIDController(0,0,0,0);

    }

    public void act() {
        extension.update();
        rotation.update();
        rotationEncoder=rotation.getEncoderPosition();
        extensionEncoder=extension.getEncoderPosition();
//        if( rotationLimit.justPressed()==true){
//            rotation.resetEncoder();
//            minRotationPosition=0;
//
//        }
//
//        if( extensionLimit.justPressed()==true){
//            extension.resetEncoder();
//            minExtensionPosition=0;
//
//        }
////        rotationPower=rotationPID.computeCorrection(rotationSetPoint,rotationEncoder);
//        rotation.setSpeed(rotationPower);
//        if(rotationSetPoint>maxRotationPosition){
//            rotationSetPoint=maxRotationPosition;
//        }
//        rotation.setPosition(rotationSetPoint,rotationPower);
//

//        torque=-1*extensionLength*mass*gravity*java.lang.Math.cos(angle);
//        extensionForce=-1*mass*gravity*java.lang.Math.sin(angle);



    }

    public void zeroRotation(){rotation.setPosition(rotationSetPoint,1);}
    public void endRotation(){rotationSetPoint=maxRotationPosition;}
    public void dumpRotation(){rotation.setPosition(dumpRotationPosition,1);}

    public void controlRotation(double y){rotation.setPower(y);}

    public void zeroExtension(){extension.setPosition(minExtensionPosition,1);}
    public void endExtension(){extension.setPosition(maxExtensionPosition,1);}
    public void controlExtension(double x){extension.setPower(x);}
    public void dumpExtension(){extension.setPosition(dumpExtensionPosition,1);}
    public double getRotationEncoder(){return rotationEncoder;}
    public double getExtensionEncoder(){return extensionEncoder;}

}
