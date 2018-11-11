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

public class Collector {

    private Motor collection;
    private ServoControl door;
    private ColorSensor left;
    private ColorSensor right;




    public Collector( Motor collection, ServoControl door, ColorSensor left, ColorSensor right) {

        this.collection = collection;
        this.door = door;
        this.left = left;
        this.right = right;

    }

    public void act() {

        collection.update();
       ;



    }
    public void forwardCollection(){collection.setPower(1);}
    public void backwardCollection(){collection.setPower(-1);}
    public void offCollection(){collection.setPower(0);}
    public void rightDoor(){door.goToPreset(RoverRuckusRobotCfg.doorPresets.RIGHT);}
    public void leftDoor(){door.goToPreset(RoverRuckusRobotCfg.doorPresets.LEFT);}
    public void closeDoor(){door.goToPreset(RoverRuckusRobotCfg.doorPresets.CLOSE);}

}
