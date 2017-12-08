package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ftc7393 on 11/18/2017.
 */

public class TestHardware {
    /* Public OpMode members. */

    public Servo    relicGrabber   = null;
    public static final double MID_SERVO       =  0.5 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TestHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        relicGrabber  = hwMap.get(Servo.class, "relic_grabber");
        relicGrabber.setPosition(MID_SERVO);
    }
}
