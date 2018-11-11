package org.firstinspires.ftc.teamcode.RoverRuckus3DayBuild;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import evlib.hardware.servos.ServoName;
import evlib.opmodes.AbstractTeleOp;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.files.Logger;

/**
 * Created by ftc7393 on 9/8/2018.
 */
@TeleOp(name = "TeleOp3DayBuild")

public class TeleOp3DayBuild extends AbstractTeleOp<RobotCfg3DayBuild> {
    double left;
    double right;
    double drive;
    double turn;
    double max;
    @Override
    public void init(){
        super.init();
        robotCfg.getMechanisms().openLatch();
    }
    @Override
    protected Function getJoystickScalingFunction() {
        return null;
    }

    @Override
    protected RobotCfg3DayBuild createRobotCfg() {
        return new RobotCfg3DayBuild(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        telemetry.addData("slideEncoder", robotCfg.getMechanisms().getSlideEncoderValue());
        int redColor = robotCfg.getColorSensor().red();
        int blueColor = robotCfg.getColorSensor().blue();
        int alpha = robotCfg.getColorSensor().alpha();
        telemetry.addData("redColor", redColor);
        telemetry.addData("blueColor", blueColor);
        if (driver1.dpad_up.isPressed()) {
            robotCfg.getMechanisms().Up();
        }
        if (driver1.dpad_down.isPressed()) {
            robotCfg.getMechanisms().Down();
        }

        if(driver1.x.justPressed()){
            robotCfg.getMechanisms().releaseMarker();
        }

        if(driver1.y.justPressed()){
            robotCfg.getMechanisms().clampMarker();
        }
        if(driver1.left_bumper.justPressed()){
            robotCfg.getMechanisms().openLatch();
        }

        if(driver1.right_bumper.justPressed()){
            robotCfg.getMechanisms().closeLatch();
        }
        drive = -gamepad1.left_stick_x;
        turn  =  gamepad1.left_stick_y;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }
        robotCfg.getMechanisms().drive(left,right);

    }

    @Override
    protected void end() {

    }
}
