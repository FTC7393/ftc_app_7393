package org.firstinspires.ftc.teamcode.fair2018;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import evlib.opmodes.AbstractTeleOp;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 * <p>
 * simple TeleOp for the fair robot which has 2 motors
 */
@TeleOp(name = "Fair Robot TeleOp")
@Disabled
public class FairTeleOp2018 extends AbstractTeleOp<FairRobotCfg> {

    private DcMotor clam = null;
    double clamSpeed=.5;
    boolean forward=true;
    boolean start=true;

    @Override
    public Time getMatchTime() {
        return Time.fromMinutes(180); //teleop is 2 minutes
    }

    @Override
    protected FairRobotCfg createRobotCfg() {
        return new FairRobotCfg(hardwareMap);
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
        clam  = robotCfg.getClam();

        //take the joystick values and send them to the motors
        robotCfg.getTwoMotors().runMotors(
                driver1.left_stick_y.getValue(),
                driver1.right_stick_y.getValue()
        );
        if(driver1.dpad_up.justPressed()) {
            clamSpeed+=.1;


        }
        if(driver1.dpad_down.justPressed()) {
            clamSpeed-=.1;


        }
        clam.setPower(clamSpeed);
        if(driver1.b.justPressed()){
            if(forward==true) {
                clam.setDirection(DcMotor.Direction.REVERSE);
                forward=false;
            }
            else if(forward==false) {
                clam.setDirection(DcMotor.Direction.FORWARD);
                forward=true;
            }
        }
        if(driver1.x.justPressed()){
            if(start==true) {
                clam.setPower(clamSpeed);
                start=false;
            }
            else if(start==false) {
                clam.setPower(0);
                start=true;
            }
        }





    }

    @Override
    public void end() {

    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }
}
