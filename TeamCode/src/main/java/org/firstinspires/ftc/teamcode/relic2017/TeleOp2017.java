package org.firstinspires.ftc.teamcode.relic2017;


import com.google.common.collect.ImmutableList;
import com.google.common.io.BaseEncoding;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;



/**
 * Created by ftc7393 on 11/24/2017.
 */
@TeleOp(name = "TeleOp2017")
public class TeleOp2017 extends AbstractTeleOp<RobotCfg2017> {

    ScalingInputExtractor rightY;
    ScalingInputExtractor leftX;
    ScalingInputExtractor rightX;

    class ScalingInputExtractor implements InputExtractor<Double> {
        InputExtractor<Double> ext;
        private double factor;
        ScalingInputExtractor(InputExtractor<Double> ext, double f) {
            this.ext = ext;
            this.factor = f;
        }
        @Override
        public Double getValue() {
            return ext.getValue()*factor;
        }
        public void setFactor(double f) {
            this.factor = f;
        }
    }
    private MotorSpeedFactor currentSpeedFactor = MotorSpeedFactor.FAST;

    enum MotorSpeedFactor {
        FAST(1.0), SLOW(0.5);
        private double factor;
        MotorSpeedFactor(double x) {
            this.factor = x;
        }
        public double getFactor() {
            return factor;
        }
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.eBased(5);
    }

    @Override
    protected RobotCfg2017 createRobotCfg() {
        return new RobotCfg2017(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        //log the gamepads, and the motors, sensors, servos, etc. from MainRobotCfg
        return new Logger("", "teleop.csv", new ImmutableList.Builder<Logger.Column>()
//                .add(
//                new Logger.Column("state", new InputExtractor<StateName>() {
//                    @Override
//                    public StateName getValue() {
//                        return stateMachine.getCurrentStateName();
//                    }
//                }))
                .addAll(robotCfg.getLoggerColumns()).add(
                        new Logger.Column(TeleOpPlayback.GAMEPAD_1_TITLE, new InputExtractor<String>() {
                            @Override
                            public String getValue() {
                                try {
                                    return BaseEncoding.base64Url().encode(gamepad1.toByteArray());
                                } catch (RobotCoreException e) {
                                    e.printStackTrace();
                                    return "";
                                }
                            }
                        })).add(
                        new Logger.Column(TeleOpPlayback.GAMEPAD_2_TITLE, new InputExtractor<String>() {
                            @Override
                            public String getValue() {
                                try {
                                    return BaseEncoding.base64Url().encode(gamepad2.toByteArray());
                                } catch (RobotCoreException e) {
                                    e.printStackTrace();
                                    return "";
                                }
                            }
                        })
                ).build());
    }


    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {

    }

    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        rightY = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_y), f);
        leftX = new ScalingInputExtractor(InputExtractors.negative(driver1.left_stick_x), f);
        rightX = new ScalingInputExtractor(driver1.right_stick_x, f);
        //noinspection SuspiciousNameCombination
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
    }

    @Override
    protected void go() {
        forwardControl();

    }

    @Override
    protected void act() {



        if(driver1.a.justPressed()) {
            if (currentSpeedFactor == MotorSpeedFactor.FAST) {
                currentSpeedFactor = MotorSpeedFactor.SLOW;
            } else {
                currentSpeedFactor = MotorSpeedFactor.FAST;
            }
            double f = currentSpeedFactor.getFactor();
            leftX.setFactor(f);
            rightY.setFactor(f);
            rightX.setFactor(f);
        }

        telemetry.addData("liftEncoder", robotCfg.getGrabber().getLiftEncoderValue());
        telemetry.addData("topSwitch", robotCfg.getGrabber().getTopLimit());
        telemetry.addData("bottomSwitch", robotCfg.getGrabber().getBottomLimit());
        telemetry.addData("leftTrigger",driver1.left_trigger.getRawValue());
        telemetry.addData("rightTrigger",driver1.right_trigger.getRawValue());
        telemetry.addData("mode",robotCfg.getGrabber().getMode().name());
        telemetry.addData("encoderValue",robotCfg.getGrabber().getLiftEncoderValue());
        telemetry.addData("setPoint",robotCfg.getRelic().getRelicEncoderValue());



        if(driver1.right_bumper.justPressed()){
            robotCfg.getGrabber().toggleServo();
        }
        if(driver1.left_bumper.justPressed()){
            robotCfg.getGrabber().closeServo();
        }

        if(driver1.dpad_up.isPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){
            robotCfg.getGrabber().goUp();
        }
        else if(driver1.dpad_down.justPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){
            robotCfg.getGrabber().goDown();
        }
        else if(driver1.dpad_right.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_left.isPressed()){
            robotCfg.getGrabber().goMiddle();
        }
        else if(driver1.dpad_left.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_right.isPressed()){
            robotCfg.getGrabber().goLittleUp();
        }
        robotCfg.getRelic().setPower(driver2.left_stick_y.getValue());

        if(driver2.right_bumper.isPressed()){
            robotCfg.getRelic().toggleServo();
        }







    }

    @Override
    protected void end() {

    }
}
