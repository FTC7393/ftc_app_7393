package org.firstinspires.ftc.teamcode.RoverRuckus3DayBuild;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import evlib.opmodes.AbstractOp;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 9/8/2018.
 */
@Autonomous(name = "Auto3DayBuild")

public class Auto3DayBuild extends AbstractOp<RobotCfg3DayBuild> {
    private StateName state = StateName.RELEASE_LATCH;
    private ElapsedTime runtime = new ElapsedTime();


    private enum StateName {
        SERVO_INIT,
        RELEASE_LATCH,
        GO_LEFT,
        ARM_DOWN,
        TURN,
        GO_TO_CORNER,
        RELEASE_MARKER,
        GO_TO_CRATER,
        PARK_CRATER

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
    protected void pre_act() {

    }

    @Override
    protected void act() {


        boolean done = false;
        int redColor = robotCfg.getColorSensor().red();
        int blueColor = robotCfg.getColorSensor().blue();
        int alpha = robotCfg.getColorSensor().alpha();
        telemetry.addData("redColor", redColor);
        telemetry.addData("blueColor", blueColor);
        telemetry.addData("slideEncoder", robotCfg.getMechanisms().getSlideEncoderValue());


        if (state == StateName.RELEASE_LATCH) {

            done = robotCfg.getMechanisms().unLatch();
            //robotCfg.getMechanisms().openLatch();
            if (done == true) {
                state = StateName.GO_LEFT;
                runtime.reset();

            }
        } else if (state == StateName.GO_LEFT) {
            if (runtime.seconds() < 1.0) {
                robotCfg.getMechanisms().drive(-1, -1);

            } else {
                robotCfg.getMechanisms().drive(0, 0);

                state = StateName.RELEASE_MARKER;
                runtime.reset();
            }


        } else if (state == StateName.TURN) {


            if (runtime.seconds() < 3.0) {
                robotCfg.getMechanisms().drive(0, -.5);

            } else {
                robotCfg.getMechanisms().drive(0, 0);
                state = StateName.GO_TO_CORNER;
                runtime.reset();
            }

        } else if (state == StateName.GO_TO_CORNER) {

            if (/*blueColor < 50 && redColor < 50 &&*/ runtime.seconds() < 5.0) {
                robotCfg.getMechanisms().drive(-.5, -.5);

            } else {
                robotCfg.getMechanisms().drive(0, 0);


                state = StateName.RELEASE_MARKER;
            }

        } else if (state == StateName.RELEASE_MARKER) {
            robotCfg.getMechanisms().releaseMarker();


        }


    }


    @Override
    protected void post_act() {

    }

    @Override
    protected void end() {

    }

    @Override
    protected Time getMatchTime() {
        return Time.fromSeconds(30);
    }

    @Override
    public void init() {

        super.init();

    }

}