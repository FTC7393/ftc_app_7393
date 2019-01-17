package org.firstinspires.ftc.teamcode.RoverRuckus.Sandbox;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

import evlib.opmodes.AbstractAutoOp;
import evlib.statemachine.EVStateMachineBuilder;
import evlib.vision.framegrabber.FrameGrabber;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;

@Autonomous(name = "Sandbox Auto")
public class SandboxAuto extends AbstractAutoOp<RoverCfg> {

    enum S implements StateName {
        DRIVE1,
        TURN1,
//        DRIVE2,
        STOP;
    }

    @Override
    public StateMachine buildStates() {
        FrameGrabber fg = null;
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.TURN1, TeamColor.RED, Angle.fromDegrees(1.5),
                robotCfg.getGyro(), fg, servos, robotCfg.getMecanumControl());
        Distance dist;
        b.addDrive(S.DRIVE1, S.TURN1, Distance.fromFeet(4), 0.75, Angle.fromDegrees(0), Angle.fromDegrees(0));
        b.addGyroTurn(S.TURN1, S.STOP, 90, 0.2);
//        b.addDrive(S.DRIVE2, S.STOP, Distance.fromFeet(4), 0.35, Angle.fromDegrees(180), Angle.fromDegrees(180));
        b.addStop(S.STOP);
        return b.build();
    }

    @Override
    protected RoverCfg createRobotCfg() {
        return new RoverCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        String prefix = "sandbox_log";
        String postfix = ".txt";
        ImmutableList.Builder<Logger.Column> cols = ImmutableList.builder();
        cols.addAll(robotCfg.getLoggerColumns());
        return new Logger(prefix, postfix, cols.build());
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
    }

    @Override
    protected void end() {

    }

}
