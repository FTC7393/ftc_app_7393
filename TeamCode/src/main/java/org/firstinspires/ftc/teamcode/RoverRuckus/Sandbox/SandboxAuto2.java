package org.firstinspires.ftc.teamcode.RoverRuckus.Sandbox;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckus.GoldDetector;
import org.firstinspires.ftc.teamcode.RoverRuckus.GoldPosition;
import org.firstinspires.ftc.teamcode.RoverRuckus.ObjectDetector;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusOptionsOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;

import evlib.hardware.control.MecanumControl;
import evlib.hardware.sensors.Gyro;
import evlib.opmodes.AbstractAutoOp;
import evlib.statemachine.EVStateMachineBuilder;
import evlib.util.EVConverters;
import evlib.util.FileUtil;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Time;

@Autonomous(name = "Sandbox Auto 2")
public class SandboxAuto2 extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;
    double orientationDepot;
    double directionDepot;
    ResultReceiver<GoldDetector.Detection> left;
    ResultReceiver<GoldDetector.Detection> middle;
    ResultReceiver<GoldDetector.Detection> right;
    TeamColor teamColor;
    boolean isStartingDepot;
    boolean moveToOpponentCrater;
    double wait;

    GoldPosition goldPosition;


    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
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
        telemetry.addData("LEFT mineral:", left.getValue());
        telemetry.addData("MID  mineral:", middle.getValue());
        telemetry.addData("RGHT mineral:", right.getValue());
        telemetry.addData("goldPosition",goldPosition);
    }

    @Override
    protected void end() {

    }
    enum S implements StateName {
        PAN_LEFT,
        PAUSE1,
        LOOK_FOR_GOLD_LEFT,
        PAN_MID,
        PAUSE2,
        LOOK_FOR_GOLD_MIDDLE,
        PAN_RIGHT,
        LOOK_FOR_GOLD_RIGHT,
        PAUSE3,
        STOP;
        }

    @Override
    public StateMachine buildStates() {
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(RoverRuckusOptionsOp.FILENAME));


        teamColor = TeamColor.RED;
        isStartingDepot = optionsFile.get(RoverRuckusOptionsOp.isStartingDepot, RoverRuckusOptionsOp.isStartingDepotDefault);
        moveToOpponentCrater = optionsFile.get(RoverRuckusOptionsOp.moveToOpponentCrater, RoverRuckusOptionsOp.moveToOpponentCraterDefault);
        if (!moveToOpponentCrater) {
            orientationDepot = -45;
            directionDepot = -135;
        } else {
            orientationDepot = -135;
            directionDepot = -45;

        }

        left = new BasicResultReceiver<>();
        middle = new BasicResultReceiver<>();
        right = new BasicResultReceiver<>();


        final ResultReceiver<GoldDetector.Detection> mineralResultReceiver = new BasicResultReceiver<>();
        final ResultReceiver<Boolean> cameraActionNotifier = new BasicResultReceiver<>();
        int numCycles = 3;
        ObjectDetector.initThread(numCycles, telemetry, hardwareMap, mineralResultReceiver, cameraActionNotifier);


        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.PAN_LEFT, teamColor, Angle.fromDegrees(3));
        RoverRuckusRobotCfg.MainServoName panServo = RoverRuckusRobotCfg.MainServoName.PHONEPAN;
        RoverRuckusRobotCfg.PhonePanPresets psLeft = RoverRuckusRobotCfg.PhonePanPresets.LEFT;
        RoverRuckusRobotCfg.PhonePanPresets psMid = RoverRuckusRobotCfg.PhonePanPresets.MIDDLE;
        RoverRuckusRobotCfg.PhonePanPresets psRight = RoverRuckusRobotCfg.PhonePanPresets.RIGHT;
        int pauseTime = 3;
        b.addServo(S.PAN_LEFT, S.PAUSE1, panServo, psLeft, 1.0, true);
        b.addWait(S.PAUSE1, S.LOOK_FOR_GOLD_LEFT, Time.fromSeconds(pauseTime));
        b.add(S.LOOK_FOR_GOLD_LEFT, getLookState(cameraActionNotifier, mineralResultReceiver, S.PAN_MID,left));
        b.addServo(S.PAN_MID, S.PAUSE2, panServo, psMid, 1.0, true);
        b.addWait(S.PAUSE2, S.LOOK_FOR_GOLD_MIDDLE, Time.fromSeconds(pauseTime));
        b.add(S.LOOK_FOR_GOLD_MIDDLE, getLookState(cameraActionNotifier, mineralResultReceiver, S.PAN_RIGHT,middle));
        b.addServo(S.PAN_RIGHT, S.PAUSE3, panServo, psRight, 1.0, true);
        b.addWait(S.PAUSE3, S.LOOK_FOR_GOLD_RIGHT, Time.fromSeconds(pauseTime));
        b.add(S.LOOK_FOR_GOLD_RIGHT, getLookState(cameraActionNotifier, mineralResultReceiver, S.STOP,right));
        b.addStop(S.STOP);
        return b.build();
    }

    private State getLookState(final ResultReceiver<Boolean> cameraActor, final ResultReceiver<GoldDetector.Detection> goldRR, final StateName next, final ResultReceiver<GoldDetector.Detection> posDetRR) {
        return new BasicAbstractState() {

            @Override
            public void init() {
                cameraActor.setValue(true);
            }

            @Override
            public boolean isDone() {
                if (goldRR.isReady()) {
                    posDetRR.setValue(goldRR.getValue());
                    goldRR.clear();
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return next;
            }
        };
    }

//    @Override
//    public StateMachine buildStates() {
//        FrameGrabber fg = null;
//        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE1, TeamColor.RED, Angle.fromRadians(0.05),
//                robotCfg.getGyro(), fg, servos, robotCfg.getMecanumControl());
//        Distance dist;
//        b.addDrive(S.DRIVE1, S.TURN1, Distance.fromFeet(3), 0.75, Angle.fromDegrees(90), Angle.fromDegrees(0));
//        b.addGyroTurn(S.TURN1, S.STOP, 90,.1);
////        b.addDrive(S.DRIVE2, S.STOP, Distance.fromFeet(4), 0.35, Angle.fromDegrees(180), Angle.fromDegrees(180));
//
//        b.addMotorOn(S.M_ON, S.STOP, robotCfg.getTestMotor(), 0.8);
//        b.addWait(S.WAIT5, S.STOP, Time.fromSeconds(5));
//
//        b.addStop(S.STOP);
//        return b.build();
//    }


}
