package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import evlib.hardware.config.RobotCfg;
import evlib.hardware.control.MecanumControl;
import evlib.hardware.control.RotationControls;
import evlib.hardware.control.TranslationControls;
import evlib.hardware.sensors.Gyro;
import evlib.opmodes.AbstractAutoOp;
import evlib.opmodes.AbstractOp;
import evlib.statemachine.EVStateMachineBuilder;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
@Autonomous(name = "RoverRuckusAutoOp")

public class RoverRuckusAuto1 extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;

    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }


    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {

    }



    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        TeamColor teamColor = TeamColor.RED;

        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.UP_HANGING, teamColor, Angle.fromDegrees(1));
        b.add(S.UP_HANGING, new BasicAbstractState() {
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public void init() {
                runtime.reset();

            }

            @Override
            public boolean isDone() {
                boolean doneHanging=false;
                    if (!robotCfg.getHanging().islatchLimitPressed()&&runtime.seconds() < 6.0) {
//                        robotCfg.getMecanumControl().setRotationControl(RotationControls.ZERO);
//                        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.constant(.2, Angle.fromDegrees(270)));

                        robotCfg.getHanging().upHanging();
                    } else {
                        robotCfg.getHanging().stopHanging();
//                        robotCfg.getMecanumControl().setRotationControl(RotationControls.ZERO);
//                        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.ZERO);
                        doneHanging=true;

                    }


                return doneHanging;
            }

            @Override
            public StateName getNextStateName() {
                return S.RELEASE_LATCH;
            }
        });
        //b.add(S.UNLATCH_SERVO)
        //b.add(S.DRIVE-90DEGREES)
        b.addServo(S.RELEASE_LATCH,S.WAIT,RoverRuckusRobotCfg.MainServoName.LATCH,RoverRuckusRobotCfg.latchPresets.UNLATCH,true);
        b.addWait(S.WAIT,S.DRIVE_TO_DEPOT,500);
//        b.add(S.DOWN_HANGING, new BasicAbstractState() {
//            private ElapsedTime runtime = new ElapsedTime();
//
//            @Override
//            public void init() {
//                runtime.reset();
//
//
//            }
//
//            @Override
//            public boolean isDone() {
//                boolean doneHanging=false;
//                if (!robotCfg.getHanging().isUnlatchLimitPressed()&&runtime.seconds() < 6.0) {
//
//                    robotCfg.getHanging().downHanging();
//                } else {
//                    robotCfg.getHanging().stopHanging();
//                    doneHanging=true;
//
//                }
//
//
//                return doneHanging;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return S.DRIVE_TO_DEPOT;
//            }
//        });
        b.addDrive(S.DRIVE_TO_DEPOT,S.RELEASE_MARKER,Distance.fromFeet(2.4),.5,270,0);
        b.addServo(S.RELEASE_MARKER,S.DRIVE_TO_CRATER,RoverRuckusRobotCfg.MainServoName.MARKER,RoverRuckusRobotCfg.markerPresets.RELEASE,true);
        b.addDrive(S.DRIVE_TO_CRATER,S.STOP,Distance.fromFeet(8),.5,45,225);
        // to go towards the other crater is 135 degrees; note - the gain on the gyro on the gyro control needs adjusting to keep it from
        b.addStop(S.STOP);

        return b.build();
    }


    private enum S implements StateName {
        WAIT,
        UP_HANGING,
        DOWN_HANGING,
        UNLATCH,
        LOCK_ROTATION,
        MOVE_OFF_HOOK,
        START,
        RELEASE_MARKER,
        ANTITIP,

        DETECT_GOLD,
        STOP,

        GOLD_ALIGN, DRIVE_TO_DEPOT, DRIVE_LITTLE,DRIVE_TO_CRATER, RELEASE_LATCH

    }
}
