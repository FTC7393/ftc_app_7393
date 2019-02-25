package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.util.List;

import evlib.hardware.control.MecanumControl;
import evlib.hardware.sensors.Gyro;
import evlib.opmodes.AbstractAutoOp;
import evlib.statemachine.EVStateMachineBuilder;
import evlib.util.EVConverters;
import evlib.util.FileUtil;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
@Autonomous(name = "RoverRuckusAutoOp3")

public class RoverRuckusAuto3 extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;
    double orientationDepot;
    double directionDepot;
    GoldDetector.Detection left;
    GoldDetector.Detection right;
    GoldDetector.Detection middle;
    Mineral leftMineral=new Mineral();
    Mineral rightMineral=new Mineral();
    Mineral middleMineral=new Mineral();
    TeamColor teamColor;
    boolean isStartingDepot;
    boolean moveToOpponentCrater;

    final ResultReceiver<List<Mineral>> potentialMineralResultReceiver = new BasicResultReceiver<>();

    GoldPosition goldPosition;

//    static PrintStream mineralLogOutputter = null;
//    public static PrintStream getMineralLogWriter() {
//        if (mineralLogOutputter == null) {
//            String fname = String.format("%d_%s", System.currentTimeMillis(), "mineral_log.csv");
//
//            File dir = FileUtil.getLogsDir();
//            File logFile = new File(dir, fname);
//            try {
//                Charset UTF8 = Charset.forName("UTF-8");
//                mineralLogOutputter = new PrintStream(new FileOutputStream(logFile));
//
//            } catch (IOException e) {
//                throw new RuntimeException("Error - can't open log file: " + e.getMessage());
//            }
//        }
//        return mineralLogOutputter;
//
//    }


    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {


        return new Logger("", "auto.csv",
                new ImmutableList.Builder<Logger.Column>()
                .add(new Logger.Column("State", new InputExtractor<StateName>() {

                    @Override
                    public StateName getValue() {
                        return stateMachine.getCurrentStateName();
                    }
                }))
                .add(new Logger.Column("Left Mineral,c,x,y,w,h,r", new InputExtractor<Mineral>() {
                    @Override
                    public Mineral getValue() {
                        return leftMineral;
                    }
                }))
                .add(new Logger.Column("Middle Mineral,c,x,y,w,h,r", new InputExtractor<Mineral>() {
                    @Override
                    public Mineral getValue() {
                        return middleMineral;
                    }
                }))
                .add(new Logger.Column("Right Mineral,c,x,y,w,h,r", new InputExtractor<Mineral>() {
                    @Override
                    public Mineral getValue() {
                        return rightMineral;
                    }
                }))
                        .build()
        );
    }


    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {
        if (potentialMineralResultReceiver.isReady()) {
            List<Mineral> mlist = potentialMineralResultReceiver.getValue();
            for (Mineral m : mlist) {
                m.showInTelem(telemetry);
            }
        }

        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("goldPosition",goldPosition);
        telemetry.addData("left",leftMineral.getType());
        telemetry.addData("middle",middleMineral.getType());
        telemetry.addData("right",rightMineral.getType());
//        telemetry.addData("leftMineral",leftMineral.toString());
//        telemetry.addData("rightMineral",rightMineral.toString());
//        telemetry.addData("middleMineral",middleMineral.toString());



    }





    @Override
    protected void end() {
//        mineralLogOutputter.close();
//        mineralLogOutputter = null;
    }

    @Override
    public StateMachine buildStates() {
        double panSpeed = 0.2;
        double wait= 0.5;

        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(RoverRuckusOptionsOp.FILENAME));


        teamColor = TeamColor.RED;
        isStartingDepot=optionsFile.get(RoverRuckusOptionsOp.isStartingDepot,RoverRuckusOptionsOp.isStartingDepotDefault);
        moveToOpponentCrater=optionsFile.get(RoverRuckusOptionsOp.moveToOpponentCrater,RoverRuckusOptionsOp.moveToOpponentCraterDefault);
        if(!moveToOpponentCrater){
            orientationDepot=-45;
            directionDepot=-135;
        }
        else{
            orientationDepot=-135;
            directionDepot=-45;

        }
        final ResultReceiver <Mineral> mineralResultReceiver = new BasicResultReceiver<>();

        final ResultReceiver<Boolean> actResultReceiver=new BasicResultReceiver<>();

        int numCycles = 3;
        ObjectDetector.initThread(numCycles, telemetry,hardwareMap,mineralResultReceiver,actResultReceiver, potentialMineralResultReceiver) ;

        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.DESCEND, teamColor, Angle.fromDegrees(3));
        double descentTime = 6.0;
        b.add(S.DESCEND, createDescendState(S.RELEASE_LATCH, descentTime));
        b.addServo(S.RELEASE_LATCH,S.WAIT_2,RoverRuckusRobotCfg.MainServoName.LATCH,RoverRuckusRobotCfg.LatchPresets.UNLATCH,true);
        b.addWait(S.WAIT_2, S.DETECT_MIDDLE_GOLD, Time.fromSeconds(1.5));
        b.add(S.DETECT_MIDDLE_GOLD, new BasicAbstractState() {
            @Override
            public void init() {
                actResultReceiver.setValue(true);
            }

            @Override
            public boolean isDone() {
                if (mineralResultReceiver.isReady()) {
                    middleMineral=mineralResultReceiver.getValue();
                    middle = middleMineral.getType();
                    mineralResultReceiver.clear();

                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {

                return S.DRIVE_CLOSER;
            }
        });

        b.addDrive(S.DRIVE_CLOSER,S.MOVE_SERVO_LEFT_GOLD,Distance.fromFeet(.30),.5,270,0);
        b.addServo(S.MOVE_SERVO_LEFT_GOLD,S.WAIT_1,RoverRuckusRobotCfg.MainServoName.PHONEPAN,RoverRuckusRobotCfg.PhonePanPresets.LEFT,panSpeed,true);
        b.addWait(S.WAIT_1,S.DETECT_LEFT_GOLD,Time.fromSeconds(wait));


        b.add(S.DETECT_LEFT_GOLD, new BasicAbstractState() {
            @Override
            public void init() {
                actResultReceiver.setValue(true);
            }

            @Override
            public boolean isDone() {
                if (mineralResultReceiver.isReady()) {
                    leftMineral=mineralResultReceiver.getValue();
                    left = leftMineral.getType();
                    mineralResultReceiver.clear();

                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return S.MOVE_SERVO_RIGHT_GOLD;
            }
        });
        b.addServo(S.MOVE_SERVO_RIGHT_GOLD,S.WAIT_3,RoverRuckusRobotCfg.MainServoName.PHONEPAN,RoverRuckusRobotCfg.PhonePanPresets.RIGHT,panSpeed,true);
        b.addWait(S.WAIT_3,S.DETECT_RIGHT_GOLD,Time.fromSeconds(wait));

        b.add(S.DETECT_RIGHT_GOLD, new BasicAbstractState() {
            @Override
            public void init() {
                actResultReceiver.setValue(true);
            }

            @Override
            public boolean isDone() {
                if (mineralResultReceiver.isReady()) {
                    rightMineral=mineralResultReceiver.getValue();

                    right = rightMineral.getType();
                    mineralResultReceiver.clear();

                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return S.CHOOSE_POSITION;
            }
        });

        b.add(S.CHOOSE_POSITION, new BasicAbstractState() {
            @Override
            public void init() { }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                StateName postGoldStateName;

                if(left==GoldDetector.Detection.GOLD&&right!=GoldDetector.Detection.GOLD&&middle!=GoldDetector.Detection.GOLD){
                    goldPosition=GoldPosition.LEFT;
                }
                else if(left!=GoldDetector.Detection.GOLD&&right==GoldDetector.Detection.GOLD&&middle!=GoldDetector.Detection.GOLD){
                    goldPosition=GoldPosition.RIGHT;
                }
                else if(left!=GoldDetector.Detection.GOLD&&right!=GoldDetector.Detection.GOLD&&middle==GoldDetector.Detection.GOLD){
                    goldPosition=GoldPosition.MIDDLE;
                }
                else{
                    goldPosition=GoldPosition.UNKNOWN;
                }


                if(goldPosition==GoldPosition.LEFT){
                    postGoldStateName=S.DLG_BRANCH_START;
                }
                else if(goldPosition==GoldPosition.MIDDLE||goldPosition==GoldPosition.UNKNOWN){
                    postGoldStateName=S.DMG_BRANCH_START;
                }
                else{
                    //the only other option is right position
                    postGoldStateName=S.DRG_BRANCH_START;
                }
                return postGoldStateName;
            }
        });


//        b.add(S.STARTING_DEPOT_MIDDLE, new BasicAbstractState() {
//            StateName depot;
//            @Override
//            public void init() { }
//            @Override
//            public boolean isDone() {
//                return true;
//            }
//            @Override
//            public StateName getNextStateName() {
//                if(isStartingDepot){
//                    depot=S.DRIVE_TO_DEPOT_MORE;
//                }
//                else{
//                    depot=S.DRIVE_DEPOT_MIDDLE;
//                }
//                return depot;
//            }
//        });







// one way handle options op level decisions
//        if (isStartingDepot) {
//            buildDepotStateMachine(b);
//        } else {
//            return buildCraterSM(b);
//        }

        b.addWait(S.DMG_BRANCH_START,S.DMG_STRAFFE_1,0);
        b.addDrive(S.DMG_STRAFFE_1, S.STOP,Distance.fromFeet(5),.5,270,0);
        //  b.add()
        // b.add(S.NAME, S.NEST-join state))




        b.addWait(S.DLG_BRANCH_START,S.DLG_STRAFFE_1,0);
        b.addDrive(S.DLG_STRAFFE_1, S.STOP,Distance.fromFeet(1.4),.5,305,0);


        b.addWait(S.DRG_BRANCH_START,S.DRG_STRAFFE_1,0);
        b.addDrive(S.DRG_STRAFFE_1, S.STOP,Distance.fromFeet(1.4),.5,225 ,0);



        b.addGyroTurn(S.DRIVE_DEPOT_MIDDLE,S.STRAFE_TO_CRATER,0, 0.5); // turn back to original dirn
        b.addDrive(S.STRAFE_TO_CRATER,S.STOP,Distance.fromFeet(1.5),.5,270,0);

        b.addDrive(S.DRIVE_TO_DEPOT_MORE,S.DRIVE_TO_DEPOT_BACK,Distance.fromFeet(5),.5,270,-90);
        b.addDrive(S.DRIVE_TO_DEPOT_BACK,S.TURN_CRATER_AGAIN,Distance.fromFeet(.25),.5,90,-90);



        //left
        b.addDrive(S.LEFT_GOLD,S.STARTING_DEPOT_LEFT,Distance.fromFeet(1.4),.5,-35,270);
        b.add(S.STARTING_DEPOT_LEFT, new BasicAbstractState() {
            StateName depot;
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                if(isStartingDepot){
                    depot=S.LEFT_GOLD_FORWARD;


                }
                else{
                    depot=S.CRATER_TURN;
                }
                return depot;
            }
        });
        b.addGyroTurn(S.DEPOT_TURN_LEFT,S.STOP,-135,.5);

        b.addDrive(S.LEFT_GOLD_FORWARD,S.LEFT_GOLD_TO_MIDDLE,Distance.fromFeet(1.3),.5,270,270);
        b.addDrive(S.LEFT_GOLD_TO_MIDDLE,S.LEFT_GOLD_FORWARD_REMOVE,Distance.fromFeet(1.25),.5,215,270);
        b.addDrive(S.LEFT_GOLD_FORWARD_REMOVE,S.LEFT_GOLD_FORWARD_BACK,Distance.fromFeet(1),.5,270,270);
        b.addDrive(S.LEFT_GOLD_FORWARD_BACK,S.TURN_CRATER_AGAIN,Distance.fromFeet(1.2),.5,90,270);


        //right
        b.addDrive(S.RIGHT_GOLD,S.STARTING_DEPOT_RIGHT,Distance.fromFeet(1.4),.5,215,270);
        b.add(S.STARTING_DEPOT_RIGHT, new BasicAbstractState() {
            StateName depot;
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                if(isStartingDepot){
                    depot=S.RIGHT_GOLD_FORWARD;


                }
                else{
                    depot=S.CRATER_TURN;
                }
                return depot;
            }
        });//-135
        b.addGyroTurn(S.CRATER_TURN,S.CRATER_DRIVE,-135,.5);
        b.addDrive(S.CRATER_DRIVE,S.STOP,Distance.fromFeet(1),.5,-135,-135);
        b.addDrive(S.RIGHT_GOLD_FORWARD,S.RIGHT_GOLD_TO_MIDDLE,Distance.fromFeet(1.3),.5,270,270);
        b.addDrive(S.RIGHT_GOLD_TO_MIDDLE,S.RIGHT_GOLD_FORWARD_REMOVE,Distance.fromFeet(1.25),.5,-35,270);
        b.addDrive(S.RIGHT_GOLD_FORWARD_REMOVE,S.RIGHT_GOLD_FORWARD_BACK,Distance.fromFeet(1.5),.5,270,270);
        b.addDrive(S.RIGHT_GOLD_FORWARD_BACK,S.TURN_CRATER_AGAIN,Distance.fromFeet(1.9),.5,90,270);




        b.addGyroTurn(S.TURN_CRATER_AGAIN,S.MOVE_LITTLE,orientationDepot,.5);

        //b.addWait(S.WAIT,S.DRIVE_TO_DEPOT,500);
        b.addDrive(S.MOVE_LITTLE,S.RELEASE_MARKER,Distance.fromInches(9),.5,directionDepot,orientationDepot);
        b.addServo(S.RELEASE_MARKER,S.WAIT_MARKER,RoverRuckusRobotCfg.MainServoName.MARKER,RoverRuckusRobotCfg.MarkerPresets.RELEASE,true);
        b.addWait(S.WAIT_MARKER,S.DRIVE_TO_CRATER,Time.fromSeconds(.5));

        //b.addWait(S.WAIT,S.DRIVE_TO_DEPOT,500);
        b.addDrive(S.DRIVE_TO_CRATER,S.STOP,Distance.fromFeet(8.4),.5,-directionDepot,orientationDepot);
        // to go towards the other crater is 135 degrees; note - the gain on the gyro on the gyro control needs adjusting to keep it from
        b.addStop(S.STOP);



        return b.build();
    }

    private State createDescendState(final StateName nextStateName, final double descentTimeSec) {
        return new BasicAbstractState() {
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public void init() {
                runtime.reset();
            }

            @Override
            public boolean isDone() {
                boolean doneHanging=false;
                if (!robotCfg.getHanging().islatchLimitPressed()&&runtime.seconds() < descentTimeSec) {
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
                return nextStateName;
            }
        };
    }


    private enum S implements StateName {
        WAIT,
        TURN_CRATER,
        LEFT_GOLD,
        RIGHT_GOLD,
        MIDDLE_GOLD,
        WAIT_MARKER,
        TURN_TO_DETECT,
        TURN_BACK,
        TURN_DETECTION,
        MOVE_LITTLE,
        MOVE_FROM_LANDER,
        TURN_CRATER_AGAIN,
        DRIVE_TO_DEPOT_MORE,
        DESCEND,
        DOWN_HANGING,
        UNLATCH,
        LOCK_ROTATION,
        MOVE_OFF_HOOK,
        START,
        RELEASE_MARKER,
        ANTITIP,
        DETECT_GOLD,
        STOP,
        GOLD_ALIGN,
        DRIVE_TO_CRATER, RELEASE_LATCH
        ,LEFT_GOLD_FORWARD,
        LEFT_GOLD_TO_MIDDLE,
        RIGHT_GOLD_FORWARD,
        RIGHT_GOLD_TO_MIDDLE,
        CHOOSE_POSITION,
        DONT_HIT_GOLD_TURN,
        RIGHT_DONT_HIT_GOLD_TURN,
        COLLECT_MIDDLE,
        LEFT_GOLD_FORWARD_REMOVE, LEFT_GOLD_FORWARD_BACK, RIGHT_GOLD_FORWARD_REMOVE,
        RIGHT_GOLD_FORWARD_BACK, DRIVE_TO_DEPOT_BACK, STARTING_DEPOT, STARTING_DEPOT_RIGHT,
        STARTING_DEPOT_LEFT, DEPOT_TURN_RIGHT, DEPOT_TURN_LEFT, STARTING_DEPOT_MIDDLE,
        DRIVE_DEPOT_MIDDLE, STRAFE_TO_CRATER, DEPOT_TURN, CRATER_TURN, CRATER_DRIVE,
        //ROTATE_UP,
       DETECT_LEFT_GOLD, DETECT_RIGHT_GOLD, DETECT_MIDDLE_GOLD,
        MOVE_SERVO_LEFT_GOLD, MOVE_SERVO_RIGHT_GOLD, DRIVE_CLOSER, WAIT_1, WAIT_2, WAIT_3, LEFT_DONT_HIT_GOLD_TURN,
        DLG_BRANCH_START,DMG_BRANCH_START,DRG_BRANCH_START,
        DLG_STRAFFE_1,DMG_STRAFFE_1,DRG_STRAFFE_1
        ;


    }
}

