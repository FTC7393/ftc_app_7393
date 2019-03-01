package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

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
//        if (potentialMineralResultReceiver.isReady()) {
//            List<Mineral> mlist = potentialMineralResultReceiver.getValue();
//            for (Mineral m : mlist) {
//                m.showInTelem(telemetry);
//            }
//        }

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
        double waitForAlliance = optionsFile.get(RoverRuckusOptionsOp.wait,RoverRuckusOptionsOp.waitDefault);

        boolean doPartnerSample = true; //this needs to be in the options op
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

        //!!!!!!!!!! CHANGE START CONDITION
        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.WAIT_2, teamColor, Angle.fromDegrees(3));//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
                return S.STOW_SERVO;
            }
        });
        b.addServo(S.STOW_SERVO,S.CHOOSE_POSITION,RoverRuckusRobotCfg.MainServoName.PHONEPAN,RoverRuckusRobotCfg.PhonePanPresets.STOW,panSpeed,false);

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

               goldPosition= new MineralDecisionMaker().decidePosition(left, middle, right);

                if (isStartingDepot){
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
                } else { //crater start
                    if(goldPosition==GoldPosition.LEFT){
                        postGoldStateName=S.CLG_BRANCH_START;
                    }
                    else if(goldPosition==GoldPosition.MIDDLE||goldPosition==GoldPosition.UNKNOWN){
                        postGoldStateName=S.CMG_BRANCH_START;
                    }
                    else{
                        //the only other option is right position
                        postGoldStateName=S.CRG_BRANCH_START;
                    }
                }
                return postGoldStateName;
            }
        });






// one way handle options op level decisions
        if (isStartingDepot) {
            buildDepotStateMachine(b, moveToOpponentCrater, doPartnerSample, waitForAlliance);
        } else {
            buildCraterSM(b, moveToOpponentCrater, waitForAlliance);
        }
        b.addStop(S.STOP);


        return b.build();
    }

    private void buildCraterSM(EVStateMachineBuilder b, boolean moveToOpponentCrater, double waitForAlliance) {
        b.addWait(S.CMG_BRANCH_START,S.CMG_STRAFFE_1,0);
        b.addDrive(S.CMG_STRAFFE_1, S.CMG_DRIVE_2,Distance.fromFeet(.85),.5,270,0);
        b.addDrive(S.CMG_DRIVE_2, S.CMG_DRIVE_3,Distance.fromFeet(.45),.5,90,0);
        b.addDrive(S.CMG_DRIVE_3, S.C_DRIVE_0,Distance.fromFeet(2),.5,0,0);

        b.addWait(S.CLG_BRANCH_START,S.CLG_DRIVE_1,0);
        b.addDrive(S.CLG_DRIVE_1, S.CLG_DRIVE_2,Distance.fromFeet(1.52),.5,310,0);
        b.addDrive(S.CLG_DRIVE_2, S.CLG_DRIVE_3,Distance.fromFeet(.8),.25,90,0);
        b.addDrive(S.CLG_DRIVE_3, S.C_DRIVE_0,Distance.fromFeet(.77),.25,180,0);


        b.addWait(S.CRG_BRANCH_START,S.CRG_DRIVE_1,0);
        b.addDrive(S.CRG_DRIVE_1, S.CRG_DRIVE_2,Distance.fromFeet(1.45),.5,225,0);
        b.addDrive(S.CRG_DRIVE_2, S.CRG_DRIVE_3,Distance.fromFeet(.43),.5,90,0);
        b.addDrive(S.CRG_DRIVE_3, S.C_DRIVE_0,Distance.fromFeet(4.25),.5,0,0);



        //Common for crater from now until end on method

        b.addDrive(S.C_DRIVE_0,S.C_TURN_1,Distance.fromFeet(2),.7,0,0);
        b.addGyroTurn(S.C_TURN_1,S.C_DRIVE_1, 45,.5);
        b.addDrive(S.C_DRIVE_1,S.C_DRIVE_2,Distance.fromFeet(.9),.5,315,45);
        b.addDrive(S.C_DRIVE_2,S.C_ALLIANCE_WAIT,Distance.fromFeet(.05),.5,135,45);
        b.addWait(S.C_ALLIANCE_WAIT, S.C_DRIVE_3, Time.fromSeconds(waitForAlliance));
        b.addDrive(S.C_DRIVE_3,S.C_DROP_MARKER,Distance.fromFeet(7),.9,45,45);
        b.addServo(S.C_DROP_MARKER, S.C_WAIT_1,RoverRuckusRobotCfg.MainServoName.MARKER,RoverRuckusRobotCfg.MarkerPresets.RELEASE,true);
        b.addWait(S.C_WAIT_1, S.C_LEVER_UP, Time.fromSeconds(0.5));
        b.addServo(S.C_LEVER_UP,S.C_BACK_1,RoverRuckusRobotCfg.MainServoName.MARKER,RoverRuckusRobotCfg.MarkerPresets.HOLD,false);
        b.addDrive(S.C_BACK_1,S.C_DRIVE_4,Distance.fromFeet(1),.7,225,45);
        b.addDrive(S.C_DRIVE_4,S.C_TURN_2,Distance.fromFeet(.35),.5,135,45);

        if(moveToOpponentCrater){
            b.addGyroTurn(S.C_TURN_2,S.STOP, 90,.5);
            b.addDrive(S.C_DRIVE_5,S.C_TURN_3,Distance.fromFeet(1),.5,90,90);
            b.addGyroTurn(S.C_TURN_3,S.C_DRIVE_6, 135,.5);
            b.addDrive(S.C_DRIVE_6,S.C_DRIVE_6A,Distance.fromFeet(.3),.5,45,135);
            b.addDrive(S.C_DRIVE_6A,S.C_DRIVE_6B,Distance.fromFeet(.2),.5,225,135);
            b.addDrive(S.C_DRIVE_6B,S.STOP,Distance.fromFeet(7.5),1,135,135);
        } else {
            b.addGyroTurn(S.C_TURN_2,S.C_DRIVE_5, 225,.5);
            b.addDrive(S.C_DRIVE_5,S.STOP,Distance.fromFeet(.3),.5,315,225);
            b.addDrive(S.C_DRIVE_5,S.STOP,Distance.fromFeet(.05),.5,135,225);
            b.addDrive(S.C_DRIVE_5,S.STOP,Distance.fromFeet(8),.9,225,225);

        }



    }

    private void buildDepotStateMachine(EVStateMachineBuilder b, boolean moveToOpponentCrater, boolean doPartnerSample, double waitForAlliance) {
        //This is the depot side auto with gold detected in the middle
        b.addWait(S.DMG_BRANCH_START,S.DMG_STRAFFE_1,Time.fromSeconds(waitForAlliance));
        b.addDrive(S.DMG_STRAFFE_1, S.D_BACKUP_1,Distance.fromFeet(1.95),.5,270,0);
        b.addDrive(S.D_BACKUP_1, S.DMG_TURN1,Distance.fromFeet(.3),.5,90,0);
        b.addGyroTurn(S.DMG_TURN1,S.D_DROP_MARKER,270, 0.5);
        //gold on left side for depot
        b.addWait(S.DLG_BRANCH_START,S.DLG_STRAFFE_1,Time.fromSeconds(waitForAlliance));
        b.addDrive(S.DLG_STRAFFE_1, S.DLG_STRAFFE_2,Distance.fromFeet(1.65),.5,310,0);
        b.addDrive(S.DLG_STRAFFE_2,S.DLG_DRIVE_1,Distance.fromFeet(.3),.5,270,0);
        b.addDrive(S.DLG_DRIVE_1,S.DLG_DRIVE_2,Distance.fromFeet(1.8),.5,225,0);
        b.addDrive(S.DLG_DRIVE_2,S.DLG_TURN_2,Distance.fromFeet(.3),.5,90,0);
        b.addGyroTurn(S.DLG_TURN_2,S.DLG_DRIVE_4,270, 0.5);
        b.addDrive(S.DLG_DRIVE_4,S.D_DROP_MARKER,Distance.fromFeet(.1),.5,270,270);
        //gold on right side for depot
        b.addWait(S.DRG_BRANCH_START,S.DRG_STRAFFE_1,Time.fromSeconds(waitForAlliance));
        b.addDrive(S.DRG_STRAFFE_1, S.DRG_STRAFE_2,Distance.fromFeet(1.8),.5,225 ,0);
        b.addDrive(S.DRG_STRAFE_2, S.DRG_STRAFE_3,Distance.fromFeet(.2),.5,270 ,0);
        b.addDrive(S.DRG_STRAFE_3, S.DRG_STRAFE_4,Distance.fromFeet(1.4),.5,315 ,0);
        b.addDrive(S.DRG_STRAFE_4, S.DRG_STRAFE_5,Distance.fromFeet(.3),.5,270 ,0);
        b.addDrive(S.DRG_STRAFE_5, S.DRG_TURN_1,Distance.fromFeet(.2),.5,90 ,0);
        b.addGyroTurn(S.DRG_TURN_1, S.D_DROP_MARKER,270, .5);

        //this is common across all branches for depot
        b.addServo(S.D_DROP_MARKER, S.D_WAIT_1,RoverRuckusRobotCfg.MainServoName.MARKER,RoverRuckusRobotCfg.MarkerPresets.RELEASE,true);
        b.addWait(S.D_WAIT_1, S.D_BACKUP_2, Time.fromSeconds(0.5));
        b.addDrive(S.D_BACKUP_2, S.D_CLOSE_SERVO, Distance.fromFeet(.5),.5,90,270);
        b.addServo(S.D_CLOSE_SERVO, S.D_TURN_CRATER,RoverRuckusRobotCfg.MainServoName.MARKER,RoverRuckusRobotCfg.MarkerPresets.HOLD,false);
        if (moveToOpponentCrater) {
            b.addGyroTurn(S.D_TURN_CRATER,S.D_OPP_DRIVE_1,45, 0.5);
            b.addDrive(S.D_OPP_DRIVE_1,S.D_OPP_DRIVE_2,Distance.fromFeet(.65),.5,315,45);
            b.addDrive(S.D_OPP_DRIVE_2,S.D_OPP_DRIVE_3,Distance.fromFeet(.03),.1,135,45);
            b.addDrive(S.D_OPP_DRIVE_3,S.STOP,Distance.fromFeet(8),.8,45,45);
        }
        else {
            b.addGyroTurn(S.D_TURN_CRATER,S.D_OUR_DRIVE_1,135, 0.5);
            b.addDrive(S.D_OUR_DRIVE_1,S.D_OUR_DRIVE_2,Distance.fromFeet(.65),.5,225,135);
            b.addDrive(S.D_OUR_DRIVE_2,S.D_OUR_DRIVE_3,Distance.fromFeet(.03),.5,45,135);
            b.addDrive(S.D_OUR_DRIVE_3,S.D_SAMPLE_DECISION,Distance.fromFeet(8),.8,135,135);
            if(doPartnerSample) {
                b.add(S.D_SAMPLE_DECISION, decideSample(S.SAMPLE_PARTNER, S.STOP));
                b.addDrive(S.SAMPLE_PARTNER, S.SP_DRIVE_1,Distance.fromFeet(0.3),0.5,315,135);
                b.addDrive(S.SP_DRIVE_1, S.SP_DRIVE_2,Distance.fromFeet(0.5),0.5,90,135);
                b.addDrive(S.SP_DRIVE_2, S.STOP,Distance.fromFeet(0.8),0.5,180,135);
            }
            else {
                b.addWait(S.D_SAMPLE_DECISION,S.STOP,0);
            }



        }


        b.addGyroTurn(S.DRIVE_DEPOT_MIDDLE,S.STRAFE_TO_CRATER,0, 0.5); // turn back to original dirn
        b.addDrive(S.STRAFE_TO_CRATER,S.STOP,Distance.fromFeet(1.5),.5,270,0);

        b.addDrive(S.DRIVE_TO_DEPOT_MORE,S.DRIVE_TO_DEPOT_BACK,Distance.fromFeet(5),.5,270,-90);
        b.addDrive(S.DRIVE_TO_DEPOT_BACK,S.TURN_CRATER_AGAIN,Distance.fromFeet(.25),.5,90,-90);

    }

    private State decideSample(final StateName keepGoingState, final StateName endState) {
        return new BasicAbstractState() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                if(goldPosition == GoldPosition.LEFT)
                    return keepGoingState;
                return endState;
            }
        };
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
        CLG_BRANCH_START,CMG_BRANCH_START,CRG_BRANCH_START,
        DLG_STRAFFE_1,DMG_STRAFFE_1,DRG_STRAFFE_1,
        CLG_STRAFFE_1,CMG_STRAFFE_1,CRG_STRAFFE_1,


        DMG_TURN1, D_DROP_MARKER, D_BACKUP_2, D_BACKUP_1, D_WAIT_1, D_OPP_DRIVE_1, D_OPP_DRIVE_2, D_OPP_DRIVE_3,
        D_OUR_DRIVE_1, D_TURN_CRATER, D_OUR_DRIVE_2, D_OUR_DRIVE_3, DLG_STRAFFE_2, DLG_TURN_1,
        DLG_DRIVE_1, DLG_DRIVE_2, DLG_DRIVE_4, DLG_TURN_2,
        DRG_TURN_1, DRG_STRAFE_5, DRG_STRAFE_4, DRG_STRAFE_3, CMG_DRIVE_2, CMG_DRIVE_3, C_TURN_1,
        CLG_DRIVE_1, CLG_DRIVE_2, CLG_DRIVE_3, CRG_DRIVE_1, CRG_DRIVE_2, CRG_DRIVE_3, C_DRIVE_1,
        C_DRIVE_2, C_DRIVE_3, C_DROP_MARKER, C_WAIT_1, C_DRIVE_4, C_DRIVE_6, C_TURN_3, C_TURN_2,
        C_DRIVE_5, C_DRIVE_6A, C_DRIVE_6B, D_CLOSE_SERVO, D_SAMPLE_DECISION, SAMPLE_PARTNER, SP_DRIVE_1, SP_DRIVE_2, C_DRIVE_0, STOW_SERVO, C_LEVER_UP, C_BACK_1, C_ALLIANCE_WAIT, DRG_STRAFE_2


    }
}









//        //left
//        b.addDrive(S.LEFT_GOLD,S.STARTING_DEPOT_LEFT,Distance.fromFeet(1.4),.5,-35,270);
//        b.add(S.STARTING_DEPOT_LEFT, new BasicAbstractState() {
//            StateName depot;
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if(isStartingDepot){
//                    depot=S.LEFT_GOLD_FORWARD;
//
//
//                }
//                else{
//                    depot=S.CRATER_TURN;
//                }
//                return depot;
//            }
//        });
//        b.addGyroTurn(S.DEPOT_TURN_LEFT,S.STOP,-135,.5);
//
//        b.addDrive(S.LEFT_GOLD_FORWARD,S.LEFT_GOLD_TO_MIDDLE,Distance.fromFeet(1.3),.5,270,270);
//        b.addDrive(S.LEFT_GOLD_TO_MIDDLE,S.LEFT_GOLD_FORWARD_REMOVE,Distance.fromFeet(1.25),.5,215,270);
//        b.addDrive(S.LEFT_GOLD_FORWARD_REMOVE,S.LEFT_GOLD_FORWARD_BACK,Distance.fromFeet(1),.5,270,270);
//        b.addDrive(S.LEFT_GOLD_FORWARD_BACK,S.TURN_CRATER_AGAIN,Distance.fromFeet(1.2),.5,90,270);
//
//
//        //right
//        b.addDrive(S.RIGHT_GOLD,S.STARTING_DEPOT_RIGHT,Distance.fromFeet(1.4),.5,215,270);
//        b.add(S.STARTING_DEPOT_RIGHT, new BasicAbstractState() {
//            StateName depot;
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if(isStartingDepot){
//                    depot=S.RIGHT_GOLD_FORWARD;
//
//
//                }
//                else{
//                    depot=S.CRATER_TURN;
//                }
//                return depot;
//            }
//        });//-135
//        b.addGyroTurn(S.CRATER_TURN,S.CRATER_DRIVE,-135,.5);
//        b.addDrive(S.CRATER_DRIVE,S.STOP,Distance.fromFeet(1),.5,-135,-135);
//        b.addDrive(S.RIGHT_GOLD_FORWARD,S.RIGHT_GOLD_TO_MIDDLE,Distance.fromFeet(1.3),.5,270,270);
//        b.addDrive(S.RIGHT_GOLD_TO_MIDDLE,S.RIGHT_GOLD_FORWARD_REMOVE,Distance.fromFeet(1.25),.5,-35,270);
//        b.addDrive(S.RIGHT_GOLD_FORWARD_REMOVE,S.RIGHT_GOLD_FORWARD_BACK,Distance.fromFeet(1.5),.5,270,270);
//        b.addDrive(S.RIGHT_GOLD_FORWARD_BACK,S.TURN_CRATER_AGAIN,Distance.fromFeet(1.9),.5,90,270);
//
//
//
//
//        b.addGyroTurn(S.TURN_CRATER_AGAIN,S.MOVE_LITTLE,orientationDepot,.5);
//
//        //b.addWait(S.WAIT,S.DRIVE_TO_DEPOT,500);
//        b.addDrive(S.MOVE_LITTLE,S.RELEASE_MARKER,Distance.fromInches(9),.5,directionDepot,orientationDepot);
//        b.addServo(S.RELEASE_MARKER,S.WAIT_MARKER,RoverRuckusRobotCfg.MainServoName.MARKER,RoverRuckusRobotCfg.MarkerPresets.RELEASE,true);
//        b.addWait(S.WAIT_MARKER,S.DRIVE_TO_CRATER,Time.fromSeconds(.5));
//
//        //b.addWait(S.WAIT,S.DRIVE_TO_DEPOT,500);
//        b.addDrive(S.DRIVE_TO_CRATER,S.STOP,Distance.fromFeet(8.4),.5,-directionDepot,orientationDepot);
//        // to go towards the other crater is 135 degrees; note - the gain on the gyro on the gyro control needs adjusting to keep it from
//        b.addStop(S.STOP);
//




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


