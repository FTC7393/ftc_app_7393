package org.firstinspires.ftc.teamcode;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.driverstation.Telem;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStates;
import ftc.evlib.vision.framegrabber.VuforiaFrameFeeder;
import ftc.evlib.vision.processors.BeaconColorResult;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 11/8/16
 */

@Autonomous(name = "VuforiaOp")
public class VuforiaOp extends AbstractAutoOp<RobotCfg> {
    private static final int FRAME_SIZE = 16;
//    private static final int FRAME_SIZE = 8;
//    private static final int FRAME_SIZE = 64;

    private static final int FRAME_WIDTH = 3 * FRAME_SIZE;
    private static final int FRAME_HEIGHT = 4 * FRAME_SIZE;
//    private static final int FRAME_WIDTH = 11 * FRAME_SIZE;
//    private static final int FRAME_HEIGHT = 9 * FRAME_SIZE;

    private enum S implements StateName {
        WAIT_FOR_VUFORIA_INIT,
        FIND_BEACON_COLOR,
        DISPLAY,
        UNKNOWN_BEACON_COLOR,
        BEACON_COLOR_TIMEOUT
    }

    private ResultReceiver<VuforiaFrameFeeder> vuforiaReceiver;
    private ResultReceiver<BeaconColorResult> beaconColorResult;

    @Override
    public StateMachine buildStates() {
        vuforiaReceiver = VuforiaFrameFeeder.initInNewThread("ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX", R.id.cameraMonitorViewId, FRAME_WIDTH, FRAME_HEIGHT);

        StateMachineBuilder b = new StateMachineBuilder(S.WAIT_FOR_VUFORIA_INIT);
        beaconColorResult = new BasicResultReceiver<>();

        b.addEmpty(S.WAIT_FOR_VUFORIA_INIT, b.t(S.FIND_BEACON_COLOR, EndConditions.receiverReady(vuforiaReceiver)));
        b.add(S.FIND_BEACON_COLOR, EVStates.findBeaconColorState(S.DISPLAY, S.UNKNOWN_BEACON_COLOR, S.BEACON_COLOR_TIMEOUT, Time.fromSeconds(10), vuforiaReceiver, beaconColorResult, TeamColor.RED, 4, true));

        b.add(S.DISPLAY, EVStates.displayBeaconColorResult(beaconColorResult));
        b.add(S.UNKNOWN_BEACON_COLOR, EVStates.displayBeaconColorResult(beaconColorResult));
        b.add(S.BEACON_COLOR_TIMEOUT, EVStates.displayBeaconColorResult(beaconColorResult));

        return b.build();
    }

    @Override
    protected RobotCfg createRobotCfg() {
        return RobotCfg.fake(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return new Logger("vuforiaOp", ".csv", ImmutableList.of(
                new Logger.Column("state", new InputExtractor<StateName>() {
                    @Override
                    public StateName getValue() {
                        return stateMachine.getCurrentStateName();
                    }
                }),
                new Logger.Column("beaconColor", new InputExtractor<BeaconColorResult>() {
                    @Override
                    public BeaconColorResult getValue() {
                        return beaconColorResult.getValue();
                    }
                })
        ));
    }

    @Override
    protected void setup_act() {
//        Telem.displayVuforiaReadiness(vuforiaReceiver);
    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        telemetry.addData("state", stateMachine.getCurrentStateName());
//        Telem.displayVuforiaReadiness(vuforiaReceiver);
    }

    @Override
    protected void end() {

    }
}
