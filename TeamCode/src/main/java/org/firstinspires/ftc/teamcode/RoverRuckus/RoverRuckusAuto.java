package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;

import org.firstinspires.ftc.teamcode.softwareTests.GoldAlignExample;
import org.firstinspires.ftc.teamcode.softwareTests.SamplingOrderExample;

import evlib.opmodes.AbstractOp;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 10/13/2018.
 */

public class RoverRuckusAuto extends AbstractOp<RoverRuckusRobotCfg> {
    private StateName state = StateName.RELEASE_LATCH;

    private enum StateName {
        START,
        DETECT_GOLD,

        GOLD_ALIGN, RELEASE_LATCH

    }
    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
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
        final ResultReceiver<SamplingOrderDetector.GoldLocation> goldLocationReceiver= SamplingOrderExample.initThread(hardwareMap);

        SamplingOrderDetector.GoldLocation goldLocation;
        if(state==StateName.DETECT_GOLD){
            while(SamplingOrderDetector.GoldLocation.UNKNOWN==goldLocationReceiver.getValue()) {
                goldLocation = goldLocationReceiver.getValue();
            }
            state=StateName.GOLD_ALIGN;



        }

//        if(state==StateName.GOLD_ALIGN) {
//            while (GoldAlignExample.getXpos() < 250) {
//
//
//            }
//            while (GoldAlignExample.getXpos() > 350) {
//
//
//            }
//
//        }






    }

    @Override
    protected void post_act() {

    }

    @Override
    protected void end() {

    }

    @Override
    protected Time getMatchTime() {
        return null;
    }
}
