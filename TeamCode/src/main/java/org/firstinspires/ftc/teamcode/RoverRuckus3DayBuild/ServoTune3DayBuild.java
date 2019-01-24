package org.firstinspires.ftc.teamcode.RoverRuckus3DayBuild;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import evlib.hardware.config.RobotCfg;
import evlib.opmodes.AbstractServoTuneOp;

/**
 * Created by ftc7393 on 12/3/2017.
 */
@TeleOp(name = "ServoTune3DayBuild")
@Disabled
public class ServoTune3DayBuild extends AbstractServoTuneOp {

    @Override
    protected RobotCfg createRobotCfg() {
        return new RobotCfg3DayBuild(hardwareMap);
    }

}
