package org.firstinspires.ftc.teamcode.relic2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

/**
 * Created by ftc7393 on 12/3/2017.
 */
@TeleOp(name = "ServoTuneOp")

public class ServoTuneOp extends AbstractServoTuneOp {

    @Override
    protected RobotCfg createRobotCfg() {
        return new RobotCfg2017(hardwareMap);
    }

}
