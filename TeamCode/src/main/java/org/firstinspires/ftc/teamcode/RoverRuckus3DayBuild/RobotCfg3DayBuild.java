package org.firstinspires.ftc.teamcode.RoverRuckus3DayBuild;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

import evlib.hardware.config.RobotCfg;
import evlib.hardware.motors.Motors;
import evlib.hardware.motors.TwoMotors;
import evlib.hardware.servos.ServoCfg;
import evlib.hardware.servos.ServoControl;
import evlib.hardware.servos.ServoName;
import evlib.hardware.servos.Servos;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

/**
 * Created by ftc7393 on 9/8/2018.
 */

public class RobotCfg3DayBuild extends RobotCfg{
    private final Servos servos;
    private final Mechanisms mechanisms;
    private static final String COLOR_SENSOR_NAME = "sensor_color";
    private final ColorSensor colorSensor;


    private static final Velocity maxRobotSpeed = new Velocity(Distance.fromFeet(1), Time.fromSeconds(1));

    public ColorSensor getColorSensor() {
        return colorSensor;
    }

    public RobotCfg3DayBuild(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get(COLOR_SENSOR_NAME);

        servos=new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        mechanisms = new Mechanisms(Motors.withEncoder(hardwareMap.dcMotor.get("slide"),false,true,stoppers),Motors.withEncoder(hardwareMap.dcMotor.get("left"),false,true,stoppers),Motors.withEncoder(hardwareMap.dcMotor.get("right"),false,true,stoppers), getServo(MainServoName.MARKER),getServo(MainServoName.LATCH));
    }
    public Servos getServos() {
        return servos;
    }



    public enum MarkerPresets {
        CLAMP,
        RELEASE
    }
    public enum LatchPresets {
        CLOSE,
        OPEN
    }

    public enum MainServoName implements ServoName{
        MARKER("marker",MarkerPresets.values()),
        LATCH("latch",LatchPresets.values());


        private final String hardwareName;

        private final Enum[] presets;

        MainServoName(String hardwareName, Enum[] presets) {
            this.hardwareName = hardwareName;
            this.presets = presets;
        }

        @Override
        public String getHardwareName() {
            return hardwareName;
        }

        @Override
        public Enum[] getPresets() {
            return presets;
        }

        @Override
        public Class<? extends RobotCfg> getRobotCfg() {
            return RobotCfg3DayBuild.class;
        }
    }

    public RobotCfg3DayBuild(HardwareMap hardwareMap){
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
    }

    @Override
    public void start() {

    }

    @Override
    public void act() {
        mechanisms.act();

    }

    @Override
    public void stop() {

    }
    public Mechanisms getMechanisms() {
        return mechanisms;
    }


}

