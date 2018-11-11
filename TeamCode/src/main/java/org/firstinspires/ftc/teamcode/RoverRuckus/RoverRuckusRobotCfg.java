package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.relic2017.Mechanisms.IMUGyro;

import java.util.Map;

import evlib.hardware.config.RobotCfg;
import evlib.hardware.control.MecanumControl;
import evlib.hardware.motors.MecanumMotors;
import evlib.hardware.motors.Motors;
import evlib.hardware.sensors.Sensors;
import evlib.hardware.servos.ServoCfg;
import evlib.hardware.servos.ServoName;
import evlib.hardware.servos.Servos;
import ftc.electronvolts.util.PIDController;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

/**
 * Created by ftc7393 on 9/29/2018.
 */

public class RoverRuckusRobotCfg extends RobotCfg {
    private final ColorSensor leftColorSensor;
    private final ColorSensor rightColorSensor;
    private final MecanumControl mecanumControl;

//    private final Collector collector;
    private final Arm arm;
    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));
    private static final String LEFT_COLOR_SENSOR_NAME = "colorBackLeft";
    private static final String RIGHT_COLOR_SENSOR_NAME = "colorBackRight";
    private IMUGyro gyro;
    private final Servos servos;


    public RoverRuckusRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));


        leftColorSensor = hardwareMap.colorSensor.get(LEFT_COLOR_SENSOR_NAME);
        rightColorSensor = hardwareMap.colorSensor.get(RIGHT_COLOR_SENSOR_NAME);
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), false, true, stoppers),
                Motors.withEncoder(hardwareMap.dcMotor.get("frontRight") , false, true, stoppers),
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backLeft") , true, true, stoppers),scaleFactor),
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backRight") , true, true, stoppers),scaleFactor),
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));
        gyro = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu"));
        arm=new Arm(
                Motors.withEncoder(hardwareMap.dcMotor.get("extension"),false,true,stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"extensionLimit")),
                Motors.withEncoder(hardwareMap.dcMotor.get("rotation"),false,true,stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"rotationLimit")));


//hi




    }
    public enum doorPresets{
        CLOSE,
        RIGHT,
        LEFT
    }
    public enum MainServoName implements ServoName {
        DOOR("door",doorPresets.values()),;

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
            return RoverRuckusRobotCfg.class;
        }
    }
    public RoverRuckusRobotCfg(HardwareMap hardwareMap){
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
    }
    public ColorSensor getLeftColorSensor() {
        return leftColorSensor;
    }
    public ColorSensor getRightColorSensor() {
        return rightColorSensor;
    }
    public IMUGyro getGyro() {
        return gyro;
    }


    @Override
    public void start() {

    }

    @Override
    public void act() {
        mecanumControl.act();
        arm.act();

    }

    @Override
    public void stop() {
        mecanumControl.stop();
        gyro.stop();

    }
    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }
    public Arm getArm(){
        return arm;
    }

}
