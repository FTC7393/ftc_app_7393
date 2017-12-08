package org.firstinspires.ftc.teamcode.relic2017;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

import ftc.electronvolts.util.files.BasicConverters;
import ftc.electronvolts.util.files.Converters;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.Sensors;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/2/17
 */

public class RobotCfg2017 extends RobotCfg {
    private final MecanumControl mecanumControl;
    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));
//    private final BNO055IMU imu;
    private final Collector collector;
    private final Servos servos;

    public Collector getCollector() {
        return collector;
    }

//    public MainRobotCfg(HardwareMap hardwareMap) {
//        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
//    }

    public enum ReleaseServoPresets{
        CLOSED,
        OPENED
    }
    public enum SensorServoPresets{
        UP,
        DOWN
    }
    public enum MainServoName implements ServoName{
        RELEASE("s0",ReleaseServoPresets.values()),
        SENSOR("s1",SensorServoPresets.values());
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
            return RobotCfg2017.class;
        }
    }
    public RobotCfg2017(HardwareMap hardwareMap){
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
    }


    public RobotCfg2017(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        servos=new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
        collector=new Collector( Motors.withEncoder(hardwareMap.dcMotor.get("4"), true, true, stoppers),
                Motors.withEncoder(hardwareMap.dcMotor.get("5"), true, true, stoppers),
                Motors.withoutEncoder(hardwareMap.dcMotor.get("6"), true, true, stoppers),
                getServo(MainServoName.RELEASE),
                Sensors.inv(Sensors.digital(hardwareMap,"d1")), //topLimit
                Sensors.inv(Sensors.digital(hardwareMap,"d2")),//bottomLimit
                Sensors.inv(Sensors.digital(hardwareMap,"d0")));//tiltLimit


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        Converters converters = EVConverters.getInstance();
        OptionsFile optionsFile = new OptionsFile(converters, FileUtil.getOptionsFile("teleop_options.txt"));
        double scaleFactor = optionsFile.get("mratio",Double.class);

        ServoControl colorServo = getServo(MainServoName.SENSOR);
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("0"), true, true, stoppers),
                Motors.withEncoder(hardwareMap.dcMotor.get("1") , false, true, stoppers),
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("2") , true, true, stoppers),scaleFactor),
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("3") , false, true, stoppers),scaleFactor),
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));

    }
    public Servos getServos() {
        return servos;
    }

    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

    @Override
    public void start() {

    }

    @Override
    public void act() {
        collector.act();
        mecanumControl.act();
    }
    @Override
    public void stop() {
        mecanumControl.stop();
    }


}
