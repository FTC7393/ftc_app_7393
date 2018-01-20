package org.firstinspires.ftc.teamcode.relic2017;

import android.util.Log;

import com.google.common.collect.ImmutableList;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Map;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.BasicConverters;
import ftc.electronvolts.util.files.Converters;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.Sensors;
import ftc.evlib.hardware.sensors.SpikeDetector;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.util.StepTimer;

import static ftc.evlib.vision.framegrabber.GlobalFrameGrabber.frameGrabber;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/2/17
 */

public class RobotCfg2017 extends RobotCfg {
    private final MecanumControl mecanumControl;
    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));
//    private final BNO055IMU imu;
    private final Grabber grabber;
    private final Relic relic;


    private final Servos servos;
    private static final String GYRO_SENSOR_NAME = "gyro";
    private static final String COLOR_SENSOR_NAME = "sensor_color";
    private final ColorSensor colorSensor;
    private final SpikeDetector rightLineSensor, leftLineSensor;
    private static final String RIGHT_LINE_SENSOR_NAME = "l1";
    private static final String LEFT_LINE_SENSOR_NAME = "l2";
    private static final double LINE_SENSOR_SPIKE_THRESHOLD = 1.05; //1.1; //1.25;

    private static final int LINE_SENSOR_LONG_READINGS = 50;
    private static final int LINE_SENSOR_SHORT_READINGS = 3;
    private final List<Logger.Column> loggerColumns;


    private IMUGyro gyro;


    public Grabber getGrabber() {
        return grabber;
    }
    public Relic getRelic() {
        return relic;
    }

    public ColorSensor getColorSensor() {
        return colorSensor;
    }

    public List<Logger.Column> getLoggerColumns() {
        return loggerColumns;
    }

    public IMUGyro getGyro() {
        return gyro;
    }

//    public MainRobotCfg(HardwareMap hardwareMap) {
//        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
//    }

    public enum LeftReleaseServoPresets{
        OPENED,
        CLOSED,
        GRAB,

    }
    public enum RightReleaseServoPresets{
        OPENED,
        CLOSED,
        GRAB,

    }
    public enum SensorServoPresets{
        UP,
        DOWN
    }
    public enum relicServoPresets{
        OPEN,
        GRAB
    }
    public enum MainServoName implements ServoName{
        LEFTRELEASE("s0",LeftReleaseServoPresets.values()),
        RIGHTRELEASE("s2",RightReleaseServoPresets.values()),
        SENSOR("s1",SensorServoPresets.values()),
        GRAB("grab",relicServoPresets.values());
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
        colorSensor = hardwareMap.colorSensor.get(COLOR_SENSOR_NAME);

        servos=new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//         imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
        grabber=new Grabber(Motors.withEncoder(hardwareMap.dcMotor.get("4"), true, true, stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"d1")), //topLimit
                Sensors.inv(Sensors.digital(hardwareMap,"d2")),//bottomLimit
                getServo(MainServoName.LEFTRELEASE),
                getServo(MainServoName.RIGHTRELEASE));

        relic= new Relic(Motors.withEncoder(hardwareMap.dcMotor.get("relic"), true, true, stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"d3")),//bottomLimit
                getServo(MainServoName.GRAB));
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        Converters converters = EVConverters.getInstance();
        OptionsFile optionsFile = new OptionsFile(converters, FileUtil.getOptionsFile("teleop_options.txt"));
        double scaleFactor = 1.0; // optionsFile.get("mratio",Double.class);

        gyro = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu"));
        leftLineSensor = Sensors.spikeDetector(hardwareMap, LEFT_LINE_SENSOR_NAME, LINE_SENSOR_SPIKE_THRESHOLD, LINE_SENSOR_LONG_READINGS, LINE_SENSOR_SHORT_READINGS);
        rightLineSensor = Sensors.spikeDetector(hardwareMap, RIGHT_LINE_SENSOR_NAME, LINE_SENSOR_SPIKE_THRESHOLD, LINE_SENSOR_LONG_READINGS, LINE_SENSOR_SHORT_READINGS);

        ServoControl colorServo = getServo(MainServoName.SENSOR);
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("0"), true, true, stoppers),
                Motors.withEncoder(hardwareMap.dcMotor.get("1") , false, true, stoppers),
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("2") , true, true, stoppers),scaleFactor),
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("3") , false, true, stoppers),scaleFactor),
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));

        loggerColumns = ImmutableList.of(
                //robot motion
                new Logger.Column("velocityX", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getVelocityX();
                    }
                }),
                new Logger.Column("velocityY", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getVelocityY();
                    }
                }),
                new Logger.Column("velocityR", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getVelocityR();
                    }
                }),
                new Logger.Column("scaleFactor", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getMecanumMotors().getScaleFactor();
                    }
                }),
                //gyro
                new Logger.Column("gyro heading", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return gyro.getHeading();
                    }
                }),

                //servos
                new Logger.Column("LEFTRELEASE", getServos().servoIE(MainServoName.LEFTRELEASE)),
                new Logger.Column("RIGHTRELEASE", getServos().servoIE(MainServoName.RIGHTRELEASE)),
                new Logger.Column("SENSOR", getServos().servoIE(MainServoName.SENSOR)),

//                new Logger.Column("distanceSensor", InputExtractors.format("%10f", distanceSensor)),

                //shooter
//                new Logger.Column("shooterSwitchInv", InputExtractors.booleanToIntIE(shooterSwitchInv)),
//                new Logger.Column("shooterSwitchBoth", InputExtractors.booleanToIntIE(shooterSwitchBoth)),



                //sensors
                new Logger.Column("leftLineSensorRaw", leftLineSensor.getRawSensor()),
                new Logger.Column("rightLineSensorRaw", rightLineSensor.getRawSensor()),
                new Logger.Column("leftLineSensor", InputExtractors.booleanToIntIE(leftLineSensor)),
                new Logger.Column("rightLineSensor", InputExtractors.booleanToIntIE(rightLineSensor)),
                new Logger.Column("frontLineSensor.ready", InputExtractors.booleanToIntIE(new InputExtractor<Boolean>() {
                    @Override
                    public Boolean getValue() {
                        return leftLineSensor.isReady();
                    }
                })),
                new Logger.Column("rightLineSensor.ready", InputExtractors.booleanToIntIE(new InputExtractor<Boolean>() {
                    @Override
                    public Boolean getValue() {
                        return rightLineSensor.isReady();
                    }
                })),
                new Logger.Column("leftLineSensor.longAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return leftLineSensor.getLongAverage();
                    }
                }),
                new Logger.Column("rightLineSensor.longAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return rightLineSensor.getLongAverage();
                    }
                }),
                new Logger.Column("leftLineSensor.shortAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return leftLineSensor.getShortAverage();
                    }
                }),
                new Logger.Column("rightLineSensor.shortAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return rightLineSensor.getShortAverage();
                    }
                }),

                //motor modes
//                new Logger.Column("flMode", Motors.modeIE(flMotor)),
//                new Logger.Column("frMode", Motors.modeIE(frMotor)),
//                new Logger.Column("blMode", Motors.modeIE(blMotor)),
//                new Logger.Column("brMode", Motors.modeIE(brMotor)),
//                new Logger.Column("shooterMotorMode", Motors.modeIE(shooterMotor)),
//                new Logger.Column("collectorMode", Motors.modeIE(collectorMotor)),
//                new Logger.Column("liftMode", Motors.modeIE(capBallLiftMotor)),
//
//                //motor powers
//                new Logger.Column("flPower", Motors.powerIE(flMotor)),
//                new Logger.Column("frPower", Motors.powerIE(frMotor)),
//                new Logger.Column("blPower", Motors.powerIE(blMotor)),
//                new Logger.Column("brPower", Motors.powerIE(brMotor)),
//                new Logger.Column("shooterPower", Motors.powerIE(shooterMotor)),
//                new Logger.Column("collectorPower", Motors.powerIE(collectorMotor)),
//                new Logger.Column("liftPower", Motors.powerIE(capBallLiftMotor)),
//
//                //motor encoders
//                new Logger.Column("flEnc", Motors.encoderIE(flMotor)),
//                new Logger.Column("frEnc", Motors.encoderIE(frMotor)),
//                new Logger.Column("blEnc", Motors.encoderIE(blMotor)),
//                new Logger.Column("brEnc", Motors.encoderIE(brMotor)),
//                new Logger.Column("shooterEnc", Motors.encoderIE(shooterMotor)),
////        new Logger.Column("collectorEnc", Motors.encoderIE(collectorMotor)),
//                new Logger.Column("liftEnc", Motors.encoderIE(capBallLiftMotor))
                new Logger.Column("liftEncoder", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getGrabber().getLiftEncoderValue();
                    }
                }),
                new Logger.Column("setPoint", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getGrabber().getSetPoint();
                    }
                })
        );



}
    public Servos getServos() {
        return servos;
    }

    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }
    public SpikeDetector getRightLineSensor() {
        return rightLineSensor;
    }

    public SpikeDetector getLeftLineSensor() {return leftLineSensor;}



    @Override
    public void start() {

    }
    private final StepTimer stepTimer = new StepTimer("robotCfg", Log.VERBOSE);

    @Override
    public void act() {
        relic.act();
        stepTimer.start();
        stepTimer.step("line sensors");
        leftLineSensor.act();
        rightLineSensor.act();
        stepTimer.step("mecanumControl");
        mecanumControl.act();
        //stepTimer.step("gyro");
        stepTimer.step("grabber");
        grabber.act();

        stepTimer.stop();
//        gyro.update();
//        leftLineSensor.act();
//        rightLineSensor.act();
//
//        mecanumControl.act();
//        grabber.act();


    }
    @Override
    public void stop() {
        mecanumControl.stop();
        gyro.stop();
    }

    public EVStateMachineBuilder createEVStateMachineBuilder(StateName firstStateName, TeamColor teamColor, Angle tolerance) {
        return new EVStateMachineBuilder(firstStateName, teamColor, tolerance, gyro, frameGrabber, servos, mecanumControl);
    }

}
