package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.util.Log;

import com.google.common.collect.ImmutableList;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.relic2017.Mechanisms.IMUGyro;

import java.util.List;
import java.util.Map;

import evlib.hardware.config.RobotCfg;
import evlib.hardware.control.MecanumControl;
import evlib.hardware.motors.MecanumMotors;
import evlib.hardware.motors.Motor;
import evlib.hardware.motors.MotorEnc;
import evlib.hardware.motors.Motors;
import evlib.hardware.sensors.AnalogSensor;
import evlib.hardware.sensors.Gyro;
import evlib.hardware.sensors.Sensors;
import evlib.hardware.servos.ServoCfg;
import evlib.hardware.servos.ServoControl;
import evlib.hardware.servos.ServoName;
import evlib.hardware.servos.Servos;
import evlib.statemachine.EVStateMachineBuilder;
import evlib.util.StepTimer;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.PIDController;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

import static evlib.vision.framegrabber.GlobalFrameGrabber.frameGrabber;

/**
 * Created by ftc7393 on 9/29/2018.
 */

public class RoverRuckusRobotCfg extends RobotCfg {
    //private final ColorSensor leftColorSensor;
    //private final ColorSensor rightColorSensor;
    private final MecanumControl mecanumControl;
    private final Collector collector;
    private final Hanging hanging;
    private final ServoControl marker;
    final MotorEnc frontLeft;
    final MotorEnc frontRight;
    final MotorEnc backLeft;
    final MotorEnc backRight;
    ServoControl Door;


    //    private final Collector collector;
    private final Arm arm;
    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));
    private static final String LEFT_COLOR_SENSOR_NAME = "colorBackLeft";
    private static final String RIGHT_COLOR_SENSOR_NAME = "colorBackRight";
    private IMUGyro gyro;
    private final Servos servos;
    ServoControl backFoot;
    public RoverRuckusRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));
        Door =   getServo(MainServoName.DOOR);
        marker = getServo(MainServoName.MARKER);



        //leftColorSensor = hardwareMap.colorSensor.get(LEFT_COLOR_SENSOR_NAME);
        //rightColorSensor = hardwareMap.colorSensor.get(RIGHT_COLOR_SENSOR_NAME);
        frontLeft=    Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), true, true, stoppers);
        frontRight=   Motors.withEncoder(hardwareMap.dcMotor.get("frontRight") , false, true, stoppers);
        backLeft=     Motors.withEncoder(hardwareMap.dcMotor.get("backLeft") , true, true, stoppers);
        backRight=    Motors.withEncoder(hardwareMap.dcMotor.get("backRight") , false, true, stoppers);




                mecanumControl = new MecanumControl(new MecanumMotors(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                        true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));
        gyro = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu"));
        arm=new Arm(
                Motors.withEncoder(hardwareMap.dcMotor.get("extension"),false,true,stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"extensionLimit")),
                Motors.withEncoder(hardwareMap.dcMotor.get("rotation"),false,true,stoppers),
                Sensors.analog(hardwareMap,"potentiometer"));
        collector=new Collector(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("collection"),false,true,stoppers),

                getServo(MainServoName.DOOR)        );
         backFoot=getServo(MainServoName.BACKFOOT);

         hanging=new Hanging(
                 Motors.withoutEncoder(hardwareMap.dcMotor.get("hanging"),false,true,stoppers),
                 getServo(MainServoName.LATCH),
                 Sensors.inv(Sensors.digital(hardwareMap,"latchLimit")),
                 Sensors.inv(Sensors.digital(hardwareMap,"unlatchLimit"))
         );




        loggerColumns = ImmutableList.of(
                new Logger.Column("Rotation Encoder",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getRotationEncoder();
                    }
                }),
                new Logger.Column("Rotation SetPoint",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getRotationSetPoint();
                    }
                }),
                new Logger.Column("Rotation Min Encoder Value",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getMinRotationValue();
                    }
                }),

                new Logger.Column("Rotation Power Output",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getRotationPower();
                    }
                }),
                new Logger.Column("Extension Encoder",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getExtensionEncoder();
                    }
                }),
                new Logger.Column("Extension SetPoint",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getExtensionSetPoint();
                    }
                }),
                new Logger.Column("Extension Min Encoder Value",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getMinExtensionValue();
                    }
                }),

                new Logger.Column("Extension Power Output",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getExtensionPower();
                    }
                }),

                new Logger.Column("Potentiometer",new InputExtractor<Double>() {
                @Override
                public Double getValue() {
                    return getArm().getPotentiometerValue();
                    }
                })
//                new Logger.Column("Extension Limit Switch",new InputExtractor<Boolean>() {
//                    @Override
//                    public Boolean getValue() {
//                        return getArm().getExtensionLimitSwitch();
//                    }
//                })


        );



//hi




    }
    public enum doorPresets{
        CLOSE,
        RIGHT,
        LEFT
    }
    public enum backFootPresets{
        LOCKED,
        OPENED,
        DEPLOYED
    }

    public enum latchPresets{
        LATCH,
        UNLATCH
    }
    public enum markerPresets{
        HOLD,
        RELEASE
    }
    public enum MainServoName implements ServoName {
        LATCH("latch",latchPresets.values()),
        DOOR("door",doorPresets.values()),
        BACKFOOT("backFoot",backFootPresets.values()),
        MARKER("marker",markerPresets.values());

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
    //public ColorSensor getLeftColorSensor() {
    //    return leftColorSensor;
    //}
    //public ColorSensor getRightColorSensor() {
    //    return rightColorSensor;
    //}
    public IMUGyro getGyro() {
        return gyro;
    }


    @Override
    public void start() {

    }

    StepTimer myTimer = new StepTimer("robotCfg",Log.INFO);


    @Override
    public void act() {
        myTimer.start();
        myTimer.step("mecanumControl.act");
        mecanumControl.act();
        myTimer.step("arm.act");
        arm.act();
        myTimer.step("collector.act");
        collector.act();
        myTimer.step("hanging.act");
        hanging.act();
        myTimer.stop();


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

    public Hanging getHanging() {
        return hanging;
    }

    public Collector getCollector(){
        return collector;
    }
    public ServoControl getBackFoot(){return backFoot;}
    public ServoControl getMarker(){return marker;}

    @Override
    public Servos getServos() {
        return servos;
    }
    private final List<Logger.Column> loggerColumns;

    public List<Logger.Column> getLoggerColumns() {
        return loggerColumns;
    }
    public EVStateMachineBuilder createEVStateMachineBuilder(StateName firstStateName, TeamColor teamColor, Angle tolerance) {
        return new EVStateMachineBuilder(firstStateName, teamColor, tolerance, gyro, frameGrabber, servos, mecanumControl);
    }

}
