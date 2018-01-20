package org.firstinspires.ftc.teamcode.relic2017;

import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;

/**
 * Created by ftc7393 on 12/30/2017.
 */

public class Relic {

    public Relic(MotorEnc extend, DigitalSensor inLimit, ServoControl grab) {
        this.extend = extend;

        this.grab = grab;
        this.inLimit = new DigitalInputEdgeDetector(inLimit);


    }





    private enum ServoAction {
        OPEN,
        GRAB
    }



    private ServoAction servoAction = ServoAction.OPEN;
    private ServoAction lastServoAction = null;


    private final MotorEnc extend;
    private final ServoControl grab;

    private final DigitalInputEdgeDetector inLimit;
    private static final double SERVO_SPEED=2;


    private static final int END_ENCODER = 10000;


    private boolean limitPressed = false;
    private double relicEncoder = 0;

    public double getRelicEncoderValue() {
        return relicEncoder;
    }

    public boolean getInLimit() {
        return limitPressed;
    }
    private  double extend_speed = 0;//need to find value



    public void act() {
        inLimit.update();
        relicEncoder=extend.getEncoderPosition();


        if((inLimit.isPressed()&&(extend_speed <=0))||(END_ENCODER<=getRelicEncoderValue()&&extend_speed >=0)) {
            extend.setPower(0);

        }
        else{
            extend.setPower(extend_speed);

        }
        if(servoAction!=lastServoAction) {
            lastServoAction=servoAction;

            if (servoAction == ServoAction.OPEN) {
                grab.goToPreset(RobotCfg2017.relicServoPresets.OPEN, SERVO_SPEED);

            } else if (servoAction == ServoAction.GRAB) {
                grab.goToPreset(RobotCfg2017.relicServoPresets.GRAB, SERVO_SPEED);
            }
        }
        extend.update();



    }


    public void setPower(double power){
        extend_speed =power;}

    public ServoAction getServo() {
        return servoAction;
    }



    public void openServo() {
        servoAction=ServoAction.OPEN;
    }

    public void grabServo() {
        servoAction = ServoAction.GRAB;
    }
    public void toggleServo() {
        if (servoAction == ServoAction.OPEN ) {
            grabServo();
        } else  {
            openServo();
        }



    }







}
