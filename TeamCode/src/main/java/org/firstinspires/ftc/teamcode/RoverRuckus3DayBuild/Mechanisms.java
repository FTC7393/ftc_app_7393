package org.firstinspires.ftc.teamcode.RoverRuckus3DayBuild;

import com.qualcomm.robotcore.hardware.Servo;

import evlib.hardware.motors.MotorEnc;
import evlib.hardware.servos.ServoControl;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by ftc7393 on 9/8/2018.
 */

public class Mechanisms {
    private final MotorEnc slide;
    private final MotorEnc left;
    private final MotorEnc right;
    private ElapsedTime servotimeout = new ElapsedTime();
    private final ServoControl marker;
    private final ServoControl latch;

    int slideEncoder;
    int upEncoderPos=520;
    int downEncoderPos=0;
    int preUnlockEncoderPos=-40;
    int hookEncoderPos;
    int latchEncoderPos;

    private Mode mode = Mode.HANG;//at the end of auto, it is at DH


    enum Mode{
        HANG,
        PRE_UNLOCK,
        UNLOCK,
        UNLOCKED

    }
    public Mechanisms (MotorEnc slide,MotorEnc left, MotorEnc right, ServoControl marker,ServoControl latch) {
        this.slide = slide;
        this.marker=marker;
        this.left=left;
        this.right=right;
        this.latch=latch;
    }



    public void act() {
        slideEncoder = slide.getEncoderPosition();

        if(this.mode== mode.HANG) {
        //Wait for command
        }
        else if(this.mode== mode.PRE_UNLOCK){
            slide.setSpeed(-.5);
            if(slideEncoder<=preUnlockEncoderPos){
                slide.setSpeed(0);
                this.mode=mode.UNLOCK;
                servotimeout.reset();
            }
        }
        else if(this.mode== mode.UNLOCK){
            openLatch();
            if (servotimeout.seconds()>=.75){
                this.mode = mode.UNLOCKED;
            }
        }
        slide.update();
        left.update();
        right.update();
    }

    public void Up(){
        slide.setPosition(slideEncoder+30,1);
    }

    public void Down(){
        slide.setPosition(slideEncoder-30,1);
    }

    public boolean unLatch() {
        boolean isUnLatched=false;
        if (this.mode==mode.HANG){
            this.mode= mode.PRE_UNLOCK;

        }
        else if (this.mode== mode.UNLOCKED){
            slide.setSpeed(.7);
            if(slideEncoder>=upEncoderPos){
                slide.setSpeed(0);
                isUnLatched=true;
            }




        }
        return  isUnLatched;


    }
    public boolean autoDown(){

        slide.setSpeed(-.7);
        boolean isDown=false;

        if(slideEncoder<=downEncoderPos){
            slide.setSpeed(0);
            isDown=true;

        }
        return isDown;
    }
    public void drive(double leftPower,double rightPower){
        left.setSpeed(leftPower);
        right.setSpeed(rightPower);
    }





//    public void Up(){
//        mode=Mode.UP;
//     slide.setSpeed(.7);
//     if(slideEncoder>=upEncoderPos){
//         slide.setSpeed(0);
//
//     }
//    }
//    public void Down(){
//        mode=Mode.DOWN;
//
//        slide.setSpeed(-.7);
//        if(slideEncoder<=downEncoderPos){
//            slide.setSpeed(0);
//
//        }
//    }
//    public void Hook(){
//        if(mode==Mode.DOWN){
//            slide.setSpeed(.7);
//            if(slideEncoder>=hookEncoderPos){
//                slide.setSpeed(0);
//
//            }
//
//        }
//        else if(mode==Mode.UP){
//            slide.setSpeed(-.7);
//            if(slideEncoder<=hookEncoderPos){
//                slide.setSpeed(0);
//
//            }
//
//        }
//
//
//    }
//    public void Latch(){
//
//    }

    public void releaseMarker(){marker.goToPreset(RobotCfg3DayBuild.MarkerPresets.RELEASE);}
    public void clampMarker(){marker.goToPreset(RobotCfg3DayBuild.MarkerPresets.CLAMP);}
    public void openLatch(){latch.goToPreset(RobotCfg3DayBuild.LatchPresets.OPEN);}
    public void closeLatch(){
        this.mode=mode.HANG;
        latch.goToPreset(RobotCfg3DayBuild.LatchPresets.CLOSE);}


//

    public double getSlideEncoderValue() {
        return slideEncoder;
    }
}
