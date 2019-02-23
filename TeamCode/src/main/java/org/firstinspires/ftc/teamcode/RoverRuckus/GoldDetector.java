package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.files.Logger;

public class GoldDetector {
//    String
//    Logger createLogger(){
//        String prefix = "sandbox_log";
//        String postfix = ".txt";
//        ImmutableList.Builder<Logger.Column> cols = ImmutableList.builder();
//        cols = ImmutableList.of(
//                new Logger.Column("gyro", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return gyro.getHeading();
//                    }
//                }),
//                new Logger.Column("velocityR", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return mecanumControl.getVelocityR();
//                    }
//                }),
//                new Logger.Column("Motor 0", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return mecanumControl.getMecanumMotors().getValue(0);
//                    }
//                }),
//                new Logger.Column("Tolerance", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return EVEndConditions.toleranceLog ;
//                    }
//                }  ),
//                new Logger.Column("Separation", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return EVEndConditions.separationLog ;
//                    }
//                }
//
//                ));
//
//
//
//        return new Logger("vision_log",".csv",);
//    }

    private final int maxWidth = 250;
    private final int minWidth = 130;
    private final int maxHeight = 250;
    private final int minHeight = 100;
    private final int x1=320;
    private final int x2=960;
    private final int y1=20;
    private final int y2=360;

    private final Rectangle centerFilter = new Rectangle(x1, x2, y1, y2);

    private final List<Recognition> fullList;
    public enum Detection{
        GOLD,
        SILVER,
        NOTHING
    }



    public GoldDetector(List<Recognition> fullList) {
        this.fullList = fullList;
    }


    public Detection findPosition(Telemetry telemetry) {
        List<Mineral> list = filter(fullList, telemetry);

        if(list.size()>0){
            Mineral m1=list.get(0);
            if(m1.isGold()){
                return Detection.GOLD;


            }
            else if(!m1.isGold()){
                return Detection.SILVER;

            }


        }

        return Detection.NOTHING;


    }




    private List<Mineral> filter(List<Recognition> fullList, Telemetry telemetry) {
        List<Mineral> b = new ArrayList<>();
        int c=0;
        for (Recognition r : fullList) {
            int x = 1280 - (int) r.getBottom();
            int y = 720 - (int) r.getRight();
            boolean insideFilterBox = centerFilter.isInside(x, y) ;
            int h = (int) r.getWidth(); // camera is sideways
            int w = (int) r.getHeight();
            //In our coordinate system, width  is height and height is width
            double centerOfFilterX=(x1-x2)/2.;
            double centerOfBlockX=(x+.5*(w));
            double centerOfFilterY=(y1-y2)/2.;
            double centerOfBlockY=(y+.5*(h));

            double sideX=centerOfFilterX-centerOfBlockX;
            double sideY=centerOfFilterY-centerOfBlockY;
            double radius=Math.sqrt(sideX*sideX+sideY*sideY);
            boolean isGold = r.getLabel().equals(ObjectDetector.LABEL_GOLD_MINERAL);

            boolean heightOk = (h < maxHeight) && (h > minHeight);
            boolean widthOK = (w < maxWidth) && (w > minWidth);

            if (telemetry != null) {
                telemetry.addData("X" + c, x);
                telemetry.addData("Y" + (c++), y);
                telemetry.addData("inside", insideFilterBox);
                telemetry.addData("hieghtOK?", heightOk + " " + r.getWidth());
                telemetry.addData("widthOK?", widthOK + " " + r.getHeight());
            }

            if (insideFilterBox && heightOk && widthOK) {
                b.add(new Mineral(x, y, w, h,radius,isGold, r.getConfidence()));
            }
        }
        Collections.sort(b);

        return ImmutableList.copyOf(b);
    }
}

