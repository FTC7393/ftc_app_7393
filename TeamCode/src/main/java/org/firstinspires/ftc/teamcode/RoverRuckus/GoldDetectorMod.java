package org.firstinspires.ftc.teamcode.RoverRuckus;


import com.google.common.collect.ImmutableList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusTests.TensorFlowObjectDetection;

import java.util.List;


class Rect {

    private final int x1,x2,y1,y2;

    public Rect(int x1, int x2, int y1, int y2) {
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
    }
    public boolean isInside(int x, int y) {
        if (x >x1 && x < x2 && y > y1 && y < y2) {
            return true;
        } else {
            return false;
        }
    }
}

public class GoldDetectorMod {

//    private final int maxWidth = 200;
//    private final int maxHeight = 450;
    private final int maxWidth = 300;
    private final int maxHeight = 800;
//    private final Rect leftFilter = new Rect(50,380,590,20);
//    private final Rect rightFilter = new Rect(690, 380, 1230, 20);
    private final Rect leftFilter = new Rect(0,640,100,380);
    private final Rect rightFilter = new Rect(640,1280,100,380);
    private final List<Recognition> fullList;

    public GoldDetectorMod(List<Recognition> fullList) {
        this.fullList = fullList;
    }

    public GoldPosition findPosition(Telemetry telemetry) {
        List<Recognition> list = filter(fullList, telemetry);
        if (list.size() != 2) {
            //Filter failed to remove extra recognitions
            return GoldPosition.UNKNOWN;
        }
        Recognition r1 = list.get(0);
        Recognition r2 = list.get(1);
        if (bothSilver(r1, r2)) {
            return GoldPosition.LEFT;
        }
        if (bothGold(r1, r2)) {
            return GoldPosition.UNKNOWN;
        }
        //If we make it this far, one of the two is gold, and it is either left or right
        Recognition left;
        int r1x = 1280 - (int) r1.getBottom();
        int r2x = 1280 - (int) r2.getBottom();
        if (r1x < r2x) {
            left = r1;
        } else {
            left = r2;
        }
        if (left.getLabel().equals(TensorFlowObjectDetection.LABEL_GOLD_MINERAL)) {
            return GoldPosition.MIDDLE;
        } else {
            return GoldPosition.RIGHT;
        }
    }


    private boolean bothSilver(Recognition r1, Recognition r2) {
        return r1.getLabel().equals(TensorFlowObjectDetection.LABEL_SILVER_MINERAL) &&
                r2.getLabel().equals(TensorFlowObjectDetection.LABEL_SILVER_MINERAL);
    }

    private boolean bothGold(Recognition r1, Recognition r2) {
        return r1.getLabel().equals(TensorFlowObjectDetection.LABEL_GOLD_MINERAL) &&
                r2.getLabel().equals(TensorFlowObjectDetection.LABEL_GOLD_MINERAL);
    }

    private List<Recognition> filter(List<Recognition> fullList, Telemetry telemetry) {
        ImmutableList.Builder<Recognition> b = ImmutableList.builder();
        int c = 0;
        for (Recognition r: fullList) {
            int x = 1280 - (int) r.getBottom();
            int y = 760 - (int) r.getRight();
            telemetry.addData("X"+c, x);
            telemetry.addData("Y"+(c++), y);
            boolean insideFilterBoxes = leftFilter.isInside(x,y) || rightFilter.isInside(x,y);
            boolean heightOk = r.getWidth() < maxHeight;
            boolean widthOK = r.getHeight() < maxWidth;
            telemetry.addData("inside", insideFilterBoxes);
            telemetry.addData("hieghtOK?", heightOk);
            telemetry.addData("widthOK?", widthOK);

            if (insideFilterBoxes && heightOk && widthOK) {
                b.add(r);
            }
        }
        return b.build();
    }
}
