package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.graphics.Rect;

import com.google.common.collect.ImmutableList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusTests.TensorFlowObjectDetection;

import java.util.List;

public class GoldDetector {

    private final int maxWidth = 300;
    private final int maxHeight = 300;
    private final Rectangle leftFilter = new Rectangle(20, 640, 20, 360);
    private final Rectangle rightFilter = new Rectangle(660, 1260, 40, 360);
    private final List<Recognition> fullList;

    public GoldDetector(List<Recognition> fullList) {
        this.fullList = fullList;
    }

    public GoldPosition findPosition(Telemetry telemetry) {
        List<Mineral> list = filter(fullList, telemetry);
        if (list.size() != 2) {
            //Filter failed to remove extra recognitions
            return GoldPosition.UNKNOWN;
        }
        Mineral m1 = list.get(0);
        Mineral m2 = list.get(1);
        if (bothSilver(m1, m2)) {
            return GoldPosition.LEFT;
        }
        if (bothGold(m1, m2)) {
            return GoldPosition.UNKNOWN;
        }
        //If we make it this far, one of the two is gold, and it is either left or right
        Mineral left;
        if (m1.getX() < m2.getX()) {
            left = m1;
        } else {
            left = m2;
        }
        if (left.isGold()) {
            return GoldPosition.MIDDLE;
        } else {
            return GoldPosition.RIGHT;
        }
    }


    private boolean bothSilver(Mineral m1, Mineral m2) {
        return (!m1.isGold()) && (!m2.isGold());
    }

    private boolean bothGold(Mineral m1, Mineral m2) {
        return (m1.isGold()) && (m2.isGold());

    }

    private List<Mineral> filter(List<Recognition> fullList, Telemetry telemetry) {
        ImmutableList.Builder<Mineral> b = ImmutableList.builder();
        int c=0;
        for (Recognition r : fullList) {
            int x = 1280 - (int) r.getBottom();
            int y = 720 - (int) r.getRight();

            boolean insideFilterBoxes = leftFilter.isInside(x, y) || rightFilter.isInside(x, y);
            //In our coordinate system, width  is height and height is width
            boolean heightOk = r.getWidth() < maxHeight;
            boolean widthOK = r.getHeight() < maxWidth;
            telemetry.addData("X"+c, x);
            telemetry.addData("Y"+(c++), y);
            telemetry.addData("inside", insideFilterBoxes);
            telemetry.addData("hieghtOK?", heightOk + " " + r.getWidth());
            telemetry.addData("widthOK?", widthOK + " " + r.getHeight());




            if (insideFilterBoxes && heightOk && widthOK) {
                boolean isGold = r.getLabel().equals(TensorFlowObjectDetection.LABEL_GOLD_MINERAL);
                b.add(new Mineral(x, y, (int) r.getHeight(), (int) r.getWidth(),
                        isGold, r.getConfidence()));
            }
        }
        return b.build();
    }
}
