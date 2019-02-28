package org.firstinspires.ftc.teamcode.RoverRuckus;

import java.util.HashMap;

public class MineralDecisionMaker {
    private HashMap<String, String> map;


    public MineralDecisionMaker() {
        map = new HashMap<>();
        map.put("GGG","M");
        map.put("GGS","M");
        map.put("GGU","M");
        map.put("GSG","R");
        map.put("GSS","L");
        map.put("GSU","L");
        map.put("GUG","R");
        map.put("GUS","L");
        map.put("GUU","L");
        map.put("SSS","M");
        map.put("SSG","R");
        map.put("SSU","R");
        map.put("SGS","M");
        map.put("SGG","M");
        map.put("SGU","M");
        map.put("SUS","M");
        map.put("SUG","R");
        map.put("SUU","M");
        map.put("UUU","M");
        map.put("UGG","M");
        map.put("UGS","M");
        map.put("UGU","M");
        map.put("USG","R");
        map.put("USS","L");
        map.put("USU","L");
        map.put("UUG","R");
        map.put("UUS","M");
    }
    public GoldPosition decidePosition(GoldDetector.Detection left, GoldDetector.Detection middle, GoldDetector.Detection right) {
        String key = getLetter(left)+getLetter(middle)+getLetter(right);
        String position = map.get(key);
        if (position == "L") {
            return GoldPosition.LEFT;
        }
        else if (position == "M") {
            return GoldPosition.MIDDLE;
        }
        else {
            return GoldPosition.RIGHT;
        }
    }

    private String getLetter(GoldDetector.Detection detection) {
        String key;
        if (detection == GoldDetector.Detection.GOLD) {
            key = "G";
        } else if (detection == GoldDetector.Detection.SILVER) {
            key = "S";
        } else if (detection == GoldDetector.Detection.NOTHING) {
            key = "U";
        } else {
            key = "U";
        }
        return key;
    }
}
