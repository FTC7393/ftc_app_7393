package org.firstinspires.ftc.teamcode.RoverRuckus;

public class Mineral {
    private final int x, y, width, height;
    private final boolean isGold;
    private final float confidence;


    public Mineral(int x, int y, int width, int height, boolean isGold, float confidence) {

        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.isGold = isGold;
        this.confidence = confidence;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public boolean isGold() {
        return isGold;
    }

    public float getConfidence() {
        return confidence;
    }
}
