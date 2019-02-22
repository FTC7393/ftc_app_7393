package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.support.annotation.NonNull;

public class Mineral implements Comparable<Mineral>{
    private final int x, y, width, height;
    private final double radius;
    private final float confidence;
    private final boolean isGold;


    public Mineral(int x, int y, int width, int height,double radius,boolean isGold, float confidence) {

        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.radius=radius;
        this.isGold=isGold;
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

    public double getRadius(){return radius;}
    public boolean isGold(){return isGold;}


    public float getConfidence() {
        return confidence;
    }

    @Override
    public int compareTo(@NonNull Mineral another) {
        if(this.radius<another.radius){
            return -1;
        }
        else if(this.radius>another.radius){
            return 1;
        }
        return 0;
    }
}
