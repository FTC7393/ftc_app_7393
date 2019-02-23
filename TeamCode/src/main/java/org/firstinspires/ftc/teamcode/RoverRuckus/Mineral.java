package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.support.annotation.NonNull;

public class Mineral implements Comparable<Mineral>{
    private final int x, y, width, height;
    private final double radius;
    private final float confidence;
    private final boolean isGold;
    private GoldDetector.Detection type;
    public Mineral(){
        this.type=GoldDetector.Detection.NOTHING;
        this.confidence = 0;
        this.x = 0;
        this.y = 0;
        this.width = 0;
        this.height = 0;
        this.radius=0;
        this.isGold=false;
    }


    public Mineral(int x, int y, int width, int height,double radius,boolean isGold, float confidence) {
        this.type=GoldDetector.Detection.NOTHING;
        this.confidence = confidence;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.radius=radius;
        this.isGold=isGold;


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
    public void setType(GoldDetector.Detection type){this.type=type;}
    public GoldDetector.Detection getType(){return type;}
    public String toString(){
        return String.format("%s,%.3f,%d,%d,%d,%d,%.2f",this.type,this.confidence,this.x,this.y,this.width,this.height,this.radius);
    };

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
