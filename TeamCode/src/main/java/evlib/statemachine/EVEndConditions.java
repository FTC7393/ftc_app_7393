package evlib.statemachine;


import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import evlib.hardware.sensors.AnalogSensor;
import evlib.hardware.sensors.DigitalSensor;
import evlib.hardware.sensors.ColorDigitalSensor;

import evlib.hardware.sensors.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import evlib.hardware.sensors.DoubleLineSensor;
import evlib.hardware.sensors.Gyro;
import evlib.hardware.sensors.LineFinder;
import evlib.hardware.sensors.LineSensorArray;



/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 5/10/16
 *
 * Factory class for EndCondition
 * extends EndConditions, which has some useful factory methods already
 *
 * @see EndCondition
 * @see EndConditions
 */
public class EVEndConditions extends EndConditions {

    /**
     * wait for a signal on a digital sensor
     *
     * @param digitalSensor the sensor
     * @return the created EndCondition
     */
    public static EndCondition digitalSensor(final DigitalSensor digitalSensor) {
        return new EndCondition() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return digitalSensor.getValue();
            }
        };
    }

    /**
     * wait for a line of any color
     *
     * @param doubleLineSensor   the two line sensors
     * @param detectionsRequired the number of detections in a row to look for (to avoid false positives)
     * @return the created EndCondition
     * @see DoubleLineSensor
     */
    public static EndCondition foundLine(final DoubleLineSensor doubleLineSensor, final int detectionsRequired) {
        return new EndCondition() {
            int detections;

            @Override
            public void init() {
                detections = 0;
            }

            @Override
            public boolean isDone() {
                DoubleLineSensor.LinePosition linePosition = doubleLineSensor.getPosition();
                if (linePosition == DoubleLineSensor.LinePosition.LEFT || linePosition == DoubleLineSensor.LinePosition.MIDDLE || linePosition == DoubleLineSensor.LinePosition.RIGHT) {
                    detections++;
                    if (detections > detectionsRequired) {
                        return true;
                    }
                } else {
                    detections = 0;
                }
                return false;
            }
        };
    }

    /**
     * wait for a line of a certain color
     *
     * @param lineFinder         two line sensors and a reflective color sensor
     * @param lineColor          the color of line to look for
     * @param detectionsRequired the number of detections in a row to look for (to avoid false positives)
     * @return the created EndCondition
     * @see LineFinder
     */
    public static EndCondition foundColoredLine(final LineFinder lineFinder, final LineFinder.LineColor lineColor, final int detectionsRequired) {
        return new EndCondition() {
            int detections;

            @Override
            public void init() {
                detections = 0;
                lineFinder.startLookingFor(lineColor);
            }

            @Override
            public boolean isDone() {
                if (lineFinder.getValue()) {
                    detections++;
                    if (detections > detectionsRequired) {
                        return true;
                    }
                } else {
                    detections = 0;
                }
                return false;
            }
        };
    }

    /**
     * look for a value from a sensor that is greater/less than a target value
     *
     * @param analogSensor the sensor
     * @param target       the target sensor value
     * @param greater      true if the sensor value needs to be greater than the target value
     * @return the created EndCondition
     * @see AnalogSensor
     */
    public static EndCondition analogSensor(final AnalogSensor analogSensor, final double target, final boolean greater) {
        return new EndCondition() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                if (greater) {
                    return (analogSensor.getValue() >= target);
                } else {
                    return (analogSensor.getValue() <= target);
                }
            }
        };
    }

    /**
     * look for a value from a sensor that is greater than a target value
     *
     * @param analogSensor the sensor
     * @param value        the target value
     * @return the created EndCondition
     * @see AnalogSensor
     */
    public static EndCondition analogSensorGreater(AnalogSensor analogSensor, double value) {
        return analogSensor(analogSensor, value, true);
    }

    /**
     * look for a value from a sensor that is less than a target value
     *
     * @param analogSensor the sensor
     * @param value        the target value
     * @return the created EndCondition
     * @see AnalogSensor
     */
    public static EndCondition analogSensorLess(AnalogSensor analogSensor, double value) {
        return analogSensor(analogSensor, value, false);
    }

    /**
     * wait until the gyro heading is close to a target value
     *
     * @param gyro             the gyro sensor
     * @param targetDegrees    the target value (in degrees)
     * @param toleranceDegrees the accepted tolerance to be considered "close to" (in degrees)
     * @return the created EndCondition
     * @see Gyro
     */
    public static EndCondition gyroCloseTo(Gyro gyro, double targetDegrees, double toleranceDegrees) {
        return gyroCloseTo(gyro, Angle.fromDegrees(targetDegrees), Angle.fromDegrees(toleranceDegrees));
    }

    /**
     * wait until the gyro heading is close to a target value
     *
     * @param gyro             the gyro sensor
     * @param target           the target value
     * @param toleranceDegrees the accepted tolerance to be considered "close to" (in degrees)
     * @return the created EndCondition
     * @see Gyro
     */
    public static EndCondition gyroCloseTo(Gyro gyro, Angle target, double toleranceDegrees) {
        return gyroCloseTo(gyro, target, Angle.fromDegrees(toleranceDegrees));
    }

    /**
     * wait until the gyro heading is close to a target value
     *
     * @param gyro          the gyro sensor
     * @param targetDegrees the target value (in degrees)
     * @param tolerance     the accepted tolerance to be considered "close to"
     * @return the created EndCondition
     * @see Gyro
     */
    public static EndCondition gyroCloseTo(Gyro gyro, double targetDegrees, Angle tolerance) {
        return gyroCloseTo(gyro, Angle.fromDegrees(targetDegrees), tolerance);
    }

    public static Double separationLog;
    public static Double toleranceLog;

    /**
     * wait until the gyro heading is close to a target value
     *
     * @param gyro      the gyro sensor
     * @param target    the target value
     * @param tolerance the accepted tolerance to be considered "close to"
     * @return the created EndCondition
     * @see Gyro
     */
    public static EndCondition gyroCloseTo(final Gyro gyro, Angle target, final Angle tolerance) {
        final Vector2D targetVector = new Vector2D(1, target);
        return new EndCondition() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                Vector2D gyroVector = new Vector2D(1, Angle.fromDegrees(gyro.getHeading()));
                Angle separation = Vector2D.signedAngularSeparation(gyroVector, targetVector);
                separationLog=separation.radians();
                toleranceLog=tolerance.radians();
                return Math.abs(separation.radians()) <= tolerance.radians();
            }
        };
    }

    /**
     * wait until the gyro heading is close to a target value relative to the starting heading
     *
     * @param gyro      the gyro sensor
     * @param target    the target value relative to the starting heading
     * @param tolerance the accepted tolerance to be considered "close to"
     * @return the created State
     * @see Gyro
     */
    public static EndCondition gyroCloseToRelative(final Gyro gyro, Angle target, final Angle tolerance) {
        final Vector2D targetVector = new Vector2D(1, target);
        return new EndCondition() {
            double gyroInit = 0;

            @Override
            public void init() {
                gyroInit = gyro.getHeading();
            }

            @Override
            public boolean isDone() {
                Vector2D gyroVector = new Vector2D(1, Angle.fromDegrees(gyro.getHeading() - gyroInit));
                Angle separation = Vector2D.signedAngularSeparation(targetVector, gyroVector);
                return Math.abs(separation.degrees()) <= tolerance.degrees();
            }
        };
    }

    /**
     * wait for a reflection of red light with a color sensor
     *
     * @param colorSensor the sensor
     * @return the created EndCondition
     * @see ColorSensor
     */
    public static EndCondition colorSensorRedLight(final ColorDigitalSensor colorSensor) {
        return new EndCondition() {
            @Override
            public void init() {
                colorSensor.setColorRed();
            }

            @Override
            public boolean isDone() {
                return colorSensor.getValue();
            }
        };
    }

    /**
     * wait for a reflection of blue light with a color sensor
     *
     * @param colorSensor the sensor
     * @return the created EndCondition
     * @see ColorSensor
     */
    public static EndCondition colorSensorBlueLight(final ColorDigitalSensor colorSensor) {
        return new EndCondition() {
            @Override
            public void init() {
                colorSensor.setColorBlue();
            }

            @Override
            public boolean isDone() {
                return colorSensor.getValue();
            }
        };
    }

    /**
     * Wait for any sensor to activate on a line sensor array
     *
     * @param lineSensorArray the sensor
     * @return the created State
     * @see LineSensorArray
     */
    public static EndCondition lineSensorArrayAny(final LineSensorArray lineSensorArray) {
        return new EndCondition() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return lineSensorArray.getNumSensorsActive() > 0;
            }
        };
    }

    /**
     * Wait for a distance sensor to cross a threshold
     *
     * @param distanceSensor the distance sensor
     * @param target         the target value
     * @param greater        whether to wait for the sensor to be greater or less than the target
     * @return the created State
     * @see DistanceSensor
     */
    public static EndCondition distanceSensor(final DistanceSensor distanceSensor, final Distance target, final boolean greater) {
        return new EndCondition() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                if (greater) {
                    return (distanceSensor.getDistance().meters() >= target.meters());
                } else {
                    return (distanceSensor.getDistance().meters() <= target.meters());
                }
            }
        };
    }

    /**
     * Wait for a distance sensor to be less than a certain distance
     *
     * @param distanceSensor the distance sensor
     * @param target         the target Distance
     * @return the created State
     * @see DistanceSensor
     */
    public static EndCondition distanceSensorLess(DistanceSensor distanceSensor, Distance target) {
        return distanceSensor(distanceSensor, target, false);
    }

    /**
     * Wait for a distance sensor to be greater than a certain distance
     *
     * @param distanceSensor the distance sensor
     * @param target         the target Distance
     * @return the created State
     * @see DistanceSensor
     */
    public static EndCondition distanceSensorGreater(DistanceSensor distanceSensor, Distance target) {
        return distanceSensor(distanceSensor, target, true);
    }

    public static EndCondition coloredLine(final ColorSensor  colorSensor) {
        return new EndCondition() {
            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.

            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;
            boolean isDone=false;

            @Override
            public void init() {
                final float values[] = hsvValues;

//                Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
//                        (int) (colorSensor.green() * SCALE_FACTOR),
//                        (int) (colorSensor.blue() * SCALE_FACTOR),
//                        hsvValues);



            }



            @Override
            public boolean isDone() {

                if( hsvValues[0]>100){
                    isDone=true;
                }

                return isDone;
            }
        };
    }
}
