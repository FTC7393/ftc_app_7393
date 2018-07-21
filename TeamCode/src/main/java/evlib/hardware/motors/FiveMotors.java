package evlib.hardware.motors;

import com.google.common.collect.ImmutableList;

import ftc.electronvolts.util.units.Velocity;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 *
 * A subclass of NMotors that provides convenience methods for passing in 5 motor powers.
 *
 * @see evlib.hardware.motors.NMotors
 */
public class FiveMotors extends evlib.hardware.motors.NMotors {
    public FiveMotors(evlib.hardware.motors.Motor motor1, evlib.hardware.motors.Motor motor2, evlib.hardware.motors.Motor motor3, evlib.hardware.motors.Motor motor4, Motor motor5, boolean useSpeedMode, Velocity maxRobotSpeed) {
        super(ImmutableList.of(motor1, motor2, motor3, motor4, motor5), useSpeedMode, maxRobotSpeed);
    }

    public void runMotorsNormalized(double value1, double value2, double value3, double value4, double value5) {
        runNormalized(ImmutableList.of(value1, value2, value3, value4, value5));
    }

    public void runMotors(double value1, double value2, double value3, double value4, double value5) {
        run(ImmutableList.of(value1, value2, value3, value4, value5));
    }
}
