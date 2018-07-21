package evlib.hardware.motors;

import com.google.common.collect.ImmutableList;

import ftc.electronvolts.util.units.Velocity;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 12/27/15
 *
 * A subclass of NMotors that provides convenience methods for passing in 4 motor powers.
 *
 * @see evlib.hardware.motors.NMotors
 */
public class FourMotors extends evlib.hardware.motors.NMotors {
    public FourMotors(evlib.hardware.motors.Motor frontLeftMotor, evlib.hardware.motors.Motor frontRightMotor, evlib.hardware.motors.Motor backLeftMotor, Motor backRightMotor, boolean useSpeedMode, Velocity maxRobotSpeed) {
        super(ImmutableList.of(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor), useSpeedMode, maxRobotSpeed);
    }

    public void runMotorsNormalized(double flValue, double frValue, double blValue, double brValue) {
        runNormalized(ImmutableList.of(flValue, frValue, blValue, brValue));
    }

    public void runMotors(double flValue, double frValue, double blValue, double brValue) {
        run(ImmutableList.of(flValue, frValue, blValue, brValue));
    }
}
