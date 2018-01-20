package org.firstinspires.ftc.teamcode.relic2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.ResultReceiver;
import ftc.evlib.hardware.sensors.Gyro;

/**
 * Created by ftc7393 on 12/9/2017.
 */

public class IMUGyro implements Gyro {
    // The IMU sensor object
    final BNO055IMU imu;

    // State used for updating telemetry
//    Orientation angles=new Orientation();
//    Acceleration gravity;

    final ResultReceiver<Double> angleReciever;
    final ResultReceiver<Boolean> stopReciever;



    public IMUGyro(final BNO055IMU imu) {
        this.imu = imu;
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        angleReciever=new BasicResultReceiver<>();
        stopReciever=new BasicResultReceiver<>();

        angleReciever.setValue(0.0);
        new Thread(new Runnable() {
            @Override
            public void run() {
                imu.initialize(parameters);
                isCalibrated.setValue(true);
                while(!stopReciever.isReady()){
                    Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    angleReciever.setValue((double) -angles.firstAngle);
                }

            }
        }).start();


    }

//    protected void notSupported(){
//        throw new UnsupportedOperationException("Method not supported for "+getDeviceName());
//
//    }






    @Override
    public double getHeading() {


        return angleReciever.getValue();
    }

//    @Override
//    public void update() {
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////        gravity  = imu.getGravity();
//    }

    @Override
    public boolean isCalibrating() {
        return !isCalibrated.isReady();

    }

    @Override
    public void stop() {
        stopReciever.setValue(true);
    }

    public ResultReceiver<Boolean> isCalibrated=new BasicResultReceiver<>();




}
