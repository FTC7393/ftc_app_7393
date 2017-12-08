//package org.firstinspires.ftc.teamcode.relic2017;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import ftc.electronvolts.util.Function;
//import ftc.electronvolts.util.Functions;
//import ftc.evlib.opmodes.AbstractOptionsOp;
//
///**
// * Created by ftc7393 on 12/6/2017.
// */
//@TeleOp(name = "OptionsOp")
//public class OptionsOp extends AbstractOptionsOp {
//    /**
//     * The filename will be set by the subclasses
//     *
//     * @param filename the name of the file where the options are stored
//     */
//    public OptionsOp(String filename) {
//        super(filename);
//    }
//
//
//
//    public static final String FILENAME = "AutoOptions.txt";
//
//    public static final String teamColorTag = "teamColor";
////    public static final TeamColor teamColorDefault = TeamColor.UNKNOWN;
//    public static final String preMatchTimeSecondsTag = "preMatchTimeSeconds";
//    public static final double preMatchTimeDefault = 0;
//    public static final String isStartingCornerTag = "isStartingCorner";
//    public static final boolean isStartingCornerDefault = true;
//    public static final String doCollectTag = "doCollect";
//    public static final boolean doCollectDefault = false;
//    public static final String collectDistanceInchesTag = "collectDistanceInches";
//    public static final double collectDistanceDefault = 12;
//    public static final String shotsTag = "shots";
//    public static final int shotsDefault = 2;
//    //    public static final String doBeaconsTag = "doBeacons";
////    public static final boolean doBeaconsDefault = true;
//    public static final String doCapBallTag = "doCapBall";
//    public static final boolean doCapBallDefault = true;
//    public static final String doRecollectionTag = "doRecollection";
//    public static final boolean doRecollectionDefault = true;
//    public static final String useShooterSensorTag = "useShooterSensor";
//    public static final boolean useShooterSensorDefault = true;
//    public static final String ejectBadParticlesTag = "ejectBadParticles";
//    public static final boolean ejectBadParticlesDefault = true;
//
//    private DigitalInputEdgeDetector driver1_left_stick_y_up, driver1_left_stick_y_down;
//
//    public OptionsOp() {
//        super(FILENAME);
//    }
//
//    @Override
//    protected void go() {
//        super.go();
//        driver1_left_stick_y_up = new DigitalInputEdgeDetector(new InputExtractor<Boolean>() {
//            @Override
//            public Boolean getValue() {
//                return driver1.left_stick_y.getValue() <= -0.5;
//            }
//        });
//        driver1_left_stick_y_down = new DigitalInputEdgeDetector(new InputExtractor<Boolean>() {
//            @Override
//            public Boolean getValue() {
//                return driver1.left_stick_y.getValue() >= 0.5;
//            }
//        });
//    }
//
//    @Override
//    protected void act() {
//        driver1_left_stick_y_up.update();
//        driver1_left_stick_y_down.update();
//        //use gamepad to change options
//
//        if (driver1.b.justPressed()) {
//            optionsFile.set(teamColorTag, TeamColor.RED);
//        }
//        if (driver1.x.justPressed()) {
//            optionsFile.set(teamColorTag, TeamColor.BLUE);
//        }
//        telemetry.addData("*1 red/blue button => " + teamColorTag, optionsFile.get(teamColorTag, TeamColor.UNKNOWN));
//
//
//        double preMatchTimeSeconds = optionsFile.get(preMatchTimeSecondsTag, preMatchTimeDefault);
//        if (driver1.dpad_up.justPressed()) {
//            preMatchTimeSeconds += .5;
//        }
//
//        if (driver1.dpad_down.justPressed()) {
//            preMatchTimeSeconds -= .5;
//        }
//        preMatchTimeSeconds = Utility.limit(preMatchTimeSeconds, 0, 30);
//        optionsFile.set(preMatchTimeSecondsTag, preMatchTimeSeconds);
//        telemetry.addData("*1 dpad up/down => " + preMatchTimeSecondsTag, String.format(Locale.ENGLISH, "%3.1f", preMatchTimeSeconds) + " seconds");
//
//
//        double collectDistanceInches = optionsFile.get(collectDistanceInchesTag, collectDistanceDefault);
//        if (driver1.dpad_left.justPressed()) {
//            collectDistanceInches -= 0.5;
//        }
//
//        if (driver1.dpad_right.justPressed()) {
//            collectDistanceInches += 0.5;
//        }
//        collectDistanceInches = Utility.limit(collectDistanceInches, 0, 12 * 6); //limit to 6 feet
//        optionsFile.set(collectDistanceInchesTag, collectDistanceInches);
//        telemetry.addData("*1 dpad left/right => " + collectDistanceInchesTag, String.format(Locale.ENGLISH, "%3.1f", collectDistanceInches) + " inches");
//
//
//        if (driver1.left_bumper.justPressed()) {
//            optionsFile.set(isStartingCornerTag, false);
//        }
//        if (driver1.right_bumper.justPressed()) {
//            optionsFile.set(isStartingCornerTag, true);
//        }
//        telemetry.addData("*1 left/right bumper => " + isStartingCornerTag, optionsFile.get(isStartingCornerTag, Boolean.class, null));
//
//
////        if (driver1.left_stick_button.justPressed()) {
////            optionsFile.set(doBeaconsTag, false);
////        }
////        if (driver1.right_stick_button.justPressed()) {
////            optionsFile.set(doBeaconsTag, true);
////        }
////        telemetry.addData("*1 left/right stick button => " + doBeaconsTag, optionsFile.get(doBeaconsTag, Boolean.class, null));
//
//
//        if (driver1.a.justPressed()) {
//            optionsFile.set(doCapBallTag, false);
//        }
//        if (driver1.y.justPressed()) {
//            optionsFile.set(doCapBallTag, true);
//        }
//        telemetry.addData("*1 a/y button => " + doCapBallTag, optionsFile.get(doCapBallTag, Boolean.class, null));
//
//
//        if (driver1.right_stick_y.getValue() > 0.5) {
//            optionsFile.set(doRecollectionTag, false);
//        }
//        if (driver1.right_stick_y.getValue() < -0.5) {
//            optionsFile.set(doRecollectionTag, true);
//        }
//        telemetry.addData("*1 right stick y => " + doRecollectionTag, optionsFile.get(doRecollectionTag, Boolean.class, null));
//
//
//        if (driver1.left_trigger.getValue() >= 0.7) {
//            optionsFile.set(doCollectTag, false);
//        }
//        if (driver1.right_trigger.getValue() >= 0.7) {
//            optionsFile.set(doCollectTag, true);
//        }
//        telemetry.addData("*1 left/right trigger => " + doCollectTag, optionsFile.get(doCollectTag, Boolean.class, null));
//
//
//        int shots = optionsFile.get(shotsTag, shotsDefault);
//        if (driver1_left_stick_y_up.justPressed()) {
//            shots++;
//        }
//        if (driver1_left_stick_y_down.justPressed()) {
//            shots--;
//        }
//
//        shots = (int) Utility.limit(shots, 0, 3);
//        optionsFile.set(shotsTag, shots);
//        telemetry.addData("*1 left_stick_y => " + shotsTag, shots);
//
////        double gain = optionsFile.get("gyro_gain", 0.1);
////        double max = optionsFile.get("gyro_max", 0.1);
////        gain += 1e-3 * pow10floor(gain) * matchTimer.getDeltaTime() * driver1.left_stick_y.getValue();
////        max += 1e-3 * pow10floor(max) * matchTimer.getDeltaTime() * driver1.right_stick_y.getValue();
////
////
////        telemetry.addData("* left joystick y => gyro_gain", gain);
////        telemetry.addData("* right joystick y => gyro_max", max);
////
////        optionsFile.set("gyro_gain", gain);
////        optionsFile.set("gyro_max", max);
//
//
////        double WALL_FOLLOW_TRANSLATE_VELOCITY = optionsFile.get("WALL_FOLLOW_TRANSLATE_VELOCITY", .1);
////        double WALL_FOLLOW_MAX_ANGULAR_SPEED = optionsFile.get("WALL_FOLLOW_MAX_ANGULAR_SPEED", .5);
//
////        WALL_FOLLOW_TRANSLATE_VELOCITY += 1e-3 * driver2.left_stick_y.getValue();
////        WALL_FOLLOW_MAX_ANGULAR_SPEED += 1e-3 * driver2.right_stick_y.getValue();
////        WALL_FOLLOW_TRANSLATE_VELOCITY += 1e-3 * pow10floor(WALL_FOLLOW_TRANSLATE_VELOCITY) * driver2.left_stick_y.getValue();
////        WALL_FOLLOW_MAX_ANGULAR_SPEED += 1e-3 * pow10floor(WALL_FOLLOW_MAX_ANGULAR_SPEED) * driver2.right_stick_y.getValue();
//
////        telemetry.addData("WALL_FOLLOW_TRANSLATE_VELOCITY", WALL_FOLLOW_TRANSLATE_VELOCITY);
////        telemetry.addData("WALL_FOLLOW_MAX_ANGULAR_SPEED", WALL_FOLLOW_MAX_ANGULAR_SPEED);
//
////        optionsFile.set("WALL_FOLLOW_TRANSLATE_VELOCITY", WALL_FOLLOW_TRANSLATE_VELOCITY);
////        optionsFile.set("WALL_FOLLOW_MAX_ANGULAR_SPEED", WALL_FOLLOW_MAX_ANGULAR_SPEED);
//
//        if (driver2.a.justPressed()) {
//            optionsFile.set(ejectBadParticlesTag, false);
//        }
//        if (driver2.y.justPressed()) {
//            optionsFile.set(ejectBadParticlesTag, true);
//        }
//        telemetry.addData("*2 a/y button => " + ejectBadParticlesTag, optionsFile.get(ejectBadParticlesTag, Boolean.class, null));
//
//        if (driver2.left_trigger.getValue() >= 0.7) {
//            optionsFile.set(useShooterSensorTag, false);
//        }
//        if (driver2.right_trigger.getValue() >= 0.7) {
//            optionsFile.set(useShooterSensorTag, true);
//        }
//        telemetry.addData("*2 left/right trigger bumper => " + useShooterSensorTag, optionsFile.get(useShooterSensorTag, Boolean.class, null));
//
//    }
//
//    public double pow10floor(double x) {
//        return Math.pow(10, Math.floor(Math.log(x) / Math.log(10)));
//    }
//
//    @Override
//    protected Function getJoystickScalingFunction() {
//        return Functions.none();
//    }
//
//}
