package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ftc7393 on 11/18/2017.
 */

public class TestTeleOp extends LinearOpMode{
    /* Declare OpMode members. */
     /* Declare OpMode members. */
    TestHardware robot           = new TestHardware();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            // Combine drive and turn for blended motion.

            // Normalize the values so neither exceed +/- 1.0

            // Output the safe vales to the motor drives.


            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.a)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.b)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.relicGrabber.setPosition(robot.MID_SERVO + clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

}
