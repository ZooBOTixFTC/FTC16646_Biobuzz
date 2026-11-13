package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Test: Plow Servo  (States Prep, Task 1.2)
 *
 * Moves the plow servo with the gamepad and shows its position (0.0 to 1.0),
 * so you can find the rough UP and DOWN numbers for SUB_FlowPlow.
 *
 *   D-pad up / down   move the plow slowly (hold)
 *   Right bumper      nudge up by 0.01 (one tap)
 *   Left bumper       nudge down by 0.01 (one tap)
 *   Y                 save the current position as UP
 *   A                 save the current position as DOWN
 *
 * Write the saved numbers down, then put them in SUB_FlowPlow.
 */
@TeleOp(name = "Test: Plow Servo", group = "Test")
public class TELEOP_PlowServo extends LinearOpMode {

    // Must match the servo name in the Control Hub configuration exactly.
    static final String PLOW_SERVO = "flowPlowServo";

    static final double START = 0.0;   // where the code thinks the plow is at the start (0 = up)
    static final double SLOW = 0.005;  // how far each loop moves while the D-pad is held
    static final double NUDGE = 0.01;  // how far one bumper tap moves

    @Override
    public void runOpMode() {
        Servo plow = hardwareMap.get(Servo.class, PLOW_SERVO);

        double position = START;
        double savedUp = -1;     // -1 means "not saved yet"
        double savedDown = -1;
        boolean rbWas = false;
        boolean lbWas = false;
        boolean active = false;  // the servo is not commanded until the first input

        telemetry.addLine("The servo does NOT move until you press the D-pad or a bumper.");
        telemetry.addLine("Hold the plow UP (position 0) before you start; it moves slowly from there.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Hold the D-pad to move slowly
            if (gamepad1.dpad_up) position += SLOW;
            if (gamepad1.dpad_down) position -= SLOW;

            // Tap a bumper to move a tiny bit (only once per press)
            if (gamepad1.right_bumper && !rbWas) position += NUDGE;
            if (gamepad1.left_bumper && !lbWas) position -= NUDGE;
            rbWas = gamepad1.right_bumper;
            lbWas = gamepad1.left_bumper;

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.right_bumper || gamepad1.left_bumper) {
                active = true;
            }

            position = Range.clip(position, 0.0, 1.0);   // servos only go from 0 to 1
            if (active) {
                plow.setPosition(position);
            }

            if (gamepad1.y) savedUp = position;
            if (gamepad1.a) savedDown = position;

            telemetry.addData("Position", "%.3f%s", position, active ? "" : "  (servo not moved yet)");
            telemetry.addData("Saved UP (Y)", savedUp < 0 ? "not yet" : String.format("%.3f", savedUp));
            telemetry.addData("Saved DOWN (A)", savedDown < 0 ? "not yet" : String.format("%.3f", savedDown));
            telemetry.addLine();
            telemetry.addLine("If the servo buzzes, it's pushing against something.");
            telemetry.addLine("Back off a little: buzzing for long can burn it out.");
            telemetry.update();

            sleep(20);
        }
    }
}