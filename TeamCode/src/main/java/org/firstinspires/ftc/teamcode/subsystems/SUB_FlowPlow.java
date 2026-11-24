package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * FlowPlow: one servo.
 *   FlowPlowToTheCloud()  = up   (the hard-set zero)
 *   FlowPlowToTheGround() = down (always a POSITIVE position)
 *
 * The plow moves SLOWLY toward its target (a little every loop) so it never slams.
 * Make the plow slower or faster with MAX_STEP.
 *
 * @Config: these numbers can be changed live from FTC Dashboard (Task 3).
 * Live changes are lost on restart, so copy good numbers back into this file.
 */
@Config
public class SUB_FlowPlow extends SubsystemBase {

    // ---- Tune these ----
    // Up is the hard-set zero. Down must always be a POSITIVE number.
    // "public static" (not final) so Dashboard can change them.
    public static double CLOUD_POS = 0.15;    // up position (hard set to 0)
    public static double GROUND_POS = 0.60;
    public static double FLOPPY_FLOW= 0.5; // down position (positive) - set this to your real down angle

    // How far the servo moves each loop. Smaller = slower and gentler.
    // About 0.005 takes roughly 2 to 4 seconds to go from 0 to 0.5 at normal loop speed.
    public static double MAX_STEP = 0.1;

    private final Servo servo;
    private double target = CLOUD_POS;
    private double current   = CLOUD_POS;

    public SUB_FlowPlow(OpMode opMode) {
        servo = opMode.hardwareMap.get(Servo.class, "flowPlowServo");

        // Use REVERSE here if a positive position moves the plow UP instead of down
        servo.setDirection(Servo.Direction.FORWARD);

        servo.setPosition(CLOUD_POS);   // start up (the zero position)
    }

    @Override
    public void periodic() {
        if (servo == null) return;

        if (current < target) {
            current = Math.min(current + MAX_STEP, target);
        } else if (current > target) {
            current = Math.max(current - MAX_STEP, target);
        }
        servo.setPosition(current);
    }

    /** Moves FlowPlow up (position 0). */
    public void FlowPlowToTheCloud() {
        target = CLOUD_POS;
    }

    /** Moves FlowPlow down (positive position). */
    public void FlowPlowToTheGround() {
        target = GROUND_POS;

    }

    public void FlowPlowFlopsAround() {
        target = FLOPPY_FLOW;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("FlowPlow target / now", "%.2f / %.2f", target, current);
    }
}