package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.DriveParamsStore;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

/**
 * Dedicated shooter PID tuning OpMode (gamepad 1).
 *
 *   RIGHT BUMPER      toggle shooter on / off
 *   LEFT STICK L/R    lower / raise target RPM (hold, faster the further you push)
 *   D-PAD UP / DOWN   raise / lower kP
 *   D-PAD RIGHT/LEFT  raise / lower kD
 *   A                 SAVE shooter + drivetrain values to the Control Hub
 *   B                 reset "lowest RPM" and "recovery time" readouts
 *
 * Tip: feed a ball while it's at speed and watch how far RPM drops
 * and how long it takes to get back within tolerance.
 */
@TeleOp(name = "Shooter PID Tuner", group = "Tuning")
public class TUNER_shooter extends LinearOpMode {

    // TODO: must match the shooter motor names in your robot configuration
    private static final String SHOOTER_MOTOR_1_NAME = "shooter1_motor";

    private static final double MAX_TARGET_RPM = 2000;   // wheel free speed with 3:1 gearing
    private static final double STICK_RPM_PER_SEC = 400; // target change rate at full stick
    private static final double STICK_DEADBAND = 0.1;

    @Override
    public void runOpMode() {
        // Clear any subsystems left registered by a previous OpMode
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SUB_Shooter shooter = new SUB_Shooter(hardwareMap);

        boolean running = false;
        boolean lastRB = false, lastUp = false, lastDown = false,
                lastLeft = false, lastRight = false, lastA = false, lastB = false;

        double lowestRpmSinceAtSpeed = Double.NaN;
        double lastRecoveryTime = Double.NaN;
        boolean wasAtSpeed = false;
        ElapsedTime recoveryTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("SHOOTER PID TUNER - press START");
        telemetry.addData("Save file", shooter.getSaveStatus());
        telemetry.update();
        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // ---- Right bumper: toggle shooter ----
            if (gamepad1.right_bumper && !lastRB) {
                running = !running;
                if (running) shooter.spinUp(); else shooter.stop();
            }

            // ---- Left stick X: adjust target RPM ----
            double stickX = gamepad1.left_stick_x;
            if (Math.abs(stickX) > STICK_DEADBAND) {
                SUB_Shooter.TARGET_RPM = Range.clip(
                        SUB_Shooter.TARGET_RPM + stickX * STICK_RPM_PER_SEC * dt,
                        0, MAX_TARGET_RPM);
            }

            // ---- D-pad: tune P and D ----
            if (gamepad1.dpad_up && !lastUp)       shooter.adjustP(SUB_Shooter.P_STEP);
            if (gamepad1.dpad_down && !lastDown)   shooter.adjustP(-SUB_Shooter.P_STEP);
            if (gamepad1.dpad_right && !lastRight) shooter.adjustD(SUB_Shooter.D_STEP);
            if (gamepad1.dpad_left && !lastLeft)   shooter.adjustD(-SUB_Shooter.D_STEP);

            // ---- A: save values ----
            if (gamepad1.a && !lastA) {
                shooter.saveGains();
                DriveParamsStore.save();
            }

            // ---- B: reset shot-recovery stats ----
            if (gamepad1.b && !lastB) {
                lowestRpmSinceAtSpeed = Double.NaN;
                lastRecoveryTime = Double.NaN;
            }

            lastRB = gamepad1.right_bumper;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            // Run the PIDF (no scheduler in this OpMode, so call periodic() ourselves)
            shooter.periodic();

            // ---- Shot recovery tracking ----
            double rpm = shooter.getRpm();
            boolean atSpeed = shooter.atSpeed();
            if (running) {
                if (wasAtSpeed && !atSpeed) {
                    // just dropped out of tolerance (probably a shot)
                    recoveryTimer.reset();
                    lowestRpmSinceAtSpeed = rpm;
                }
                if (!atSpeed && !Double.isNaN(lowestRpmSinceAtSpeed)) {
                    lowestRpmSinceAtSpeed = Math.min(lowestRpmSinceAtSpeed, rpm);
                }
                if (!wasAtSpeed && atSpeed && !Double.isNaN(lowestRpmSinceAtSpeed)) {
                    lastRecoveryTime = recoveryTimer.seconds();
                }
            }
            wasAtSpeed = atSpeed;

            // ---- Screen ----
            telemetry.addLine("=== SHOOTER PID TUNER ===");
            telemetry.addData("Shooter", running ? "RUNNING" : "OFF (press RIGHT BUMPER)");
            telemetry.addLine();
            telemetry.addLine("LEFT STICK L/R = adjust target");
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm() > 0 ? shooter.getTargetRpm() : SUB_Shooter.TARGET_RPM);
            telemetry.addData("Actual RPM (avg)", "%.0f", rpm);
            telemetry.addData("Motor 1 / 2 RPM", "%.0f / %.0f", shooter.getMotor1Rpm());
            telemetry.addData("Error RPM", "%.0f", shooter.getTargetRpm() - rpm);
            telemetry.addData("At Speed", atSpeed);
            telemetry.addLine();
            telemetry.addData("Lowest RPM after shot", Double.isNaN(lowestRpmSinceAtSpeed) ? "--" : String.format("%.0f", lowestRpmSinceAtSpeed));
            telemetry.addData("Recovery time (s)", Double.isNaN(lastRecoveryTime) ? "--" : String.format("%.2f", lastRecoveryTime));
            telemetry.addLine("(B = reset these)");
            telemetry.addLine();
            telemetry.addData("P (Dpad Up/Down)", "%.5f", SUB_Shooter.kP);
            telemetry.addData("D (Dpad Right/Left)", "%.5f", SUB_Shooter.kD);
            telemetry.addData("F (feedforward)", "%.6f", SUB_Shooter.kF);
            telemetry.addLine();
            telemetry.addLine("Press A to SAVE all values");
            telemetry.addData("Shooter Save", shooter.getSaveStatus());
            telemetry.addData("Drive Save", DriveParamsStore.status);
            telemetry.update();
        }

        shooter.stop();
        shooter.periodic();
        CommandScheduler.getInstance().reset();
    }
}