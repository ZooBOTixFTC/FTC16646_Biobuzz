package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/**
 * Shared building blocks for the v2 test autos (Claude's first drafts, Nov 24).
 * Every auto here is RED. Change @Config numbers on Dashboard BEFORE pressing INIT.
 */
@Config
public abstract class AUTO_Base_v2 extends Robot_Auto {

    // ---- Shooting ----
    public static double SHOOT_RPM = 2250;
    public static int RAMP_STEP_MS = 250;      // preload: intake ramps up in steps this long
    public static int SETTLE_MS = 400;         // stand still before opening the stopper
    public static int STOPPER_OPEN_MS = 520;   // wait after opening the stopper
    public static int FEED_MS = 2500;          // intake feeds this long (3500 = slow but sure, 2000 = fast)

    // ---- Auto aim (camera) before a shot ----
    public static boolean USE_AIM = true;      // false = shoot where the path ends, like before
    public static double AIM_SIGN = 1;         // flip to -1 if it turns AWAY from the hive
    public static double AIM_MAX_DEG = 25;     // ignore the camera if it says more than this (bad read)

    // ---- Flower ----
    public static double INTAKE_FLOWER_POWER = 0.75;
    public static int SHIMMY_COUNT = 2;        // 0 = no shimmy
    public static double SHIMMY_IN = 2;        // slide this far left and right
    public static double SHIMMY_DEG = 0;       // or twist this many degrees each way (0 = slide instead)
    public static int FLOWER_WAIT_MS = 250;    // extra wait at the flower after the shimmy

    // ---- Plow ----
    public static int PLOW_MOVE_MS = 400;      // the plow only moves while the robot stands still

    protected final ElapsedTime m_clock = new ElapsedTime();

    // ---------------- Small building blocks ----------------

    /** Writes "what: 12.3 s" to the Driver Station log. */
    protected InstantCommand lap(String what) {
        return new InstantCommand(() -> telemetry.log().add("%s: %.1f s", what, m_clock.seconds()));
    }

    /** Drives a Road Runner path (the scheduler freezes until it finishes). */
    protected InstantCommand run(Action path) {
        return new InstantCommand(() -> Actions.runBlocking(path));
    }

    protected InstantCommand startClock() {
        return new InstantCommand(m_clock::reset);
    }

    protected InstantCommand intake(double power) {
        return new InstantCommand(() -> m_robot.m_intake.setMotorPower(power));
    }

    protected SequentialCommandGroup plowDown() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
                new WaitCommand(PLOW_MOVE_MS));
    }

    protected SequentialCommandGroup plowUp() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheCloud()),
                new WaitCommand(PLOW_MOVE_MS));
    }

    protected Command spinUpAndWait() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp(SHOOT_RPM)),
                new WaitUntilCommand(() -> m_robot.m_shooterPollen.atSpeed()).withTimeout(2000));
    }

    /** Preload shot: same ramp as Auto 1. Leaves the intake running at 0.8. */
    protected SequentialCommandGroup shootPreload() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(RAMP_STEP_MS), intake(0.1),
                new WaitCommand(RAMP_STEP_MS), intake(0.2),
                new WaitCommand(RAMP_STEP_MS), intake(0.3),
                new WaitCommand(RAMP_STEP_MS), intake(0.4),
                new WaitCommand(RAMP_STEP_MS), intake(0.5),
                new WaitCommand(RAMP_STEP_MS), intake(0.8),
                new WaitCommand(700),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed()),
                intake(0.8));
    }

    /** Normal shot: settle, open, feed, close. */
    protected SequentialCommandGroup shootPollen() {
        return new SequentialCommandGroup(
                new WaitCommand(SETTLE_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(STOPPER_OPEN_MS),
                intake(0.5),
                new WaitCommand(FEED_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed()));
    }

    /** True if the camera sees OUR hive, right side up (same check as the TeleOp aim). */
    protected boolean hiveUpright() {
        return m_robot.m_vision.getHiveCluster(true) != null;
    }

    /**
     * One-time camera aim: read the bearing once, then turn that far.
     * Does nothing if USE_AIM is false, the hive isn't seen, or the reading looks wrong.
     */
    protected InstantCommand aim() {
        return new InstantCommand(() -> {
            if (!USE_AIM) return;
            AprilTagClusterDetection hive = m_robot.m_vision.getHiveCluster(true);
            if (hive == null) {
                telemetry.log().add("Aim: hive not seen, shooting anyway");
                return;
            }
            double error = hive.ftcPose.bearing - VisionConstants.kAimOffsetDeg;
            if (Math.abs(error) > AIM_MAX_DEG) {
                telemetry.log().add("Aim: %.0f deg looks wrong, skipped", error);
                return;
            }
            Pose2d now = m_robot.drivetrain.getPoseEstimate();
            double target = now.heading.toDouble() + Math.toRadians(error) * AIM_SIGN;
            Actions.runBlocking(m_robot.drivetrain.getDrive().actionBuilder(now).turnTo(target).build());
            telemetry.log().add("Aim: turned %.1f deg", error * AIM_SIGN);
        });
    }

    /** Small side-to-side (or twist) wiggle at a spot, like the TeleOp shimmy. */
    protected Action shimmy(Pose2d at) {
        TrajectoryActionBuilder b = m_robot.drivetrain.getDrive().actionBuilder(at);
        double x = at.position.x, y = at.position.y, h = at.heading.toDouble();
        for (int i = 0; i < SHIMMY_COUNT; i++) {
            if (SHIMMY_DEG > 0) {
                b = b.turnTo(h + Math.toRadians(SHIMMY_DEG)).turnTo(h - Math.toRadians(SHIMMY_DEG));
            } else {
                b = b.strafeTo(new Vector2d(x + SHIMMY_IN, y)).strafeTo(new Vector2d(x - SHIMMY_IN, y));
            }
        }
        if (SHIMMY_DEG > 0) {
            b = b.turnTo(h);
        } else {
            b = b.strafeTo(new Vector2d(x, y));
        }
        return b.build();
    }

    /** At the flower: intake at plow power, shimmy, short wait. */
    protected SequentialCommandGroup grabFlower(Pose2d at) {
        Action wiggle = shimmy(at);
        return new SequentialCommandGroup(
                intake(INTAKE_FLOWER_POWER),
                run(wiggle),
                new WaitCommand(FLOWER_WAIT_MS));
    }

    protected static Pose2d pose(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }
}
