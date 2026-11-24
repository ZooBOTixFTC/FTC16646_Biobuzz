package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * AUTO 1 v2 (TEST): garden side, 3 shots + park. Claude's best guess, Nov 24.
 * Same route as "Red Solo". Changes:
 *   1. Garden: intake stays ON during the corner pause (it was off), pause back to 1 s.
 *   2. Flower: intake at plow power + a shimmy instead of the plain wait.
 *   3. Shot 3: one-time camera aim (USE_AIM), plow goes up while standing still.
 *   4. Safety: if it's late, skip shot 3 and go park (protects the park points).
 */
@Config
@Autonomous(name = "Red Solo v2 (TEST)", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_RedSolo_v2 extends AUTO_Base_v2 {

    // Same numbers as Red Solo after the Nov 23 gym tuning (code numbers, "bent")
    public static double A1_SHOT2_X = -45.5;
    public static double A1_SHOT2_Y = 75;
    public static double A1_SHOT2_HEADING = -80;
    public static double A1_HIVE_PASS_X = -72;
    public static double A1_FLOWER_X_NUDGE = 0;   // + or - to center the plow on the flower base
    public static double A1_FLOWER_Y = 84;
    public static double A1_PARK_X = -95;
    public static double A1_PARK_Y = 47;
    public static int A1_CORNER_PAUSE_MS = 1000;  // intake ON during this pause
    public static double A1_SKIP_SHOT3_AFTER = 24; // seconds: later than this, skip shot 3 and park
    // true = drive straight up the lane BACKWARD (heading -90), already almost facing the shot heading.
    // false = the old path (heading swings 90 -> 50 -> 0 -> -80, which is mostly strafing).
    // NOTE: a new path slips differently, so shot 2 may land somewhere new. Re-check SHOT2 numbers.
    public static boolean A1_DRIVE_BACKWARD = true;
    public static double A1_LANE_HEADING = -90;

    private Action toPreload, toCorner, cornerToShot2, toFlower, flowerToShot, park;

    private Pose2d shot2() { return pose(A1_SHOT2_X, A1_SHOT2_Y, A1_SHOT2_HEADING); }
    private Pose2d flower() { return pose(A1_SHOT2_X + A1_FLOWER_X_NUDGE, A1_FLOWER_Y, -90); }

    @Override
    public void prebuildTasks() {
        setStartingPose(pose(-12, -63, 90));
        GlobalVariables.m_red = true;

        toPreload = m_robot.drivetrain.getDrive().actionBuilder(getStartingPose())
                .lineToY(-61)
                .build();

        toCorner = m_robot.drivetrain.getDrive().actionBuilder(pose(-12, -61, 90))
                .splineToLinearHeading(pose(-60, -56.25, 45), Math.toRadians(135))
                .strafeTo(new Vector2d(-81, -56.25))
                .turnTo(Math.toRadians(90))
                .build();

        // No waitSeconds here any more: the pause happens before this, with the intake on
        if (A1_DRIVE_BACKWARD) {
            cornerToShot2 = m_robot.drivetrain.getDrive().actionBuilder(pose(-81, -56.25, 90))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(pose(A1_HIVE_PASS_X, -20, A1_LANE_HEADING), Math.toRadians(90))
                    .splineToLinearHeading(pose(A1_HIVE_PASS_X, 40, A1_LANE_HEADING), Math.toRadians(90))
                    .splineToLinearHeading(shot2(), Math.toRadians(90))
                    .build();
        } else {
            cornerToShot2 = m_robot.drivetrain.getDrive().actionBuilder(pose(-81, -56.25, 90))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(pose(A1_HIVE_PASS_X, -20, 50), Math.toRadians(90))
                    .splineToSplineHeading(pose(A1_HIVE_PASS_X, 40, 0), Math.toRadians(90))
                    .splineToSplineHeading(shot2(), Math.toRadians(90))
                    .build();
        }

        toFlower = m_robot.drivetrain.getDrive().actionBuilder(shot2())
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(flower().position.x, flower().position.y))
                .build();

        flowerToShot = m_robot.drivetrain.getDrive().actionBuilder(flower())
                .strafeTo(new Vector2d(A1_SHOT2_X, A1_SHOT2_Y))
                .turnTo(Math.toRadians(A1_SHOT2_HEADING))
                .build();

        park = m_robot.drivetrain.getDrive().actionBuilder(shot2())
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(A1_PARK_X, A1_PARK_Y))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                startClock(),
                // 1. Preload
                run(toPreload),
                spinUpAndWait(),
                lap("Shot 1 starts"),
                shootPreload(),                       // leaves the intake running
                lap("Shot 1 done"),

                // 2. Garden: pause with the intake STILL ON, then turn it off
                run(toCorner),
                lap("In the corner"),
                new WaitCommand(A1_CORNER_PAUSE_MS),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),
                lap("Leaving the corner"),
                run(cornerToShot2),
                lap("At shot 2"),

                // 3. Shot 2 (plow goes down while standing still)
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
                aim(),
                shootPollen(),
                lap("Shot 2 done"),

                // 4. Flower
                run(toFlower),
                lap("At the flower"),
                grabFlower(flower()),
                lap("Flower done"),

                // 5. Shot 3, unless it's too late (then go straight to park)
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                run(flowerToShot),
                                lap("At shot 3"),
                                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheCloud()),
                                aim(),
                                shootPollen(),
                                lap("Shot 3 done")),
                        new SequentialCommandGroup(
                                lap("Late: skipping shot 3"),
                                run(flowerToShot),
                                plowUp()),
                        () -> m_clock.seconds() < A1_SKIP_SHOT3_AFTER),

                // 6. Park
                run(park),
                lap("Done")
        );
    }
}
