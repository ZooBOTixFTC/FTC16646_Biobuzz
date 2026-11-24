package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * AUTO 3 v1 (TEST): garden side, wait for the hive. Claude's first draft, Nov 24.
 * Run it when the partner has a strong auto that starts FLOWER side and tips the hive.
 *
 *   Preload + garden (same as Auto 1), drive to the wait spot, wait for the hive upright.
 *   Upright before A3_FULL_SEC:   shoot, side-wall flower, shoot again if time, park.
 *   Upright before A3_GIVE_UP_SEC: shoot, park.
 *   Never upright:                 park.
 *
 * Spots after the garden corner are CODE numbers. The corner slip "bends" them by about
 * 17 in (code x = real x - 17 near the garden). Guesses: tune on Dashboard.
 */
@Config
@Autonomous(name = "Red Garden Wait v1 (TEST)", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_RedGardenWait_v1 extends AUTO_Base_v2 {

    public static int A3_CORNER_PAUSE_MS = 1000;
    public static double A3_WAIT_X = -57, A3_WAIT_Y = -44, A3_WAIT_HEADING = 75;   // real about (-40, -44)
    public static double A3_FLOWER_X = -76, A3_FLOWER_Y = -24;                      // real about (-59, -24), back to the wall
    public static double A3_PARK_X = -80, A3_PARK_Y = 26;                           // real about (-63, 26)
    public static double A3_LANE_X = -72;
    public static double A3_FULL_SEC = 13;        // hive upright before this: do the flower too
    public static double A3_SHOT3_LATEST = 20;    // leaving the flower after this: skip the last shot
    public static double A3_GIVE_UP_SEC = 21;     // hive still not upright: stop waiting and park

    private Action toPreload, toCorner, cornerToWait, toFlower, flowerToWait, waitToPark;
    private double m_hiveSeenAt = -1;

    private Pose2d waitSpot() { return pose(A3_WAIT_X, A3_WAIT_Y, A3_WAIT_HEADING); }
    private Pose2d flower() { return pose(A3_FLOWER_X, A3_FLOWER_Y, 0); }

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

        cornerToWait = m_robot.drivetrain.getDrive().actionBuilder(pose(-81, -56.25, 90))
                .strafeTo(new Vector2d(A3_WAIT_X, A3_WAIT_Y))
                .turnTo(Math.toRadians(A3_WAIT_HEADING))
                .build();

        toFlower = m_robot.drivetrain.getDrive().actionBuilder(waitSpot())
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(A3_WAIT_X, A3_FLOWER_Y))
                .strafeTo(new Vector2d(A3_FLOWER_X, A3_FLOWER_Y))   // back straight into the wall flower
                .build();

        flowerToWait = m_robot.drivetrain.getDrive().actionBuilder(flower())
                .strafeTo(new Vector2d(A3_WAIT_X, A3_FLOWER_Y))     // pull straight out first
                .strafeTo(new Vector2d(A3_WAIT_X, A3_WAIT_Y))
                .turnTo(Math.toRadians(A3_WAIT_HEADING))
                .build();

        // Up the left lane (away from the hive), then into the loading zone
        waitToPark = m_robot.drivetrain.getDrive().actionBuilder(waitSpot())
                .strafeTo(new Vector2d(A3_LANE_X, A3_WAIT_Y))
                .strafeTo(new Vector2d(A3_LANE_X, A3_PARK_Y))
                .strafeTo(new Vector2d(A3_PARK_X, A3_PARK_Y))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup flowerPart = new SequentialCommandGroup(
                plowDown(),
                run(toFlower),
                lap("At the flower"),
                grabFlower(flower()),
                run(flowerToWait),
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheCloud()),
                lap("Back at the shot spot"),
                new ConditionalCommand(
                        new SequentialCommandGroup(aim(), shootPollen(), lap("Shot 3 done")),
                        lap("Too late for shot 3"),
                        () -> m_clock.seconds() < A3_SHOT3_LATEST + 2));   // +2 = drive back from the flower

        SequentialCommandGroup hiveUprightPart = new SequentialCommandGroup(
                lap("Hive upright"),
                aim(),
                shootPollen(),
                lap("Shot 2 done"),
                new ConditionalCommand(flowerPart, lap("No time for the flower"),
                        () -> m_hiveSeenAt < A3_FULL_SEC));

        return new SequentialCommandGroup(
                startClock(),
                run(toPreload),
                spinUpAndWait(),
                shootPreload(),
                lap("Shot 1 done"),
                run(toCorner),
                lap("In the corner"),
                new WaitCommand(A3_CORNER_PAUSE_MS),          // intake still on from the preload
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),
                run(cornerToWait),
                lap("Waiting for the hive"),
                new WaitUntilCommand(() -> {
                    if (hiveUpright()) {
                        m_hiveSeenAt = m_clock.seconds();
                        return true;
                    }
                    return m_clock.seconds() > A3_GIVE_UP_SEC;
                }),
                new ConditionalCommand(hiveUprightPart, lap("Hive never upright"), () -> m_hiveSeenAt >= 0),
                run(waitToPark),
                lap("Done")
        );
    }
}
