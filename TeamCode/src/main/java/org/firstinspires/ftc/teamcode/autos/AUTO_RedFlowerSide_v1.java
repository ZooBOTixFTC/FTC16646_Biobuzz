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
 * AUTO 2 v1 (TEST): flower side, 2 shots, with a backup plan. Claude's first draft, Nov 24.
 * Run it when the partner has a strong auto that starts on the GARDEN side.
 *
 *   Pull forward, watch for our hive (upright) until A2_CUTOFF_SEC.
 *   Seen:     aim + shoot preload, back into the end-wall flower, pull out, aim + shoot, park.
 *   Not seen: drive down the left lane to the garden, collect it, come back up and park.
 *
 * All spots are REAL field inches (this start never touches the garden corner, so the
 * numbers shouldn't be "bent" like Auto 1's). They are guesses: tune on Dashboard.
 */
@Config
@Autonomous(name = "Red Flower Side v1 (TEST)", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_RedFlowerSide_v1 extends AUTO_Base_v2 {

    public static double A2_START_X = -12.5, A2_START_Y = 63;
    public static double A2_WATCH_Y = 56;              // pull forward to here and watch for the hive
    public static double A2_CUTOFF_SEC = 12;           // give up waiting for the hive at this time
    public static double A2_FLOWER_X = -24, A2_FLOWER_Y = 60;    // backed into the end-wall flower
    public static double A2_SHOT_X = -12, A2_SHOT_Y = 52;        // second shot (near where the preload shot works)
    public static double A2_SHOT_HEADING = -95;     // camera aim fixes the rest
    public static double A2_PARK_X = -63, A2_PARK_Y = 50;
    // Backup route (hive not seen)
    public static double A2_LANE_X = -40;
    public static double A2_GARDEN_X = -66, A2_GARDEN_Y = -58;
    public static int A2_GARDEN_PAUSE_MS = 1000;

    private Action toWatch, toFlower, flowerToShot, shotToPark, toGarden, gardenToPark;
    private boolean m_hiveSeen = false;

    private Pose2d watch() { return pose(A2_START_X, A2_WATCH_Y, -90); }
    private Pose2d flower() { return pose(A2_FLOWER_X, A2_FLOWER_Y, -90); }
    private Pose2d shot() { return pose(A2_SHOT_X, A2_SHOT_Y, A2_SHOT_HEADING); }

    @Override
    public void prebuildTasks() {
        setStartingPose(pose(A2_START_X, A2_START_Y, -90));
        GlobalVariables.m_red = true;

        toWatch = m_robot.drivetrain.getDrive().actionBuilder(getStartingPose())
                .lineToY(A2_WATCH_Y)
                .build();

        // Main plan
        toFlower = m_robot.drivetrain.getDrive().actionBuilder(watch())
                .strafeTo(new Vector2d(A2_FLOWER_X, A2_WATCH_Y))
                .strafeTo(new Vector2d(A2_FLOWER_X, A2_FLOWER_Y))   // back straight in
                .build();

        flowerToShot = m_robot.drivetrain.getDrive().actionBuilder(flower())
                .strafeTo(new Vector2d(A2_FLOWER_X, A2_SHOT_Y))     // pull straight out first
                .strafeTo(new Vector2d(A2_SHOT_X, A2_SHOT_Y))
                .turnTo(Math.toRadians(A2_SHOT_HEADING))
                .build();

        shotToPark = m_robot.drivetrain.getDrive().actionBuilder(shot())
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(A2_PARK_X, A2_PARK_Y))
                .build();

        // Backup plan: down the left lane, turn so the intake (back) faces the corner
        toGarden = m_robot.drivetrain.getDrive().actionBuilder(watch())
                .strafeTo(new Vector2d(A2_LANE_X, A2_WATCH_Y))
                .strafeTo(new Vector2d(A2_LANE_X, -30))
                .turnTo(Math.toRadians(45))
                .strafeTo(new Vector2d(A2_GARDEN_X, A2_GARDEN_Y))
                .build();

        gardenToPark = m_robot.drivetrain.getDrive().actionBuilder(pose(A2_GARDEN_X, A2_GARDEN_Y, 45))
                .strafeTo(new Vector2d(A2_LANE_X, -30))
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(A2_LANE_X, A2_PARK_Y))
                .strafeTo(new Vector2d(A2_PARK_X, A2_PARK_Y))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup mainPlan = new SequentialCommandGroup(
                lap("Hive seen"),
                aim(),
                shootPreload(),
                lap("Shot 1 done"),
                plowDown(),
                run(toFlower),
                lap("At the flower"),
                grabFlower(flower()),
                run(flowerToShot),
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheCloud()),
                lap("At shot 2"),
                aim(),
                shootPollen(),
                lap("Shot 2 done"),
                run(shotToPark),
                lap("Done"));

        SequentialCommandGroup backupPlan = new SequentialCommandGroup(
                lap("No hive: backup"),
                intake(0.8),
                run(toGarden),
                lap("In the garden"),
                new WaitCommand(A2_GARDEN_PAUSE_MS),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),
                run(gardenToPark),
                lap("Done (backup)"));

        return new SequentialCommandGroup(
                startClock(),
                run(toWatch),
                spinUpAndWait(),
                lap("Watching for the hive"),
                new WaitUntilCommand(() -> {
                    m_hiveSeen = hiveUpright();
                    return m_hiveSeen || m_clock.seconds() > A2_CUTOFF_SEC;
                }),
                new ConditionalCommand(mainPlan, backupPlan, () -> m_hiveSeen)
        );
    }
}
