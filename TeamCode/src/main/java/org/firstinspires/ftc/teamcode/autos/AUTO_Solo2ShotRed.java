package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;


@Config     // the numbers below can be changed live on FTC Dashboard (change them BEFORE pressing INIT)
@Autonomous(name = "Red Solo", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_Solo2ShotRed extends Robot_Auto {

    // ---- Tune these on Dashboard, then copy the good numbers back here ----
    // These are CODE numbers, not real field inches (the robot isn't fully tuned).
    // Shot 2 (and shot 3): in front of the flower
    public static double SHOT2_X = -42.5;
    public static double SHOT2_Y = 75;
    public static double SHOT2_HEADING = -80;    // degrees
    // Side lane on the way up: more negative = further from the hive stand (was -60)
    public static double PASS_X = -72;
    // Flower: how far it backs up into the flower (was 78; the video showed 3-4 in short)
    public static double FLOWER_Y = 82;
    // Park: in the loading zone, on the half nearest our start wall
    public static double PARK_X = -80;
    public static double PARK_Y = 48;

    // ---- Times ----
    public static int SHOT2_FEED_MS = 2000;      // how long shot 2 and shot 3 feed
    public static double CORNER_PAUSE_SEC = 0.5; // pause in the garden corner
    public static int RAMP_STEP_MS = 150;        // shot 1 intake ramp, each step
    public static int SHOT2_SETTLE_MS = 200;     // settle before shot 2 and shot 3
    public static int FLOWER_INTAKE_MS = 500;    // wait at the flower to pull pollen in


    private Action moveForward, pickUpCorner, getPickUpCorner2, intakeFLower, backToShot, park;

    // Lap times: shown in the Driver Station log, so you can see where the time goes
    private final ElapsedTime m_clock = new ElapsedTime();


    // ==================== SPOTS ====================
    // Methods (not variables) so they read the newest Dashboard numbers every time.

    /** Where shots 2 and 3 happen. */
    private Pose2d shot2Spot() {
        return new Pose2d(SHOT2_X, SHOT2_Y, Math.toRadians(SHOT2_HEADING));
    }

    /** Backed into the flower, facing -90. */
    private Pose2d flowerSpot() {
        return new Pose2d(SHOT2_X, FLOWER_Y, Math.toRadians(-90));
    }


    // ==================== PATHS ====================
    // Rule: every path starts exactly where the one before it ended.

    /** From shot 2: turn so the plow faces the flower, then back into it. Ends at flowerSpot(). */
    private Action flowerPath() {
        return m_robot.drivetrain.getDrive()
                .actionBuilder(shot2Spot())
                .turnTo(Math.toRadians(-90))
                .lineToY(FLOWER_Y)
                .build();
    }

    /** From the flower: drive back out and turn to the shot heading. Ends at shot2Spot(). */
    private Action backToShot2Path() {
        return m_robot.drivetrain.getDrive()
                .actionBuilder(flowerSpot())
                .lineToY(SHOT2_Y)
                .turnTo(Math.toRadians(SHOT2_HEADING))
                .build();
    }

    /** From shot 2/3: one straight slide to the loading zone. Keeps the same heading. */
    private Action parkPath() {
        return m_robot.drivetrain.getDrive()
                .actionBuilder(shot2Spot())
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .build();
    }


    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-12, -63, Math.toRadians(90)));
        GlobalVariables.m_red = true;

        // Stop 1: drive up 2" to shoot preload
        moveForward = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(-61)
                .build();

        // Stop 2: curve toward the corner, turning to 45° so the intake (back) points into the corner
        pickUpCorner = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-12, -61, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-60, -56.25, Math.toRadians(45)), Math.toRadians(135))
                .strafeTo(new Vector2d(-81, -56.25))   // stop 3: slide into the corner, stays at 45°
                .turnTo(Math.toRadians(90))            // turn in place to face 90°
                .build();

        // Stops 3-5: wait in the corner, then drive up the side lane to shot 2
        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-81, -56.25, Math.toRadians(90)))
                .waitSeconds(CORNER_PAUSE_SEC)         // let the garden pollen in
                .setTangent(Math.toRadians(90))        // start moving out of the corner
                .splineToSplineHeading(new Pose2d(PASS_X, -20, Math.toRadians(50)), Math.toRadians(90))  // stop 4
                .splineToSplineHeading(new Pose2d(PASS_X, 40, Math.toRadians(0)), Math.toRadians(90))    // pass the hive stand
                .splineToSplineHeading(shot2Spot(), Math.toRadians(90))                                  // stop 5: shot 2
                .build();

        intakeFLower = flowerPath();
        backToShot   = backToShot2Path();
        park         = parkPath();
    }


    // ==================== BUILDING BLOCKS ====================

    /** Writes "what: 12.3 s" to the Driver Station log (it stays on screen). */
    private InstantCommand lap(String what) {
        return new InstantCommand(() -> telemetry.log().add("%s: %.1f s", what, m_clock.seconds()));
    }

    /** Runs a Road Runner path as one step. */
    private InstantCommand run(Action path) {
        return new InstantCommand(() -> Actions.runBlocking(path));
    }

    /** Shot 1: open the gate, ramp the intake up slowly, close the gate. */
    private SequentialCommandGroup shootPreload() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(RAMP_STEP_MS),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.1)),
                new WaitCommand(RAMP_STEP_MS),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.2)),
                new WaitCommand(RAMP_STEP_MS),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.3)),
                new WaitCommand(RAMP_STEP_MS),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.4)),
                new WaitCommand(RAMP_STEP_MS),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.5)),
                new WaitCommand(RAMP_STEP_MS),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.8)),
                new WaitCommand(700),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed()),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.8)));
    }

    /**
     * Shots 2 and 3: settle, open the gate, start the intake 0.52 s later, feed, close the gate.
     * A METHOD, so every call makes a brand-new copy. (FTCLib can't run the same command object twice.)
     * Leaves the intake running at 0.5, which is what pulls pollen in at the flower.
     */
    private SequentialCommandGroup shootPollen() {
        return new SequentialCommandGroup(
                new WaitCommand(SHOT2_SETTLE_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(520),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.5)),
                new WaitCommand(SHOT2_FEED_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed()));
    }


    // ==================== THE AUTO ====================

    @Override
    public SequentialCommandGroup buildTasks() {
        m_clock.reset();    // buildTasks() runs right after START, so this is time 0

        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp(2250)),

                // Shot 1 (preload)
                run(moveForward),
                new WaitUntilCommand(() -> m_robot.m_shooterPollen.atSpeed()).withTimeout(2000),
                lap("Shot 1 starts"),
                shootPreload(),
                lap("Shot 1 done"),

                // Garden
                run(pickUpCorner),
                lap("In the corner"),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),

                // Shot 2
                run(getPickUpCorner2),
                lap("At shot 2"),
                shootPollen(),
                lap("Shot 2 done"),

                // Flower
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
                run(intakeFLower),
                lap("At the flower"),
                new WaitCommand(FLOWER_INTAKE_MS),
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheCloud()),   // plow up before driving (delete this line if it should stay down)

                // Shot 3: back to the shot 2 spot
                run(backToShot),
                lap("At shot 3"),
                shootPollen(),
                lap("Shot 3 done"),

                // Park
                run(park),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),
                lap("Done")
        );
    }
}