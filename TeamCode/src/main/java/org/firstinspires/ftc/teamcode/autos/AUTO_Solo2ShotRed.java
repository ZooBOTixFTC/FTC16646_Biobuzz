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
    // Stop 5: shot 2, in front of the flower (code numbers, not real field inches)
    public static double SHOT2_X = -45.5;
    public static double SHOT2_Y = 75;

    public static double PARK_X = -95;

    public static double PARK_Y = 47;
    public static double HIVE_PASS_X = -72;

    public static double SHOT2_HEADING = -80;    // degrees
    // Flower: how far it backs up into the flower (a y number)
    public static double FLOWER_Y = 84;
    // ---- Time savers: if one causes problems, put back the old value shown ----
    public static int SHOT2_FEED_MS = 2000;      // how long shot 2 feeds          (was 3500)
    public static double CORNER_PAUSE_SEC = 0.5; // pause in the garden corner     (was 1)
    public static int RAMP_STEP_MS = 250;        // shot 1 intake ramp, each step  (was 250)
    public static int SHOT2_SETTLE_MS = 400;     // settle before shot 2           (was 400)


    private Action moveForward, shoot, pickUpCorner, getPickUpCorner2, intakeFLower, backToShot, park;

    // Lap times: shown in the Driver Station log, so you can see where the time goes
    private final ElapsedTime m_clock = new ElapsedTime();


    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-12, -63, Math.toRadians(90)));
        GlobalVariables.m_red = true;

        // Each path starts where the previous one ended
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


        // Stops 3-5: wait in the corner, then drive up past the hive to stop 5
        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-81, -56.25, Math.toRadians(90)))
                .waitSeconds(CORNER_PAUSE_SEC)         // let the garden pollen in
                .setTangent(Math.toRadians(90))         // start moving out of the corner
                .splineToSplineHeading(new Pose2d(HIVE_PASS_X, -20, Math.toRadians(50)), Math.toRadians(90))  // stop 4: now heading up
                .splineToSplineHeading(new Pose2d(HIVE_PASS_X, 40, Math.toRadians(0)), Math.toRadians(90))    // pass the hive on the left (more negative X = further from hive)
                .splineToSplineHeading(shot2Spot(), Math.toRadians(90))  // stop 5
                .build();

        // Back up to intake from the flower.
        // Starts exactly where stop 5 ends (same numbers), so the robot doesn't spin first (Task 6.2).
        intakeFLower = m_robot.drivetrain.getDrive()
                .actionBuilder(shot2Spot())
                .turnTo(Math.toRadians(-90))
                .lineToY(FLOWER_Y)
                .build();

        backToShot = m_robot.drivetrain.getDrive()
                .actionBuilder(flowerSideFlowerSpot())
                .lineToY(SHOT2_Y)
                .turnTo(Math.toRadians(SHOT2_HEADING))
                .build();

        // Park (later): must start where the flower path ends = (SHOT2_X, FLOWER_Y) facing -90°
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(SHOT2_X, SHOT2_Y, Math.toRadians(SHOT2_HEADING)))
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .build();
    }

    /** Writes "what: 12.3 s" to the Driver Station log (it stays on screen). */
    private InstantCommand lap(String what) {
        return new InstantCommand(() -> telemetry.log().add("%s: %.1f s", what, m_clock.seconds()));
    }
    private InstantCommand run(Action path) {
        return new InstantCommand(() -> Actions.runBlocking(path));
    }
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


    private SequentialCommandGroup shootPollen() {
        return new SequentialCommandGroup(new WaitCommand(SHOT2_SETTLE_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(520),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.5)),
                new WaitCommand(SHOT2_FEED_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed())
        );
    }

    private Pose2d shot2Spot() {
        return new Pose2d(SHOT2_X, SHOT2_Y, Math.toRadians(SHOT2_HEADING));
    }
    private Pose2d flowerSideFlowerSpot() {
        return new Pose2d(SHOT2_X, FLOWER_Y, Math.toRadians(-90));
    }
    @Override
    public SequentialCommandGroup buildTasks() {
        m_clock.reset();    // buildTasks() runs right after START, so this is time 0

        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp(2250)),

                // Stop 1: drive up and shoot preload
                run(moveForward),
                new WaitUntilCommand(()-> m_robot.m_shooterPollen.atSpeed()).withTimeout(2000),
                lap("Shot 1 starts"),
                shootPreload(),
                lap("Shot 1 done"),


                run(pickUpCorner),
                lap("In the corner"),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),
                run(getPickUpCorner2),
                lap("At shot 2"),
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
                shootPollen(),
                lap("Shot 2 done"),

                run(intakeFLower),
                lap("At the flower"),
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowFlopsAround()),
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
                new WaitCommand(1250),
                run(backToShot),
                lap("Shot started"),
                shootPollen(),
                lap("Shot 3 done"),

                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheCloud()),
                new InstantCommand(() -> Actions.runBlocking(park)),
                lap("Done")
        );
    }
}