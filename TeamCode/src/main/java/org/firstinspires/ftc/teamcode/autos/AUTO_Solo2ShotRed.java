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


@Config
@Autonomous(name = "Red Solo", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_Solo2ShotRed extends Robot_Auto {
    public static double SHOT2_X = -42.5;
    public static double SHOT2_Y = 75;
    public static double SHOT2_HEADING = -80;
    public static double X_PASS = -72;
    public static double FLOWER_Y = 82;
    public static double PARK_X = -80;
    public static double PARK_Y = 48;

    public static int SHOT2_FEED_MS = 1900;
    public static double CORNAR_PAUSE_SEC = 0.5;
    public static int RAMP_STEP_MS = 150;
    public static int SHOT2_SETTLE_MS = 200;
    public static int FLOWER_INTAKE_MS = 500;



    private Action moveForward, pickUpCorner, getPickUpCorner2, intakeFLower, backToShot, park;
    private final ElapsedTime m_clock = new ElapsedTime();
    private Pose2d shot2Spot() {
        return new Pose2d(SHOT2_X, SHOT2_Y, Math.toRadians(SHOT2_HEADING));
    }
    private Pose2d flowerSpot() {
        return new Pose2d(SHOT2_X, FLOWER_Y, Math.toRadians(-90));
    }
    private Action flowerPath() {
        return m_robot.drivetrain.getDrive()
                .actionBuilder(shot2Spot())
                .turnTo(Math.toRadians(-90))
                .lineToY(FLOWER_Y)
                .build();
    }
    private Action backToShot2Path() {
        return m_robot.drivetrain.getDrive()
                .actionBuilder(flowerSpot())
                .lineToY(SHOT2_Y)
                .turnTo(Math.toRadians(SHOT2_HEADING))
                .build();
    }
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

        moveForward = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(-61)
                .build();

        pickUpCorner = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-12, -61, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-60, -56.25, Math.toRadians(45)), Math.toRadians(135))
                .strafeTo(new Vector2d(-81, -56.25))
                .turnTo(Math.toRadians(90))
                .build();

        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-81, -56.25, Math.toRadians(90)))
                .waitSeconds(CORNAR_PAUSE_SEC)
                .splineToSplineHeading(new Pose2d(X_PASS, -20, Math.toRadians(50)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(X_PASS, 40, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(shot2Spot(), Math.toRadians(90))
                .build();

        intakeFLower = flowerPath();
        backToShot   = backToShot2Path();
        park         = parkPath();
    }
    private InstantCommand lap(String what) {
        return new InstantCommand(() -> telemetry.log().add("%s: %.1f s", what, m_clock.seconds()));
    }
    private InstantCommand run(Action path) {
        return new InstantCommand(() -> Actions.runBlocking(path));
    }
    private SequentialCommandGroup preloadShoot() {
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
        return new SequentialCommandGroup(
                new WaitCommand(SHOT2_SETTLE_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(520),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.55)),
                new WaitCommand(SHOT2_FEED_MS),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed()));
    }
    @Override
    public SequentialCommandGroup buildTasks() {
        m_clock.reset();

        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp(2250)),

                // shoot preload
                run(moveForward),
                new WaitUntilCommand(() -> m_robot.m_shooterPollen.atSpeed()).withTimeout(2250),
                lap("Start shot 1"),
                preloadShoot(),
                lap("End of shot 1"),

                run(pickUpCorner),
                lap("In corner"),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),

                run(getPickUpCorner2),
                lap("Start of shot 2"),
                shootPollen(),
                lap("End of Shot 2"),

                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
                run(intakeFLower),
                lap("At flower"),
                new WaitCommand(FLOWER_INTAKE_MS),
                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheCloud()),

                run(backToShot),
                lap("Start of shot 3"),
                shootPollen(),
                lap("End of shot 3"),

                run(park),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),
                lap("End")
        );
    }
}