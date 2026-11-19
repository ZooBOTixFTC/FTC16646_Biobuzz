package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;


@Autonomous(name = "Red Solo", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_Solo2ShotRed extends Robot_Auto {


    private Action moveForward, shoot, pickUpCorner, getPickUpCorner2, intakeFLower, park;


    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-12, -63, Math.toRadians(90)));
        GlobalVariables.m_red = true;

        // Each path starts where the previous one ended
        // Stop 1: drive up 4" to shoot preload
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


        // Stops 3-5: slide into the corner, turn, wait, then drive up past the hive to stop 5
        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-81, -56.25, Math.toRadians(90)))
                .waitSeconds(1)                        // pause 1 second
                .setTangent(Math.toRadians(90))         // start moving right, out of the corner
                .splineToSplineHeading(new Pose2d(-60, -20, Math.toRadians(50)), Math.toRadians(90))  // stop 4: now heading up
                .splineToSplineHeading(new Pose2d(-60, 40, Math.toRadians(0)), Math.toRadians(90))    // NEW: pass the hive on the left (more negative X = further from hive)
                .splineToSplineHeading(new Pose2d(-42.5, 75, Math.toRadians(-80)), Math.toRadians(90))  // stop 5
                .build();
//Back up to intake from the flower and shoot
        intakeFLower = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-42.5, 75, Math.toRadians(90)))
                .turnTo(Math.toRadians(-90))
                .lineToY(78)
                .build();

        // Park (later): must start at stop 5 = (-36, 80) facing -50°
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-55, 85, Math.toRadians(-66)))
//                .strafeToLinearHeading(new Vector2d(-37, 30), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(-60, 54), Math.toRadians(90))
                .build();
    }


    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp(2250)),

                // Stop 1: drive up and shoot preload
                new InstantCommand(() -> Actions.runBlocking(moveForward)),
                new WaitUntilCommand(()-> m_robot.m_shooterPollen.atSpeed()).withTimeout(2000),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(250),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.1)),
                new WaitCommand(250),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.2)),
                new WaitCommand(250),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.3)),
                new WaitCommand(250),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.4)),
                new WaitCommand(250),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.5)),
                new WaitCommand(250),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.8)),
                new WaitCommand(700),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed()),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.8)),

                new InstantCommand(() -> Actions.runBlocking(pickUpCorner)),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),
                new InstantCommand(() -> Actions.runBlocking(getPickUpCorner2)),


                // Stop 5: settle, then shoot.
                // Gate opens first; the intake only turns on 0.4 s into the shot.
                new WaitCommand(400),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperOpen()),
                new WaitCommand(520),
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.5)),
                new WaitCommand(3500),
                new InstantCommand(() -> m_robot.m_shooterPollen.setStopperClosed()),

                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
                new InstantCommand(() -> Actions.runBlocking(intakeFLower)),
                // Park (later)
                new InstantCommand(() -> Actions.runBlocking(park))
        );
    }
}