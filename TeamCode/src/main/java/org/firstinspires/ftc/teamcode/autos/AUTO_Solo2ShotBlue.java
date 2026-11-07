package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;


// Blue mirror of AUTO_Solo2ShotRed:
//   x -> -x,  heading/tangent theta -> (180 - theta)
@Autonomous(name = "Blue Solo", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_Solo2ShotBlue extends Robot_Auto {


    private Action moveForward, shoot, pickUpCorner, getPickUpCorner2, park;


    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(12, -63, Math.toRadians(90)));
        GlobalVariables.m_red = false;

        // Each path starts where the previous one ended
        // Stop 1: drive up 2" to shoot preload
        moveForward = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(-61)
                .build();


        // Stop 2: curve toward the corner, turning to 135° so the intake (back) points into the corner
        pickUpCorner = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(60, -56.25, Math.toRadians(135)), Math.toRadians(45))
                .strafeTo(new Vector2d(-76, -56.25))    // stop 3: slide into the corner, stays at 135°
                .turnTo(Math.toRadians(90))            // turn in place to face 90°
                .build();


        // Stops 3-5: wait in the corner, then drive up past the hive to stop 5
        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-76, -56.25, Math.toRadians(90)))
                .waitSeconds(1)                        // pause 1 second
                .setTangent(Math.toRadians(180))       // start moving left, out of the corner
                .splineToLinearHeading(new Pose2d(50, -36, Math.toRadians(130)), Math.toRadians(90))   // stop 4: now heading up
                .splineToLinearHeading(new Pose2d(46, 0, Math.toRadians(180)), Math.toRadians(90))     // pass the hive on the right (more positive X = further from hive)
                .splineToLinearHeading(new Pose2d(20, 83, Math.toRadians(-114)), Math.toRadians(90))   // stop 5
                .build();


        // Park: starts where getPickUpCorner2 ends = stop 5 (20, 83) facing -114°
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(20, 83, Math.toRadians(-114)))
                .strafeToLinearHeading(new Vector2d(37, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(68, 51), Math.toRadians(90))
                .build();
    }


    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp(2250)),

                // Stop 1: drive up and shoot preload
                new InstantCommand(() -> Actions.runBlocking(moveForward)),
                new WaitCommand(2000),
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

                // Park
                new InstantCommand(() -> Actions.runBlocking(park))
        );
    }
}