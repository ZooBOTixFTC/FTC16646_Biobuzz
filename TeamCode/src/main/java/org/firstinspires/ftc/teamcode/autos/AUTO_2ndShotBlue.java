package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;


// Blue mirror of AUTO_2ndShotRed: every X is negated (x -> -x).
// Headings mirror as (180 - heading), so 90 deg stays 90 deg.
@Autonomous(name = "Blue-Shoot Second", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_2ndShotBlue extends Robot_Auto {


    private Action moveForward, align2flower, plowIntoFlower, park;

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-12, -63, Math.toRadians(90)));
        GlobalVariables.m_red = false;

        // Each path starts where the previous one ended
        // Stop 1: drive up 4" to shoot preload
        moveForward = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(-61)
                .build();

//        align2flower = m_robot.drivetrain.getDrive()
//                .actionBuilder(new Pose2d(12, -61, Math.toRadians(90)))
//                .lineToY(-49)                     // forward 12"
//                .strafeTo(new Vector2d(28, -49))  // right 12"
//                .build();
//
//        plowIntoFlower = m_robot.drivetrain.getDrive()
//                .actionBuilder(new Pose2d(28, -49, Math.toRadians(90)))
//                .lineToY(-60, new TranslationalVelConstraint(5))
//                .waitSeconds(1.0)
//                .lineToY(-59, new TranslationalVelConstraint(3))
//                .build();


        // Park (later): must start at stop 5 = (-36, 80) facing -50°
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                .lineToY(-50)                    // forward 11" (still facing 90°)
                .turnTo(Math.toRadians(0))       // turn right to face +X
                .setTangent(Math.toRadians(0))   // travel direction = +X
                .lineToX(70)                     // drive forward to X = 80
                .build();

    }


    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new WaitCommand(15000),
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
                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.0)),
//                new InstantCommand(() -> Actions.runBlocking(align2flower)),
//                new InstantCommand(() -> m_robot.m_flowPlow.FlowPlowToTheGround()),
//                new InstantCommand(() -> Actions.runBlocking(plowIntoFlower)),
//                new InstantCommand(() -> m_robot.m_intake.setMotorPower(0.6)),
//                new WaitCommand(2000),
//
//                new WaitCommand(10000),
                //park
                new InstantCommand(() -> Actions.runBlocking(park))

        );
    }
}