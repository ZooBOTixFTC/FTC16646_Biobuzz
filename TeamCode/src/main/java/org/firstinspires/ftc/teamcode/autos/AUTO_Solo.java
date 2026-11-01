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
import org.firstinspires.ftc.teamcode.commands.CMD_ShootPollen;

@Autonomous(name = "Red Solo", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_Solo extends Robot_Auto {
    private Action moveForward, pickUpCorner, goToOtherSide, goToFlower, park;

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-63, 12, Math.toRadians(0)));
        GlobalVariables.m_red = true;

        moveForward = m_robot.drivetrain.getDrive().actionBuilder(getStartingPose())
                .lineToX(-59)
                .build();

        pickUpCorner = m_robot.drivetrain.getDrive().actionBuilder(new Pose2d(-59, 12,0))
                .setTangent(Math.PI/2)
                .lineToYLinearHeading(67, Math.toRadians(-87))
                .build();

        goToOtherSide = m_robot.drivetrain.getDrive().actionBuilder(new Pose2d(-63, 67, Math.toRadians(86.5)))
                .strafeToLinearHeading(new Vector2d(79.5, 36), Math.toRadians(200))
                .build();

        park = m_robot.drivetrain.getDrive().actionBuilder(new Pose2d(79.5, 9, Math.toRadians(185)))
                .strafeToLinearHeading(new Vector2d(60, 63), Math.toRadians(0))
                .build();

        m_robot.m_shooterPollen.spinUp();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                new InstantCommand(()-> Actions.runBlocking(moveForward)),
                new WaitCommand(1000),
                new CMD_ShootPollen(m_robot),
                new InstantCommand(()->m_robot.m_intake.kIntakeOn()),
                new InstantCommand(()->Actions.runBlocking(pickUpCorner)),
                new WaitCommand(1000),
                new InstantCommand(()->m_robot.m_intake.kIntakeOff()),
                new InstantCommand(()->Actions.runBlocking(goToOtherSide)),
                new InstantCommand(()->m_robot.m_shooterPollen.spinUp()),
                new WaitCommand(1000),
                new CMD_ShootPollen(m_robot),
                new InstantCommand(()->Actions.runBlocking(park))
        );
    }
}
