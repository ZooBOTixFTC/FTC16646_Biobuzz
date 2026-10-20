package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;

@Autonomous(name = "Far Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class  AUTO_FarBlue9Ball extends Robot_Auto {

    private Action moveAwayAction;
    private Action intakeFirstSpikeAction;
    private Action intakeCornerAction;
    private Action parkAction;

    @Override
    public void prebuildTasks() {
        Pose2d startPose = new Pose2d(-64.75, 15.5, Math.toRadians(0));
        setStartingPose(startPose);

        GlobalVariables.m_red = false;

        // 1. Move away initial trajectory
        moveAwayAction = m_robot.drivetrain.getDrive().actionBuilder(getStartingPose())
                .lineToXLinearHeading(-58, Math.toRadians(17.5))
                .build();

        // 2. First spike mark intake route (Pose at end of moveAwayAction is approx (-58, 15, 17.5 deg))
        Pose2d poseAfterMoveAway = new Pose2d(-58, 15, Math.toRadians(17.5));
        intakeFirstSpikeAction = m_robot.drivetrain.getDrive().actionBuilder(poseAfterMoveAway)
                .strafeToLinearHeading(new Vector2d(-32, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-36, 62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-60, 15), Math.toRadians(17.5))
                .build();

        // 3. Corner intake route
        Pose2d poseAfterSpike = new Pose2d(-60, 15, Math.toRadians(17.5));
        intakeCornerAction = m_robot.drivetrain.getDrive().actionBuilder(poseAfterSpike)
                .strafeToLinearHeading(new Vector2d(-64, 62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-60, 15), Math.toRadians(17.5))
                .build();

        // 4. Parking / Final movement to corner
        parkAction = m_robot.drivetrain.getDrive().actionBuilder(poseAfterSpike)
                .strafeToLinearHeading(new Vector2d(-64, 62), Math.toRadians(90))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                new ActionCommand(moveAwayAction, m_robot.drivetrain),
                new ActionCommand(intakeFirstSpikeAction, m_robot.drivetrain),
                new ActionCommand(intakeCornerAction, m_robot.drivetrain),
                new ActionCommand(parkAction, m_robot.drivetrain)
        );
    }
}