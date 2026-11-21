package org.firstinspires.ftc.teamcode.autos.autos;


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


@Autonomous(name = "AUTO_Advanced", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_AdvancedSolo extends Robot_Auto {

    private Action moveForward, getFlower1, shootLoad2, ShootLoad2NoTip, getGetFlower2, ShootLoad3, park;

    @Override
    public SequentialCommandGroup buildTasks() {
        return null;
    }

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-12, -63, Math.toRadians(90)));
        GlobalVariables.m_red = false;

        moveForward = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(-61)
                .build();

        getFlower1 = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(-24)
                .lineToX(-57)
                .build();

        shootLoad2 = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(57)
                .lineToX(-33)
                .build();

    }


    }
