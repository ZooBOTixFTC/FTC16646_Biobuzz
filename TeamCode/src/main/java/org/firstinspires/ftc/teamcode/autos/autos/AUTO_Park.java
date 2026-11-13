package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Robot_Auto;


@Autonomous(name = "AUTO_Park", group = "Autonomous")
public class AUTO_Park extends Robot_Auto {


    private Action park;


    @Override
    public void prebuildTasks() {
        // Start pose
        setStartingPose(new Pose2d(-62.3426, 59.5697, Math.toRadians(-90)));

        // Short move to park
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .splineTo(new Vector2d(-62.3904, 53.498), Math.toRadians(-87.3974))
                .build();
    }


    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> Actions.runBlocking(park))
        );
    }
}