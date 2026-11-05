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


// Blue version of AUTO_Solo2Shot ("Red Solo"), turned 180° around the field center.
// Every point (x, y) became (-x, -y), and every angle got +180°.
// Starts on the SCORING-side wall, blue half.
@Autonomous(name = "Blue Solo", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_Solo2ShotBlue extends Robot_Auto {


    private Action moveForward, pickUpCorner, getPickUpCorner2, park;


    @Override
    public void prebuildTasks() {
        // red: (-12, -63) facing 90°
        setStartingPose(new Pose2d(12, 63, Math.toRadians(270)));
        GlobalVariables.m_red = false;


        // Each path starts where the previous one ended


        // Stop 1: drive 4" (down the field) to shoot preload
        moveForward = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(59)                                   // red: -59
                .build();


        // Stop 2: curve toward the corner, turning so the intake (back) points into the corner
        pickUpCorner = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(12, 59, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(60, 56.25, Math.toRadians(225)), Math.toRadians(315))
                .build();


        // Stops 3-5: slide into the corner, turn, wait, then drive past the hive to stop 5
        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(60, 56.25, Math.toRadians(225)))
                .strafeTo(new Vector2d(76, 56.25))     // stop 3: slide into the corner, stays at 225°
                .turnTo(Math.toRadians(270))           // turn in place to face 270° (red: 90°)
                .waitSeconds(1)                        // pause 1 second
                .setTangent(Math.toRadians(180))       // start moving out of the corner (red: 0°)
                .splineToLinearHeading(new Pose2d(50, 36, Math.toRadians(230)), Math.toRadians(270))  // stop 4
                .splineToLinearHeading(new Pose2d(46, 0, Math.toRadians(180)), Math.toRadians(270))   // pass the hive
                .splineToLinearHeading(new Pose2d(36, -80, Math.toRadians(130)), Math.toRadians(270)) // stop 5
                .build();


        // Park: must start at stop 5 = (36, -80) facing 130°
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(36, -80, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(68, -51), Math.toRadians(270))
                .build();
    }


    @Override
    public SequentialCommandGroup buildTasks() {
        // Exactly the same steps as the red auto
        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp()),


                // Stop 1: drive and shoot preload
                new InstantCommand(() -> Actions.runBlocking(moveForward)),
                new WaitCommand(1000),
                new CMD_ShootPollen(m_robot),


                // Stops 2-5: intake from the corner, then drive to stop 5
                new InstantCommand(() -> m_robot.m_intake.kIntakeOn()),
                new InstantCommand(() -> Actions.runBlocking(pickUpCorner)),
                new InstantCommand(() -> Actions.runBlocking(getPickUpCorner2)),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),


                // Stop 5: settle, then shoot
                new WaitCommand(500),
                new CMD_ShootPollen(m_robot),


                // Park
                new InstantCommand(() -> Actions.runBlocking(park))
        );
    }
}