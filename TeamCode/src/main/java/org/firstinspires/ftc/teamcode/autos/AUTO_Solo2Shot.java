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
public class AUTO_Solo2Shot extends Robot_Auto {


    private Action moveForward, pickUpCorner, getPickUpCorner2, park;


    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-12, -63, Math.toRadians(90)));
        GlobalVariables.m_red = true;


        // Each path starts where the previous one ended


        // Stop 1: drive up 4" to shoot preload
        moveForward = m_robot.drivetrain.getDrive()
                .actionBuilder(getStartingPose())
                .lineToY(-59)
                .build();


        // Stop 2: curve toward the corner, turning to 45° so the intake (back) points into the corner
        pickUpCorner = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-12, -59, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-60, -56.25, Math.toRadians(45)), Math.toRadians(135))
                //     .lineToX(-63, new TranslationalVelConstraint(20))   // first part: slow, 5 in/s
                .build();


        // Stops 3-5: slide into the corner, turn, wait, then drive up past the hive to stop 5
        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-60, -56.25, Math.toRadians(45)))
                .strafeTo(new Vector2d(-76, -56.25))   // stop 3: slide into the corner, stays at 45°
                .turnTo(Math.toRadians(90))            // turn in place to face 90°
                .waitSeconds(1)                        // pause 1 second
                .setTangent(Math.toRadians(0))         // start moving right, out of the corner
                .splineToLinearHeading(new Pose2d(-50, -36, Math.toRadians(50)), Math.toRadians(90))  // stop 4: now heading up
                .splineToLinearHeading(new Pose2d(-46, 0, Math.toRadians(0)), Math.toRadians(90))    // NEW: pass the hive on the left (more negative X = further from hive)
                .splineToLinearHeading(new Pose2d(-36, 80, Math.toRadians(-50)), Math.toRadians(90))  // stop 5
                .build();


//        getPickUpCorner2 = m_robot.drivetrain.getDrive()
//                .actionBuilder(new Pose2d(-60, -59, Math.toRadians(45)))   // where the robot is now
//                .strafeTo(new Vector2d(-69, -59))                          // 9" in -X, stays at 45°
//                .build();

        // Old path, no longer used: it starts at (-67, -59), so running it after stop 5
        // pulls the robot back down the field.
//        goToOtherSide = m_robot.drivetrain.getDrive()
//                .actionBuilder(new Pose2d(-67, -59, Math.toRadians(0)))
//                .strafeToLinearHeading(new Vector2d(-10, 55), Math.toRadians(90))
//                .build();


        // Park (later): must start at stop 5 = (-36, 80) facing -50°
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-36, 80, Math.toRadians(-50)))
                .strafeToLinearHeading(new Vector2d(-68, 51), Math.toRadians(90))
                .build();
    }


    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp()),


                // Stop 1: drive up and shoot preload
                new InstantCommand(() -> Actions.runBlocking(moveForward)),
                new WaitCommand(1000),
                new CMD_ShootPollen(m_robot),


                // Stops 2-5: intake from the corner, then drive up to stop 5
                new InstantCommand(() -> m_robot.m_intake.kIntakeOn()),
                new InstantCommand(() -> Actions.runBlocking(pickUpCorner)),
                new InstantCommand(() -> Actions.runBlocking(getPickUpCorner2)),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),


                // Stop 5: settle, then shoot
                new WaitCommand(500),
                new CMD_ShootPollen(m_robot),

                // If it still won't shoot, the pollen may need the intake running to feed it
                // into the shooter. Swap the line above for this block to test:
//                new InstantCommand(() -> {
//                    m_robot.m_shooterPollen.setStopperOpen();
//                    m_robot.m_intake.kIntakeOn();
//                }),
//                new WaitCommand(3000),
//                new InstantCommand(() -> {
//                    m_robot.m_intake.kIntakeOff();
//                    m_robot.m_shooterPollen.setStopperClosed();
//                })

//                // Park (later)
                new InstantCommand(() -> Actions.runBlocking(park))
        );
    }
}