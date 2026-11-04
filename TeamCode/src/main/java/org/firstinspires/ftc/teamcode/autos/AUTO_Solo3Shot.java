package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_ShootPollen;


@Autonomous(name = "Red Solo 3 Shot", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_Solo3Shot extends Robot_Auto {

    // Stop 5: where the robot shoots from (end of getPickUpCorner2)
    private static final double SHOOT_X = -36;
    private static final double SHOOT_Y = 80;
    private static final double SHOOT_HEADING = -50;     // degrees

    // Flower: the robot drives to the APPROACH point turned to FLOWER_HEADING,
    // then backs slowly PUSH_DIST inches into the flower (intake first).
    private static final double FLOWER_HEADING = -153;   // degrees: the back (intake) points at the flower
    private static final double APPROACH_X = -28;        // stops short? raise X by 2
    private static final double APPROACH_Y = 84;         //   ...and Y by 1
    private static final double PUSH_DIST = 6;           // inches of slow push into the flower
    private static final double PUSH_SPEED = 15;         // in/s for the push

    // Where the push ends (worked out from the numbers above, don't edit)
    private static final double PUSH_ANGLE = Math.toRadians(FLOWER_HEADING + 180);   // the way the back points
    private static final double FLOWER_X = APPROACH_X + PUSH_DIST * Math.cos(PUSH_ANGLE);
    private static final double FLOWER_Y = APPROACH_Y + PUSH_DIST * Math.sin(PUSH_ANGLE);

    // Passing the hive: tune these two if it still bumps
    private static final double STOP4_X = -62;           // more negative = further left of the hive
    private static final double ARRIVE_ANGLE = 60;       // direction it arrives at stop 5; lower = stays left longer

    // Speed from stop 4 to stop 5 (Road Runner's default is about 50 in/s)
    private static final double FAST_SPEED = 65;         // top speed, inches per second
    private static final double FAST_ACCEL = 70;         // how hard it speeds up, in/s²
    private static final double FAST_DECEL = -45;        // how hard it brakes before stop 5 (must be negative)


    private Action moveForward, pickUpCorner, getPickUpCorner2, flowerIntake, backToShoot, park;


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
                .build();


        // Stops 3-5: slide into the corner, turn, wait, then drive up past the hive to stop 5
        getPickUpCorner2 = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(-60, -56.25, Math.toRadians(45)))
                .strafeTo(new Vector2d(-76, -56.25))   // stop 3: slide into the corner, stays at 45°
                .turnTo(Math.toRadians(90))            // turn in place to face 90°
                .waitSeconds(1)                        // pause 1 second
                .setTangent(Math.toRadians(0))         // start moving right, out of the corner
                .splineToLinearHeading(new Pose2d(STOP4_X, -36, Math.toRadians(50)), Math.toRadians(90))  // stop 4: heading straight up, well left of the hive
                .splineToLinearHeading(new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING)), Math.toRadians(ARRIVE_ANGLE),
                        new TranslationalVelConstraint(FAST_SPEED),
                        new ProfileAccelConstraint(FAST_DECEL, FAST_ACCEL))   // stop 5: fast run up the field
                .build();


        // Stop 5a: turn to -153° on the way to the approach point, then back slowly into the flower
        flowerIntake = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING)))
                .strafeToLinearHeading(new Vector2d(APPROACH_X, APPROACH_Y), Math.toRadians(FLOWER_HEADING))
                .setTangent(PUSH_ANGLE)                                              // move the way the intake faces
                .lineToX(FLOWER_X, new TranslationalVelConstraint(PUSH_SPEED))       // slow push, intake first
                .build();


        // Stop 5b: back to the stop 5 spot to shoot a third time
        backToShoot = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(FLOWER_X, FLOWER_Y, Math.toRadians(FLOWER_HEADING)))
                .strafeToLinearHeading(new Vector2d(SHOOT_X, SHOOT_Y), Math.toRadians(SHOOT_HEADING))
                .build();


        // Park: starts at stop 5 (the robot is back there after backToShoot)
        park = m_robot.drivetrain.getDrive()
                .actionBuilder(new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING)))
                .strafeToLinearHeading(new Vector2d(-68, 51), Math.toRadians(90))
                .build();
    }


    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                // Shooter spins from the start of the match, not during init
                new InstantCommand(() -> m_robot.m_shooterPollen.spinUp()),


                // Shot 1, stop 1: drive up and shoot preload
                new InstantCommand(() -> Actions.runBlocking(moveForward)),
                new WaitCommand(1000),
                new CMD_ShootPollen(m_robot),


                // Stops 2-5: intake from the corner, then drive up to stop 5
                new InstantCommand(() -> m_robot.m_intake.kIntakeOn()),
                new InstantCommand(() -> Actions.runBlocking(pickUpCorner)),
                new InstantCommand(() -> Actions.runBlocking(getPickUpCorner2)),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),


                // Shot 2, stop 5: settle, then shoot
                new WaitCommand(500),
                new CMD_ShootPollen(m_robot),


                // Stop 5a: intake from the flower (intake runs during the push and the wait)
                new InstantCommand(() -> m_robot.m_intake.kIntakeOn()),
                new InstantCommand(() -> Actions.runBlocking(flowerIntake)),
                new WaitCommand(1500),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff()),


                // Shot 3, stop 5b: back to stop 5, settle, shoot
                new InstantCommand(() -> Actions.runBlocking(backToShoot)),
                new WaitCommand(500),
                new CMD_ShootPollen(m_robot),


                // Park
                new InstantCommand(() -> Actions.runBlocking(park))
        );
    }
}