package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class Robot_Auto extends LinearOpMode {

     public RobotContainer m_robot;

     private Pose2d m_startingPose = new Pose2d(0, 0, 0);

     private final ElapsedTime m_runTime = new ElapsedTime();

     public void initialize() {
          telemetry.clearAll();
          telemetry.addData("init complete", "BaseRobot");
     }

     @Override
     public void runOpMode() throws InterruptedException {
          initializeSubsystems();

          prebuildTasks();

          IMU imu = hardwareMap.get(IMU.class, "imu");

          RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                  RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                  RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
          imu.initialize(new IMU.Parameters(orientationOnRobot));

          // Necter stopper holds its auto position (0.25) for the whole auto
          m_robot.m_shooterNecter.setStopperAuto();

          while (!opModeIsActive() && !isStopRequested()) {
               m_robot.run(); // run the scheduler
               telemetry.addData("ODM", "x[%3.2f] y[%3.2f] heading(%3.2f)",
                       m_robot.drivetrain.getPoseEstimate().position.x,
                       m_robot.drivetrain.getPoseEstimate().position.y,
                       Math.toDegrees(m_robot.drivetrain.getPoseEstimate().heading.toDouble()));
               telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
               telemetry.update();
          }

          waitForStart();

          // Zero the IMU heading right as the auto begins (robot must be still, facing its start direction)
          imu.resetYaw();

          buildTasks().schedule();

          m_runTime.reset();

          while (!isStopRequested() && opModeIsActive()) {
               // m_robot.run() triggers periodic() on all subsystems, updating odometry
               m_robot.run();

               Pose2d poseEstimate = m_robot.drivetrain.getPoseEstimate();

               telemetry.addData("ODM", "x[%3.2f] y[%3.2f] heading(%3.2f)",
                       poseEstimate.position.x,
                       poseEstimate.position.y,
                       Math.toDegrees(poseEstimate.heading.toDouble()));
               double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
               telemetry.addData("Heading", heading);
               telemetry.update();
          }

          // Auto is over: Necter stopper goes back to its normal closed position
          m_robot.m_shooterNecter.setStopperClosed();

          // Save final heading for TeleOp handoff
          GlobalVariables.m_autoEndHeading = m_robot.drivetrain.getPoseEstimate().heading.toDouble();
          m_robot.reset();
     }

     public void initializeSubsystems() {
          // Clear anything left over from a previous (or stopped-early) run
          CommandScheduler.getInstance().reset();
          m_robot = new RobotContainer(this);
     }

     public void setStartingPose(Pose2d p_pose) {
          m_startingPose = p_pose;
          m_robot.drivetrain.setPoseEstimate(m_startingPose);
     }

     public Pose2d getStartingPose() {
          return m_startingPose;
     }

     public abstract SequentialCommandGroup buildTasks();
     public abstract void prebuildTasks();
}