package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.*;

@TeleOp(name = "Teleop Red", group = "Teleop Red")
public class TELEOP_FieldCentric extends LinearOpMode {

    public RobotContainer m_robot;
    private GamepadEx m_driverOp;
    private GamepadEx m_toolOp;

    private static final ElapsedTime m_runTime = new ElapsedTime();

    public void initialize() {
        telemetry.clearAll();
        telemetry.addData("init complete", true);

        m_runTime.reset();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeSubsystems();
        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        m_runTime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            // m_robot.run() runs the scheduler, which automatically triggers
            // every subsystem's periodic() (drivetrain localizer + shooter PIDF)
            m_robot.run();

            Pose2d poseEstimate = m_robot.drivetrain.getPoseEstimate();

            // RR 1.0 field access syntax (.position.x, .position.y, .heading.toDouble())
            telemetry.addData("ODM", "x[%3.2f] y[%3.2f] heading(%3.2f)",
                    poseEstimate.position.x,
                    poseEstimate.position.y,
                    Math.toDegrees(poseEstimate.heading.toDouble()));

            telemetry.addData("end heading", GlobalVariables.m_autoEndHeading);

            // Shooter PIDF readout (Driver Station + FTC Dashboard)
            m_robot.m_shooterPollen.addTelemetry(telemetry);
            m_robot.m_shooterNecter.addTelemetry(telemetry);

            // Per-wheel distance (ticks + inches), power, and velocity
            m_robot.drivetrain.addTelemetry(telemetry);

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        endOfOpMode();
        m_robot.reset();
    }

    public void endOfOpMode() {}

    public void initializeSubsystems() {
        // Send telemetry to both the Driver Station and FTC Dashboard (for graphing)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Clear anything left over from a previous (or crashed) run
        com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();

        m_robot = new RobotContainer(this);
        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);

        m_robot.drivetrain.setFieldCentric(true);

        // Default drive command for TeleOp controls
        m_robot.drivetrain.setDefaultCommand(
                new RR_MecanumDriveDefault(m_robot.drivetrain, m_driverOp, 0.03)
        );

        setSide();
        GlobalVariables.m_red = m_robot.getRedSide();

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // Operator Start: Reset field-centric pose to (0,0,0)
        AddButtonCommand(m_toolOp, GamepadKeys.Button.START,
                new InstantCommand(() -> m_robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0)))
        );

        // Operator Back: zero the per-wheel distance readout
        AddButtonCommand(m_toolOp, GamepadKeys.Button.BACK,
                new InstantCommand(() -> m_robot.drivetrain.resetDistance())
        );

        // Driver Right Trigger: Pollen (bottom) fires and finishes, then Necter (top) fires and finishes
        AddTriggerCommand(m_driverOp, GamepadKeys.Trigger.RIGHT_TRIGGER,
                new InstantCommand(() -> new CMD_Shoot(m_robot).schedule())
        );

        // Driver Left Trigger: hold to run the intake, release to stop
        AddTriggerToggleCommand(m_driverOp, GamepadKeys.Trigger.LEFT_TRIGGER,
                new InstantCommand(() -> m_robot.m_intake.kIntakeOn()),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff())
        );

        // Driver X: hold to run intake in reverse, release to stop
        AddButtonToggleCommand(m_driverOp, GamepadKeys.Button.X,
                new InstantCommand(() -> m_robot.m_intake.kIntakeReverse()),
                new InstantCommand(() -> m_robot.m_intake.kIntakeOff())
        );

        // Operator Left Trigger: shoot ONLY Pollen (bottom)
        AddTriggerCommand(m_toolOp, GamepadKeys.Trigger.LEFT_TRIGGER,
                new InstantCommand(() -> new CMD_shootPollen(m_robot).schedule())
        );

        // Operator Right Trigger: shoot ONLY Necter (top)
        AddTriggerCommand(m_toolOp, GamepadKeys.Trigger.RIGHT_TRIGGER,
                new InstantCommand(() -> new CMD_ShootNecter(m_robot).schedule())
        );
    }

    public void setSide() {
        m_robot.setRedSide();
    }

    public void AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
        new GamepadButton(gamepad, button).whenPressed(command);
    }

    public void AddButtonToggleCommand(GamepadEx gamepad, GamepadKeys.Button button, Command onTrue, Command onFalse) {
        new GamepadButton(gamepad, button).whenPressed(onTrue).whenReleased(onFalse);
    }

    public void AddTriggerCommand(GamepadEx gamepad, GamepadKeys.Trigger trigger, Command command) {
        new Trigger(() -> gamepad.getTrigger(trigger) >= .5).whenActive(command);
    }

    public void AddTriggerToggleCommand(GamepadEx gamepad, GamepadKeys.Trigger trigger, Command onTrue, Command onFalse) {
        new Trigger(() -> gamepad.getTrigger(trigger) >= .5).whenActive(onTrue).whenInactive(onFalse);
    }
}