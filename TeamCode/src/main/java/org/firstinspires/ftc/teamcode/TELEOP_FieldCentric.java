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
    private boolean m_plowDown = false;   // true while the FlowPlow is down (intake runs the whole time)

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
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        // Shooters spin the whole time. Shooting only opens the gate and runs the intake.
        m_robot.m_shooterPollen.spinUp();
        m_robot.m_shooterNecter.spinUp();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        m_runTime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            // m_robot.run() runs the scheduler, which automatically triggers
            // every subsystem's periodic() (drivetrain localizer + shooter PIDF)
            // Bumpers/buttons only work if the gamepads are read every loop
            m_driverOp.readButtons();
            m_toolOp.readButtons();

            m_robot.run();

            Pose2d poseEstimate = m_robot.drivetrain.getPoseEstimate();

            // RR 1.0 field access syntax (.position.x, .position.y, .heading.toDouble())
            telemetry.addData("ODM", "x[%3.2f] y[%3.2f] heading(%3.2f)",
                    poseEstimate.position.x,
                    poseEstimate.position.y,
                    Math.toDegrees(poseEstimate.heading.toDouble()));

            telemetry.addData("end heading", GlobalVariables.m_autoEndHeading);

            // Per-wheel distance (ticks + inches), power, and velocity
            m_robot.drivetrain.addTelemetry(telemetry);

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading", heading);

            // Intake check: if "Intake power" reads 0.00 when you let go, the code stopped it
            telemetry.addData("Intake trigger (L)", "%.2f", m_driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            telemetry.addData("Intake power", "%.2f", m_robot.m_intake.getPower());
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

        // Field-centric frame of reference: red = +90, blue = -90 (set in getFieldOffsetDegrees)
        m_robot.drivetrain.setPoseEstimate(fieldStartPose());

        // Default drive command for TeleOp controls
        m_robot.drivetrain.setDefaultCommand(
                new RR_MecanumDriveDefault(m_robot.drivetrain, m_driverOp, 0.03)
        );

        // Intake: left trigger = in, right trigger = out (harder press = more power)
        m_robot.m_intake.setDefaultCommand(new CMD_IntakeTriggers(m_robot.m_intake, m_robot.m_shooterNecter, m_driverOp,
                () -> m_plowDown));

        setSide();
        GlobalVariables.m_red = m_robot.getRedSide();

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // Start: re-zero the field-centric direction (uses the alliance offset)
        AddButtonCommand(m_driverOp, GamepadKeys.Button.START,
                new InstantCommand(() -> m_robot.drivetrain.setPoseEstimate(fieldStartPose()))
        );

        // Operator Back: zero the per-wheel distance readout
        AddButtonCommand(m_toolOp, GamepadKeys.Button.BACK,
                new InstantCommand(() -> m_robot.drivetrain.resetDistance())
        );

        // Driver Right Bumper: shoot ONLY Pollen (bottom)
        AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER,
                new InstantCommand(() -> new CMD_ShootPollen(m_robot).schedule())
        );
        // Driver Left Bumper: shoot ONLY Necter (top)
        AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER,
                new InstantCommand(() -> new CMD_ShootNectar(m_robot).schedule())
        );
        // Dpad Up: FlowPlow goes up, the plow intake stops (triggers work as normal)
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP,
                new InstantCommand(() -> {
                    m_plowDown = false;
                    m_robot.m_flowPlow.FlowPlowToTheCloud();
                })
        );

        // Dpad Down: FlowPlow goes down and the intake runs the whole time at kIntakePlow (0.75)
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_DOWN,
                new InstantCommand(() -> {
                    m_plowDown = true;
                    m_robot.m_flowPlow.FlowPlowToTheGround();
                })
        );
    }

    public void setSide() {
        m_robot.setRedSide();
    }

    /** Field-centric offset in degrees. Red = +90. The blue TeleOp overrides this with -90. */
    public double getFieldOffsetDegrees() {
        return 90;
    }

    /** Pose used when the TeleOp starts and when you press Start. */
    private Pose2d fieldStartPose() {
        return new Pose2d(0, 0, Math.toRadians(getFieldOffsetDegrees()));
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