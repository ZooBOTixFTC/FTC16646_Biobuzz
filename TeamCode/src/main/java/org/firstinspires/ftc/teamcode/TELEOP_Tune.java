package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.roadrunner.DriveParamsStore;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

@TeleOp(name = "Teleop TEST (tuning)", group = "Testing")
public class TELEOP_Tune extends LinearOpMode {

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
            m_robot.m_shooter.addTelemetry(telemetry);

            // Per-wheel distance (ticks + inches), power, and velocity
            m_robot.drivetrain.addTelemetry(telemetry);

            telemetry.update();
        }

        endOfOpMode();
        m_robot.reset();
    }

    public void endOfOpMode() {}

    public void initializeSubsystems() {
        // Send telemetry to both the Driver Station and FTC Dashboard (for graphing)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        // Driver Right Bumper: hold to spin shooter up to TARGET_RPM, release to stop
        AddButtonToggleCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER,
                new InstantCommand(() -> m_robot.m_shooter.spinUp()),
                new InstantCommand(() -> m_robot.m_shooter.stop())
        );

        // Driver D-pad Up/Down: raise/lower shooter kP
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP,
                new InstantCommand(() -> m_robot.m_shooter.adjustP(SUB_Shooter.P_STEP))
        );
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_DOWN,
                new InstantCommand(() -> m_robot.m_shooter.adjustP(-SUB_Shooter.P_STEP))
        );

        // Driver D-pad Right/Left: raise/lower shooter kD
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_RIGHT,
                new InstantCommand(() -> m_robot.m_shooter.adjustD(SUB_Shooter.D_STEP))
        );
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_LEFT,
                new InstantCommand(() -> m_robot.m_shooter.adjustD(-SUB_Shooter.D_STEP))
        );

        // Driver A: save shooter AND drivetrain values to the Control Hub
        AddButtonCommand(m_driverOp, GamepadKeys.Button.A,
                new InstantCommand(() -> {
                    m_robot.m_shooter.saveGains();
                    DriveParamsStore.save();
                })
        );

        // Operator D-pad Up/Down: raise/lower inches per tick by 1% (TESTING ONLY)
        AddButtonCommand(m_toolOp, GamepadKeys.Button.DPAD_UP,
                new InstantCommand(() -> MecanumDrive.PARAMS.inPerTick *= 1.01)
        );
        AddButtonCommand(m_toolOp, GamepadKeys.Button.DPAD_DOWN,
                new InstantCommand(() -> MecanumDrive.PARAMS.inPerTick *= 0.99)
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