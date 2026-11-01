package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/**
 * Default teleop drive command: reads the driver's sticks every loop.
 * Left stick = move, right stick X = turn.
 */
public class RR_MecanumDriveDefault extends CommandBase {

    // Flip any of these to -1 / 1 if that control feels backwards.
    private static final double FORWARD_SIGN = 1;
    private static final double STRAFE_SIGN = 1;
    private static final double TURN_SIGN = 1;

    private final MecanumDriveSubsystem drivetrain;
    private final GamepadEx gamepad;
    private final double deadband;

    public RR_MecanumDriveDefault(MecanumDriveSubsystem drivetrain, GamepadEx gamepad, double deadband) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;
        this.deadband = deadband;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // FTCLib GamepadEx: getLeftY() is already positive when pushed UP
        double forward = FORWARD_SIGN * applyDeadband(gamepad.getLeftY());
        double strafe = STRAFE_SIGN * applyDeadband(gamepad.getLeftX());
        double turn = TURN_SIGN * applyDeadband(gamepad.getRightX());

        drivetrain.drive(forward, strafe, turn);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < deadband ? 0 : value;
    }
}