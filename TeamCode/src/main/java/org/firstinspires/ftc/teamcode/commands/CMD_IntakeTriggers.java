package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;

/**
 * Intake default command. The harder you press, the more power it gets.
 *   Left trigger  = intake in   (up to IntakeConstants.kIntakeOn)
 *   Right trigger = intake out  (up to IntakeConstants.kIntakeReverse)
 * It pauses while a shoot command is running, then comes back by itself.
 */
public class CMD_IntakeTriggers extends CommandBase {

    private static final double DEADBAND = 0.05;

    private final SUB_Intake intake;
    private final GamepadEx gamepad;

    public CMD_IntakeTriggers(SUB_Intake intake, GamepadEx gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        double in = Math.pow(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),3);
        double out = Math.pow(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),3);

        double power = in * Constants.IntakeConstants.kIntakeOn
                - out * Math.abs(Constants.IntakeConstants.kIntakeReverse);

        intake.setMotorPower(Math.abs(power) < DEADBAND ? 0 : power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.kIntakeOff();
    }
}