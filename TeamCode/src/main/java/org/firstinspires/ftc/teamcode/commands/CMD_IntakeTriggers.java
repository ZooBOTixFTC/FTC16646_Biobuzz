package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterNectar;

import java.util.function.BooleanSupplier;

/**
 * Intake default command. The harder you press, the more power it gets.
 *   Left trigger  = intake in   (up to IntakeConstants.kIntakeOn)
 *   Right trigger = intake out  (up to IntakeConstants.kIntakeReverse)
 * While the FlowPlow is down the intake runs the whole time at IntakeConstants.kIntakePlow
 * (the right trigger can still reverse it to clear a jam).
 * While the intake runs forward, the Necter stopper sits a little lower
 * (Constants.ShooterConstants.NECTAR_SERVO_INTAKE_OFFSET) and goes back when you let go.
 * It pauses while a shoot command is running, then comes back by itself.
 */
public class CMD_IntakeTriggers extends CommandBase {

    private static final double DEADBAND = 0.02;

    private final SUB_Intake intake;
    private final SUB_ShooterNectar nectar;
    private final GamepadEx gamepad;
    private final BooleanSupplier plowDown;
    private boolean stopperLowered = false;

    public CMD_IntakeTriggers(SUB_Intake intake, SUB_ShooterNectar nectar, GamepadEx gamepad,
                              BooleanSupplier plowDown) {
        this.intake = intake;
        this.nectar = nectar;
        this.gamepad = gamepad;
        this.plowDown = plowDown;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        double in = Math.pow(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),3);
        double out = Math.pow(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),3);

        double power = in * Constants.IntakeConstants.kIntakeOn
                - out * Math.abs(Constants.IntakeConstants.kIntakeReverse);

        double applied = Math.abs(power) < DEADBAND ? 0 : power;

        // FlowPlow down: intake stays on at plow speed (reverse trigger still wins)
        if (plowDown.getAsBoolean() && applied >= 0) {
            applied = Constants.IntakeConstants.kIntakePlow;
        }

        intake.setMotorPower(applied);

        // Lower the Necter stopper only while intaking forward
        boolean intaking = applied > 0;
        if (intaking != stopperLowered) {
            if (intaking) {
                nectar.setStopperIntake();
            } else {
                nectar.setStopperClosed();
            }
            stopperLowered = intaking;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.kIntakeOff();
        if (stopperLowered) {
            nectar.setStopperClosed();
            stopperLowered = false;
        }
    }
}