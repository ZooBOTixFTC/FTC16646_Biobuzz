package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;

/** Shoots ONLY Necter (top). Necter keeps spinning afterward, so later shots skip spin-up. */
public class CMD_ShootNecter extends SequentialCommandGroup {

    public static final long SHOOT_MILLIS = 5000;
    public static final long SPINUP_TIMEOUT_MILLIS = 2500;

    private final RobotContainer m_robot;

    public CMD_ShootNecter(RobotContainer p_robot) {
        m_robot = p_robot;

        addCommands(
                new InstantCommand(() -> p_robot.m_shooterNecter.spinUp()),
                new WaitUntilCommand(p_robot.m_shooterNecter::atSpeed)
                        .withTimeout(SPINUP_TIMEOUT_MILLIS),
                new InstantCommand(p_robot.m_intake::kIntakeOn),
                new WaitCommand(SHOOT_MILLIS),
                new InstantCommand(this::stopFeeding)
        );
    }

    private void stopFeeding() {
        m_robot.m_intake.kIntakeOff();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            stopFeeding();
        }
    }
}