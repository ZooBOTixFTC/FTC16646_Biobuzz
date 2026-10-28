package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Pollen and Necter shoot AT THE SAME TIME. Speeds come from Constants.ShooterConstants
 * (shootingRPM for Pollen, shootingNectarRPM for Necter). The shooters keep spinning after a
 * shot, so later shots skip the spin-up wait.
 */
public class CMD_Shoot extends SequentialCommandGroup {

    public static final long SHOOT_MILLIS = 5000;           // how long the intake feeds balls
    public static final long SPINUP_TIMEOUT_MILLIS = 2500;  // give up waiting for speed after this

    private final RobotContainer m_robot;

    public CMD_Shoot(RobotContainer p_robot) {
        m_robot = p_robot;

        addCommands(
                new InstantCommand(() -> {
                    p_robot.m_shooterPollen.spinUp();
                    p_robot.m_shooterNecter.spinUp();
                }),
                new WaitUntilCommand(() -> p_robot.m_shooterPollen.atSpeed()
                        && p_robot.m_shooterNecter.atSpeed())
                        .withTimeout(SPINUP_TIMEOUT_MILLIS),
                new InstantCommand(() -> {
                    p_robot.m_shooterPollen.setStopperOpen();
                    p_robot.m_intake.kIntakeOn();
                }),
                new WaitCommand(SHOOT_MILLIS),
                new InstantCommand(this::stopFeeding)
        );
    }

    private void stopFeeding() {
        m_robot.m_intake.kIntakeOff();
        m_robot.m_shooterPollen.setStopperClosed();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            stopFeeding();
        }
    }
}