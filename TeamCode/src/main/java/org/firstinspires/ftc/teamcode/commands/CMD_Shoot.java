package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Shoots BOTH. The shooters are already spinning, so this just opens both gates and runs the
 * intake, then closes the gates again. Timings are in Constants.ShooterConstants.
 */
public class CMD_Shoot extends SequentialCommandGroup {

    private final RobotContainer m_robot;

    public CMD_Shoot(RobotContainer p_robot) {
        m_robot = p_robot;
        addRequirements(p_robot.m_intake);

        double delaySec = Math.max(Constants.ShooterConstants.POLLEN_SERVO_DELAY_SEC,
                Constants.ShooterConstants.NECTAR_SERVO_DELAY_SEC);
        double openSec = Math.max(Constants.ShooterConstants.POLLEN_SERVO_OPEN_SEC,
                Constants.ShooterConstants.NECTAR_SERVO_OPEN_SEC);

        addCommands(
                // Safety: instant if both are already at speed, otherwise waits (up to the delay)
                new WaitUntilCommand(() -> p_robot.m_shooterPollen.atSpeed()
                        && p_robot.m_shooterNecter.atSpeed())
                        .withTimeout((long) (delaySec * 1000)),
                new InstantCommand(() -> {
                    p_robot.m_shooterPollen.setStopperOpen();
                    p_robot.m_shooterNecter.setStopperOpen();
                    p_robot.m_intake.kIntakeOn();
                }),
                new WaitCommand((long) (openSec * 1000)),
                new InstantCommand(this::stopFeeding)
        );
    }

    private void stopFeeding() {
        m_robot.m_intake.kIntakeOff();
        m_robot.m_shooterPollen.setStopperClosed();
        m_robot.m_shooterNecter.setStopperClosed();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            stopFeeding();
        }
    }
}