package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Shoots Pollen (bottom). The shooter is already spinning, so this just opens the gate and
 * runs the intake, then closes the gate again. Timings are in Constants.ShooterConstants.
 */
public class CMD_ShootPollen extends SequentialCommandGroup {

    private final RobotContainer m_robot;

    public CMD_ShootPollen(RobotContainer p_robot) {
        m_robot = p_robot;
        addRequirements(p_robot.m_intake);

        addCommands(
                // Safety: instant if already at speed, otherwise waits (up to the delay)
                new WaitUntilCommand(p_robot.m_shooterPollen::atSpeed)
                        .withTimeout((long) (Constants.ShooterConstants.POLLEN_SERVO_DELAY_SEC * 1000)),
                new InstantCommand(() -> {
                    p_robot.m_shooterPollen.setStopperOpen();
                    p_robot.m_intake.kIntakeOn();
                }),
                new WaitCommand((long) (Constants.ShooterConstants.POLLEN_SERVO_OPEN_SEC * 1000)),
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