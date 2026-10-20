package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot(RobotContainer p_robot) {

        addCommands(
                new InstantCommand(p_robot.m_shooter::setStopperClosed)
                ,new WaitCommand(250)
                ,new InstantCommand(()->p_robot.m_shooter.spinUp())
                ,new WaitCommand(250)
                ,new InstantCommand(p_robot.m_shooter::setStopperOpen)
                ,new InstantCommand(p_robot.m_intake::kIntakeOn)
                ,new InstantCommand(p_robot.m_transfer::kTransferOn)
                ,new WaitCommand(750)
                ,new InstantCommand(()->p_robot.m_shooter.start(2000))
                ,new WaitCommand(2000)
                ,new InstantCommand(p_robot.m_shooter::setStopperClosed)
                ,new InstantCommand(p_robot.m_intake::kIntakeOff)
                ,new InstantCommand(p_robot.m_transfer::kTransferOff)
                ,new InstantCommand(()->p_robot.m_shooter.stop())
        );
    }
}
