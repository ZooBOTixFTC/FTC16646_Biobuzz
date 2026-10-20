package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class RobotContainer {
    public boolean m_red = true;
    public Robot m_robot = new Robot();
    public MecanumDriveSubsystem drivetrain;
    public SUB_Shooter m_shooter;
    public SUB_Intake m_intake;

    public SUB_Transfer m_transfer;
    public GlobalVariables GlobalVariables;

    public RobotContainer(OpMode p_opMode) {
        // Initialize drivetrain subsystem with hardware map and default (0,0,0) pose
        drivetrain = new MecanumDriveSubsystem(p_opMode.hardwareMap, new Pose2d(0, 0, 0));

        // Names must match the motor names in your robot configuration
        m_shooter = new SUB_Shooter(p_opMode.hardwareMap);

        m_intake = new SUB_Intake(p_opMode);

        m_transfer = new SUB_Transfer(p_opMode);
    }

    public void run() {
        m_robot.run();
    }

    public void reset() {
        m_robot.reset();
    }

    public void schedule(Command... commands) {
        m_robot.schedule(commands);
    }

    public void setRedSide() {
        m_red = true;
    }

    public void setBlueSide() {
        m_red = false;
    }

    public boolean getRedSide() {
        return m_red;
    }
}