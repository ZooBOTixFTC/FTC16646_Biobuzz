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

    // Two independent shooters: Pollen = bottom, Necter = top
    public SUB_ShooterPollen m_shooterPollen;          // bottom, has the stopper servo
    public SUB_ShooterNectar m_shooterNecter;    // top

    public SUB_Intake m_intake;

    public GlobalVariables GlobalVariables;

    public RobotContainer(OpMode p_opMode) {
        // Initialize drivetrain subsystem with hardware map and default (0,0,0) pose
        drivetrain = new MecanumDriveSubsystem(p_opMode.hardwareMap, new Pose2d(0, 0, 0));

        // Motor and servo names are set at the top of each shooter file.
        m_shooterPollen = new SUB_ShooterPollen(p_opMode.hardwareMap);
        m_shooterNecter = new SUB_ShooterNectar(p_opMode.hardwareMap);

        m_intake = new SUB_Intake(p_opMode);
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