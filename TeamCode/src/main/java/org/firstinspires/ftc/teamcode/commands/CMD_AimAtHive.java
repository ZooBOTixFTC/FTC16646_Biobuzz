package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/**
 * Aims at our alliance's hive cluster by overriding ONLY the driver's turn input. The driver keeps
 * full control of forward/strafe the whole time. Opposing-alliance clusters are never used.
 *
 * Does not require the drivetrain, so the default drive command keeps running. Runs until
 * cancelled (bind it with whileHeld) and re-aims continuously, so it keeps tracking if the robot
 * is bumped or the driver moves. If the hive isn't visible the driver's own turning is left alone.
 */
public class CMD_AimAtHive extends CommandBase {

    private final RobotContainer m_robot;
    private final ElapsedTime m_timer = new ElapsedTime();
    private final ElapsedTime m_settle = new ElapsedTime();
    private boolean m_aligned;

    public CMD_AimAtHive(RobotContainer p_robot) {
        m_robot = p_robot;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_settle.reset();
        m_aligned = false;
    }

    @Override
    public void execute() {
        AprilTagClusterDetection hive = m_robot.m_vision.getHiveCluster(m_robot.getRedSide());
        if (hive == null) {
            // Hive not visible this frame: let the driver turn to find it
            m_robot.drivetrain.clearTurnOverride();
            m_settle.reset();
            m_aligned = false;
            return;
        }

        // The cluster pose points at the center of the hive opening
        double error = hive.ftcPose.bearing - VisionConstants.kAimOffsetDeg;

        if (Math.abs(error) < VisionConstants.kAimToleranceDeg) {
            m_robot.drivetrain.setTurnOverride(0.0);   // hold the aim
            if (!m_aligned) {
                m_aligned = true;
                m_settle.reset();
            }
            return;
        }

        m_aligned = false;
        m_settle.reset();

        double turn = Math.abs(error * VisionConstants.kAimKP);
        turn = Math.max(VisionConstants.kAimMinTurn, Math.min(VisionConstants.kAimMaxTurn, turn));
        // bearing > 0 means the hive is to the left of the camera
        turn *= Math.signum(error) * VisionConstants.kAimTurnSign;

        m_robot.drivetrain.setTurnOverride(turn);
    }

    /** True once lined up and held there for kAimSettleMs. */
    public boolean isAligned() {
        return m_aligned && m_settle.milliseconds() >= VisionConstants.kAimSettleMs;
    }

    /** Never finishes on its own: it keeps aiming until cancelled (button released). */
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_robot.drivetrain.clearTurnOverride();
    }
}
