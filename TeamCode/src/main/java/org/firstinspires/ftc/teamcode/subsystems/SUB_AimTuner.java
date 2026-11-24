package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/**
 * In-match aim tuner (gamepad 2). Changes the VisionConstants aim numbers live, so
 * CMD_AimAtHive uses them on the very next RB/LB press. No laptop needed.
 *
 *   Hold GP2 LEFT BUMPER, then:
 *     D-pad up/down     = pick a number (">>" on the DS shows which one)
 *     D-pad right/left  = make it bigger/smaller
 *     B                 = undo every change made since this TeleOp started
 *   Without GP2 left bumper held, the D-pad and B do nothing (no accidental changes).
 *
 * Changes last until the robot restarts. Write the good numbers down and copy them
 * into Constants.java (VisionConstants).
 */
public class SUB_AimTuner extends SubsystemBase {

    private static final String[] NAMES = {
            "kAimKP", "kAimMinTurn", "kAimMaxTurn", "kAimToleranceDeg", "kAimSettleMs", "kAimOffsetDeg"};
    private static final double[] STEPS = {0.002, 0.01, 0.05, 0.25, 25, 0.5};

    private final RobotContainer m_robot;
    private final Telemetry m_telemetry;
    private final Gamepad m_driver;
    private final Gamepad m_tuner;
    private final double[] m_startValues = new double[NAMES.length];

    private int m_selected = 0;
    private boolean m_lastUp, m_lastDown, m_lastLeft, m_lastRight, m_lastB;

    // Times how long it takes from pressing RB/LB until the hive is lined up
    private final ElapsedTime m_aimTimer = new ElapsedTime();
    private boolean m_lastAimHeld = false;
    private boolean m_timing = false;
    private double m_lastAimSec = -1;

    public SUB_AimTuner(RobotContainer p_robot, Telemetry p_telemetry, Gamepad p_driver, Gamepad p_tuner) {
        m_robot = p_robot;
        m_telemetry = p_telemetry;
        m_driver = p_driver;
        m_tuner = p_tuner;
        for (int i = 0; i < NAMES.length; i++) {
            m_startValues[i] = get(i);
        }
    }

    private double get(int i) {
        switch (i) {
            case 0: return VisionConstants.kAimKP;
            case 1: return VisionConstants.kAimMinTurn;
            case 2: return VisionConstants.kAimMaxTurn;
            case 3: return VisionConstants.kAimToleranceDeg;
            case 4: return VisionConstants.kAimSettleMs;
            default: return VisionConstants.kAimOffsetDeg;
        }
    }

    private void set(int i, double value) {
        if (i != 5) {
            value = Math.max(0, value);   // only the offset is allowed to go negative
        }
        switch (i) {
            case 0: VisionConstants.kAimKP = value; break;
            case 1: VisionConstants.kAimMinTurn = value; break;
            case 2: VisionConstants.kAimMaxTurn = value; break;
            case 3: VisionConstants.kAimToleranceDeg = value; break;
            case 4: VisionConstants.kAimSettleMs = value; break;
            default: VisionConstants.kAimOffsetDeg = value; break;
        }
    }

    @Override
    public void periodic() {
        // ---- Buttons (only while GP2 left bumper is held) ----
        boolean armed = m_tuner.left_bumper;
        boolean up = armed && m_tuner.dpad_up;
        boolean down = armed && m_tuner.dpad_down;
        boolean left = armed && m_tuner.dpad_left;
        boolean right = armed && m_tuner.dpad_right;
        boolean b = armed && m_tuner.b;

        if (up && !m_lastUp) m_selected = (m_selected + NAMES.length - 1) % NAMES.length;
        if (down && !m_lastDown) m_selected = (m_selected + 1) % NAMES.length;
        if (right && !m_lastRight) set(m_selected, get(m_selected) + STEPS[m_selected]);
        if (left && !m_lastLeft) set(m_selected, get(m_selected) - STEPS[m_selected]);
        if (b && !m_lastB) {
            for (int i = 0; i < NAMES.length; i++) set(i, m_startValues[i]);
        }
        m_lastUp = up; m_lastDown = down; m_lastLeft = left; m_lastRight = right; m_lastB = b;

        // ---- Aim timer ----
        AprilTagClusterDetection hive = m_robot.m_vision.getHiveCluster(m_robot.getRedSide());
        double error = (hive == null) ? 0 : hive.ftcPose.bearing - VisionConstants.kAimOffsetDeg;
        boolean aimHeld = m_driver.right_bumper || m_driver.left_bumper;
        if (aimHeld && !m_lastAimHeld) {
            m_aimTimer.reset();
            m_timing = true;
        }
        if (m_timing && aimHeld && hive != null && Math.abs(error) < VisionConstants.kAimToleranceDeg) {
            m_lastAimSec = m_aimTimer.seconds();
            m_timing = false;
        }
        if (!aimHeld) m_timing = false;
        m_lastAimHeld = aimHeld;

        // ---- Driver Station ----
        m_telemetry.addLine("AIM TUNER: hold GP2 LB + d-pad (B = undo all)");
        for (int i = 0; i < NAMES.length; i++) {
            m_telemetry.addData((i == m_selected ? ">> " : "   ") + NAMES[i], "%.3f", get(i));
        }
        m_telemetry.addData("Hive", hive == null ? "not seen" : String.format("error %.1f deg", error));
        m_telemetry.addData("Last aim time", m_lastAimSec < 0 ? "-" : String.format("%.2f s", m_lastAimSec));
    }
}
