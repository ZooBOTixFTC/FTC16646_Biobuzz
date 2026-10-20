package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Two-motor flywheel shooter with one velocity PIDF controller.
 * Both motors drive the same flywheel and always get the same power.
 * Speed = average of both encoders.
 * 6000 RPM motors with a 3:1 reduction -> ~2000 RPM max at the flywheel.
 * All values are in flywheel (wheel) RPM.
 *
 * Tuned values can be saved to a file on the Control Hub with saveGains(),
 * and are loaded automatically every time the shooter is created.
 */
@Config
public class SUB_Shooter extends SubsystemBase {

    // ---- PIDF gains (tunable from Dashboard or D-pad) ----
    // These are the DEFAULTS, used only if no saved file exists.
    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0005;          // ~1 / 2000 max wheel RPM

    // ---- D-pad tuning step sizes ----
    public static double P_STEP = 0.00005;
    public static double D_STEP = 0.00001;

    // ---- Targets ----
    public static double TARGET_RPM = 1700;    // keep below ~2000 (max wheel RPM)
    public static double AT_SPEED_TOLERANCE_RPM = 50;

    // 28 ticks per motor rev x 3:1 reduction = 84 ticks per wheel rev
    public static double TICKS_PER_REV = 84;

    // Saved to /sdcard/FIRST/settings/ on the Control Hub
    private static final String SAVE_FILE_NAME = "shooter_pid.txt";

    private final DcMotorEx motor1;
    private final Servo m_stopper;
    private final PIDFController controller;

    private boolean enabled = false;
    private boolean useDashboardTarget = true;
    private double targetRpm = 0;
    private double lastPower = 0;
    private String saveStatus;

    public SUB_Shooter(HardwareMap hardwareMap) {
        // Load saved gains BEFORE building the controller
        saveStatus = loadGains() ? "Loaded saved values" : "Using code defaults";
        controller = new PIDFController(kP, kI, kD, kF);

        motor1 = hardwareMap.get(DcMotorEx.class, "shooter1_motor");
        m_stopper = hardwareMap.get(Servo.class, "stopper");

        // TODO: if the motors face opposite directions on the flywheel, one must be reversed.
        //   Test at a LOW target: both "Motor 1 RPM" and "Motor 2 RPM" must read POSITIVE.
        //   If one reads negative, flip that motor's direction here.
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{motor1}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we run our own PID; encoders still read
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let the flywheel coast down
        }
    }

    @Override
    public void periodic() {
        // Pick up any gain changes from Dashboard or the D-pad
        controller.setPIDF(kP, kI, kD, kF);

        if (useDashboardTarget && enabled) {
            targetRpm = TARGET_RPM;
        }

        if (!enabled) {
            setPower(0);
            return;
        }

        double power = controller.calculate(getRpm(), targetRpm);
        power = Range.clip(power, 0, 1); // flywheel only spins one way
        setPower(power);
    }

    private void setPower(double power) {
        motor1.setPower(power);
        lastPower = power;
    }

    /** Spin up to TARGET_RPM (follows live Dashboard edits). */
    public void spinUp() {
        useDashboardTarget = true;
        start(TARGET_RPM);
    }

    /** Spin up to a specific wheel RPM. */
    public void spinUp(double rpm) {
        useDashboardTarget = false;
        start(rpm);
    }

    public void start(double rpm) {
        if (!enabled) controller.reset(); // clear old integral/derivative state
        targetRpm = rpm;
        enabled = true;
    }

    public void stop() {
        enabled = false;
        targetRpm = 0;
    }

    public double getMotor1Rpm() {
        return motor1.getVelocity() / TICKS_PER_REV * 60.0;
    }

    /** Flywheel speed in RPM (average of both motors). */
    public double getRpm() {
        return (getMotor1Rpm() / 2.0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public boolean atSpeed() {
        return enabled && Math.abs(targetRpm - getRpm()) <= AT_SPEED_TOLERANCE_RPM;
    }

    // ---- D-pad tuning ----
    public void adjustP(double delta) {
        kP = Math.max(0, kP + delta);
    }

    public void adjustD(double delta) {
        kD = Math.max(0, kD + delta);
    }

    // ---- Saving / loading ----

    /** Saves kP, kI, kD, kF, and TARGET_RPM to a file on the Control Hub. */
    public void saveGains() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(SAVE_FILE_NAME);
            String data = kP + "," + kI + "," + kD + "," + kF + "," + TARGET_RPM;
            ReadWriteFile.writeFile(file, data);
            saveStatus = "SAVED  P=" + String.format("%.5f", kP)
                    + " D=" + String.format("%.5f", kD)
                    + " RPM=" + String.format("%.0f", TARGET_RPM);
        } catch (Exception e) {
            saveStatus = "SAVE FAILED: " + e.getMessage();
        }
    }

    /** Loads saved values if the file exists. Returns true if it loaded. */
    public static boolean loadGains() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(SAVE_FILE_NAME);
            if (!file.exists()) return false;

            String[] parts = ReadWriteFile.readFile(file).trim().split(",");
            if (parts.length < 5) return false;

            kP = Double.parseDouble(parts[0]);
            kI = Double.parseDouble(parts[1]);
            kD = Double.parseDouble(parts[2]);
            kF = Double.parseDouble(parts[3]);
            TARGET_RPM = Double.parseDouble(parts[4]);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /** Deletes the saved file so the code defaults are used next time. */
    public void clearSavedGains() {
        File file = AppUtil.getInstance().getSettingsFile(SAVE_FILE_NAME);
        saveStatus = (file.exists() && file.delete())
                ? "Saved file deleted (defaults next start)"
                : "No saved file to delete";
    }

    public String getSaveStatus() {
        return saveStatus;
    }

    /** Call every loop before telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        double rpm = getRpm();
        telemetry.addData("Shooter Target RPM", "%.0f", targetRpm);
        telemetry.addData("Shooter Actual RPM (avg)", "%.0f", rpm);
        telemetry.addData("Shooter Motor RPM", "%.0f", getMotor1Rpm());
        telemetry.addData("Shooter Error RPM", "%.0f", targetRpm - rpm);
        telemetry.addData("Shooter Power", "%.3f", lastPower);
        telemetry.addData("Shooter At Speed", atSpeed());
        telemetry.addData("Shooter PIDF", "P=%.5f I=%.5f D=%.5f F=%.6f", kP, kI, kD, kF);
        telemetry.addData("Shooter Save", saveStatus);
    }

    public void setStopperClosed(){
        m_stopper.setPosition(0);
    }

    public void setStopperOpen(){
        m_stopper.setPosition((double) 6 /7);
    }
}