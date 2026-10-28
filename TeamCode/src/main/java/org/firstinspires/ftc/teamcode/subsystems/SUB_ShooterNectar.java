package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/** NECTER (top) shooter: one flywheel motor with speed control. No stopper. */
public class SUB_ShooterNectar extends SubsystemBase {

    // ---- Speed control gains ----
    private static final double kP = 0.0005;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0005;

    private static final double AT_SPEED_TOLERANCE_RPM = 50;
    private static final double TICKS_PER_REV = 84;   // 28 ticks x 3:1 reduction = flywheel revs

    private final DcMotorEx motor;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    private boolean enabled = false;
    private double targetRpm = 0;
    private double lastPower = 0;

    public SUB_ShooterNectar(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooter2_motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void periodic() {
        if (motor == null) return;

        lastPower = enabled ? Range.clip(controller.calculate(getRpm(), targetRpm), 0, 1) : 0;
        motor.setPower(lastPower);
    }

    /** Spin up to the speed in Constants.ShooterConstants.shootingNectarRPM. */
    public void spinUp() {
        start(Constants.ShooterConstants.shootingNectarRPM);
    }

    public void start(double rpm) {
        if (!enabled) controller.reset();
        targetRpm = rpm;
        enabled = true;
    }

    public void stop() {
        enabled = false;
        targetRpm = 0;
    }

    public double getRpm() {
        return motor.getVelocity() / TICKS_PER_REV * 60.0;
    }

    public boolean atSpeed() {
        return enabled && Math.abs(targetRpm - getRpm()) <= AT_SPEED_TOLERANCE_RPM;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Necter Target RPM", "%.0f", targetRpm);
        telemetry.addData("Necter Actual RPM", "%.0f", getRpm());
        telemetry.addData("Necter At Speed", atSpeed());
    }
}