package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/** POLLEN (bottom) shooter: one flywheel motor with speed control, plus the ball stopper servo. */
public class SUB_ShooterPollen extends SubsystemBase {

    // ---- Speed control gains ----
    private static final double kP = 0.0005;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0005;

    private static final double AT_SPEED_TOLERANCE_RPM = 50;
    private static final double TICKS_PER_REV = 84;   // 28 ticks x 3:1 reduction = flywheel revs

    // ---- Stopper servo (goBILDA 300 degree servo) ----
    private static final double STOPPER_TURN_FRACTION = 90.0 / 300.0;   // 90 degrees of 300
    private static final double STOPPER_CLOSED = 0.0;                   // base position blocks balls
    private static final double STOPPER_OPEN = STOPPER_TURN_FRACTION;   // turned 90 degrees lets balls in

    private final DcMotorEx motor;
    private final ServoImplEx stopper;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    private boolean enabled = false;
    private double targetRpm = 0;
    private double lastPower = 0;

    public SUB_ShooterPollen(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooter1_motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stopper = hardwareMap.get(ServoImplEx.class, "stopper");
        stopper.setPwmRange(new PwmControl.PwmRange(500, 2500));
        stopper.setDirection(Servo.Direction.REVERSE);   // turns clockwise. Use FORWARD if backwards.
        setStopperClosed();
    }

    @Override
    public void periodic() {
        if (motor == null) return;

        lastPower = enabled ? Range.clip(controller.calculate(getRpm(), targetRpm), 0, 1) : 0;
        motor.setPower(lastPower);
    }

    // ---- Shooter ----

    /** Spin up to the speed in Constants.ShooterConstants.shootingPollenRPM. */
    public void spinUp() {
        start(Constants.ShooterConstants.shootingPollenRPM);
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

    // ---- Stopper ----

    public void setStopperClosed() {
        stopper.setPosition(STOPPER_CLOSED);
    }

    public void setStopperOpen() {
        stopper.setPosition(STOPPER_OPEN);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pollen Target RPM", "%.0f", targetRpm);
        telemetry.addData("Pollen Actual RPM", "%.0f", getRpm());
        telemetry.addData("Pollen At Speed", atSpeed());
    }
}