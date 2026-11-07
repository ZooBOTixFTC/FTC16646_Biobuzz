package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;

/** Intake and transfer are the same motor, so this one subsystem runs both. */
public class SUB_Intake extends SubsystemBase {

    private final DcMotorEx m_intakeMotor;
    private final OpMode m_OpMode;

    public SUB_Intake(OpMode p_OpMode) {
        m_OpMode = p_OpMode;

        m_intakeMotor = m_OpMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        m_intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intakeMotor.setPower(0);
        m_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void kIntakeOn() {
        m_intakeMotor.setPower(Constants.IntakeConstants.kIntakeOn);
    }

    public void kIntakeOff() {
        m_intakeMotor.setPower(Constants.IntakeConstants.kIntakeOff);
    }

    public void kIntakeOn2() {
        m_intakeMotor.setPower(Constants.IntakeConstants.kIntakeOn2);
    }


    public void kIntakeReverse() {
        m_intakeMotor.setPower(Constants.IntakeConstants.kIntakeReverse);
    }
    public void setMotorPower(double Speed) {
        m_intakeMotor.setPower(Speed);
    }

    /** The power the motor is currently being told to run at (for telemetry). */
    public double getPower() {
        return m_intakeMotor.getPower();
    }
}