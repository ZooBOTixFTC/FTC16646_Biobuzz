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

    public void kIntakeReverse() {
        m_intakeMotor.setPower(Constants.IntakeConstants.kIntakeReverse);
    }

    // Same motor, so these just do the same thing (kept so older code still compiles)
    public void kTransferOn() {
        kIntakeOn();
    }

    public void kTransferOff() {
        kIntakeOff();
    }

    public void setMotorPower(double Speed) {
        m_intakeMotor.setPower(Speed);
    }
}