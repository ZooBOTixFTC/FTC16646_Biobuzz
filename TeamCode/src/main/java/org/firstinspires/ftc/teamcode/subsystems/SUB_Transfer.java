package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;

public class SUB_Transfer extends SubsystemBase {

    private final DcMotorEx m_transferMotor;
    private final OpMode m_OpMode;

    public SUB_Transfer(OpMode p_OpMode) {
        m_OpMode = p_OpMode;

        m_transferMotor = m_OpMode.hardwareMap.get(DcMotorEx.class, "transferMotor");
        m_transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_transferMotor.setPower(0);
        m_transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void kTransferOn() {
        m_transferMotor.setPower(Constants.TransferConstants.kTransferOn);
    }

    public void kTransferOff() {
        m_transferMotor.setPower(Constants.TransferConstants.kTransferOff);
    }

    public void setMotorPower(double Speed) {
        m_transferMotor.setPower(Speed);
    }
}
