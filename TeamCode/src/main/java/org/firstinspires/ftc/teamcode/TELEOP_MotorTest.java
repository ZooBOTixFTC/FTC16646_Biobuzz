package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Drivetrain motor test. Holds ONE motor at a chosen voltage so you can check
 * direction, wiring and encoders. It does not use RoadRunner or the scheduler.
 *
 *   A = front left    B = front right    X = back left    Y = back right
 *   Hold the button to run that motor FORWARD.
 *   Hold LEFT BUMPER at the same time to run it in REVERSE.
 *   D-pad UP / DOWN = volts +0.5 / -0.5
 *
 * FTC motors take power from -1 to 1, not volts. This converts:
 *   power = volts / battery voltage
 * so "8 volts" is about 0.62 power on a 13 V battery.
 */
@TeleOp(name = "Motor Test", group = "Test")
public class TELEOP_MotorTest extends LinearOpMode {

    // Encoder ticks per ONE output-shaft turn of your drive motors.
    // 537.7 = goBILDA 19.2:1 (312 RPM). Change this to match your motors.
    private static final double TICKS_PER_REV = 537.7;

    private static double rpm(DcMotorEx m) {
        return m.getVelocity() / TICKS_PER_REV * 60.0;
    }

    @Override
    public void runOpMode() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "fl_drive");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "fr_drive");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "bl_drive");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "br_drive");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        // No direction changes: shows each motor's raw direction as wired.
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        double volts = 8.0;
        boolean upLast = false, downLast = false;

        telemetry.addLine("Hold A/B/X/Y to run FL/FR/BL/BR. Left bumper = reverse.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upLast) volts += 0.5;
            if (gamepad1.dpad_down && !downLast) volts -= 0.5;
            upLast = gamepad1.dpad_up;
            downLast = gamepad1.dpad_down;

            double batt = battery.getVoltage();
            volts = Range.clip(volts, 0, batt);
            double power = Range.clip(volts / batt, 0, 1) * (gamepad1.left_bumper ? -1 : 1);

            fl.setPower(gamepad1.a ? power : 0);
            fr.setPower(gamepad1.b ? power : 0);
            bl.setPower(gamepad1.x ? power : 0);
            br.setPower(gamepad1.y ? power : 0);

            telemetry.addData("Battery V", "%.2f", batt);
            telemetry.addData("Target V", "%.1f", volts);
            telemetry.addData("Power sent", "%.2f", power);
            telemetry.addData("FL RPM", "%.0f   (ticks %d)", rpm(fl), fl.getCurrentPosition());
            telemetry.addData("FR RPM", "%.0f   (ticks %d)", rpm(fr), fr.getCurrentPosition());
            telemetry.addData("BL RPM", "%.0f   (ticks %d)", rpm(bl), bl.getCurrentPosition());
            telemetry.addData("BR RPM", "%.0f   (ticks %d)", rpm(br), br.getCurrentPosition());
            telemetry.update();
        }

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setPower(0);
    }
}