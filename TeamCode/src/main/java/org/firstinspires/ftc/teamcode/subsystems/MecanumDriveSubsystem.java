package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.DriveParamsStore;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * FTCLib subsystem wrapper around the Road Runner 1.0 MecanumDrive.
 * - periodic() keeps the pose estimate updated every loop
 * - drive() handles teleop joystick driving (field-centric or robot-centric)
 * - addTelemetry() shows per-wheel distance (ticks + inches), power, and speed
 * - getDrive() exposes the Road Runner drive for building autonomous Actions
 */
public class MecanumDriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private boolean fieldCentric = false;

    // Encoder positions at the last resetDistance(), used as the "zero" for distance traveled
    private int lfStart, lbStart, rbStart, rfStart;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
        resetDistance();
    }

    @Override
    public void periodic() {
        // Null guard: SubsystemBase's constructor can register this subsystem with the
        // scheduler before the "drive" field finishes being assigned above (since super()
        // always runs first in Java). If periodic() fires in that window, drive is still
        // null -- this check prevents the crash without changing normal behavior otherwise.
        if (drive != null) {
            drive.updatePoseEstimate();
        }
    }

    /**
     * Drive with joystick-style inputs, each from -1 to 1.
     *
     * @param forward positive = drive forward
     * @param strafe  positive = strafe right
     * @param turn    positive = turn clockwise (right)
     */
    public void drive(double forward, double strafe, double turn) {
        // Road Runner frame: +x forward, +y LEFT, +heading counterclockwise
        Vector2d input = new Vector2d(forward, -strafe);

        if (fieldCentric) {
            // Rotate joystick input by the robot's heading so "up" on the stick is always downfield
            Rotation2d heading = drive.localizer.getPose().heading;
            input = heading.inverse().times(input);
        }

        drive.setDrivePowers(new PoseVelocity2d(input, -turn));
    }

    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public Pose2d getPoseEstimate() {
        return drive.localizer.getPose();
    }

    /** Also resets the field-centric "forward" direction. */
    public void setPoseEstimate(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    /**
     * Sets the current wheel positions as the new "zero" for distance traveled.
     * Does NOT reset the actual encoders, so Road Runner's position tracking is unaffected.
     */
    public void resetDistance() {
        lfStart = drive.leftFront.getCurrentPosition();
        lbStart = drive.leftBack.getCurrentPosition();
        rbStart = drive.rightBack.getCurrentPosition();
        rfStart = drive.rightFront.getCurrentPosition();
    }

    /** Per-wheel distance, power, and speed. Call every loop before telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive Save", DriveParamsStore.status);

        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        telemetry.addData("Ticks per inch", "%.2f", 1.0 / inPerTick);

        int lf = drive.leftFront.getCurrentPosition() - lfStart;
        int lb = drive.leftBack.getCurrentPosition() - lbStart;
        int rb = drive.rightBack.getCurrentPosition() - rbStart;
        int rf = drive.rightFront.getCurrentPosition() - rfStart;

        telemetry.addData("LF ticks / in", "%d / %.1f", lf, lf * inPerTick);
        telemetry.addData("LB ticks / in", "%d / %.1f", lb, lb * inPerTick);
        telemetry.addData("RB ticks / in", "%d / %.1f", rb, rb * inPerTick);
        telemetry.addData("RF ticks / in", "%d / %.1f", rf, rf * inPerTick);

        telemetry.addData("LF power / vel", "%.2f / %.0f", drive.leftFront.getPower(), drive.leftFront.getVelocity());
        telemetry.addData("LB power / vel", "%.2f / %.0f", drive.leftBack.getPower(), drive.leftBack.getVelocity());
        telemetry.addData("RB power / vel", "%.2f / %.0f", drive.rightBack.getPower(), drive.rightBack.getVelocity());
        telemetry.addData("RF power / vel", "%.2f / %.0f", drive.rightFront.getPower(), drive.rightFront.getVelocity());
    }

    /** Use this in autos: getDrive().actionBuilder(startPose)... */
    public MecanumDrive getDrive() {
        return drive;
    }
}