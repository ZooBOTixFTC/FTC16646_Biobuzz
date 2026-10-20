package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class LimelightOTOSLocalizer implements Localizer {

    public static class Params {
        // Rejection thresholds for Limelight vision updates
        public static double maxAngularVelocityDegSec = 120.0; // Reject vision updates if turning faster than this
        public static double maxLinearVelocityInchesSec = 30.0; // Reject vision updates if driving faster than this
        public static double maxTargetDistanceInches = 120.0;   // Reject vision updates if AprilTag is too far (~10ft)
    }

    public static Params PARAMS = new Params();

    private final OTOSLocalizer otosLocalizer;
    private final Limelight3A limelight;
    private boolean visionEnabled = true;

    public LimelightOTOSLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // Initialize the underlying OTOS localizer
        this.otosLocalizer = new OTOSLocalizer(hardwareMap, initialPose);

        // Fetch Limelight 3A instance (ensures device name matches Driver Hub config)
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Start Limelight pipeline
        limelight.pipelineSwitch(0); // Set to your AprilTag pipeline index
        limelight.start();
    }

    public LimelightOTOSLocalizer(HardwareMap hardwareMap, Pose2d initialPose, Limelight3A limelight) {
        this.otosLocalizer = new OTOSLocalizer(hardwareMap, initialPose);
        this.limelight = limelight;
    }

    @Override
    public Pose2d getPose() {
        return otosLocalizer.getPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        otosLocalizer.setPose(pose);
    }

    @Override
    public PoseVelocity2d update() {
        // 1. Get high-frequency velocity and pose estimate from OTOS
        PoseVelocity2d otosVel = otosLocalizer.update();
        Pose2d currentPose = otosLocalizer.getPose();

        // 2. Feed current gyro heading (in degrees) to Limelight for MegaTag2 localization
        double currentHeadingDeg = Math.toDegrees(currentPose.heading.toDouble());
        limelight.updateRobotOrientation(currentHeadingDeg);

        // 3. Conditionally evaluate Limelight AprilTag pose updates
        if (visionEnabled && shouldAcceptVisionUpdate(otosVel)) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botPoseMT2 = result.getBotpose_MT2();

                if (botPoseMT2 != null) {
                    // Extract position in inches
                    double visionX = botPoseMT2.getPosition().toUnit(DistanceUnit.INCH).x;
                    double visionY = botPoseMT2.getPosition().toUnit(DistanceUnit.INCH).y;

                    // Preserve trusted OTOS gyro heading to prevent rotational jumps
                    Pose2d correctedPose = new Pose2d(
                            visionX,
                            visionY,
                            currentPose.heading.toDouble()
                    );

                    // Inject global vision correction back into OTOS
                    otosLocalizer.setPose(correctedPose);
                }
            }
        }

        return otosVel;
    }

    /**
     * Determines whether to accept or reject vision updates based on chassis dynamics.
     */
    private boolean shouldAcceptVisionUpdate(PoseVelocity2d otosVel) {
        double linearSpeed = otosVel.linearVel.norm(); // inches/sec
        double angularSpeedDeg = Math.toDegrees(Math.abs(otosVel.angVel)); // deg/sec

        // Reject updates if robot is spinning rapidly (prevents motion blur error)
        if (angularSpeedDeg > Params.maxAngularVelocityDegSec) {
            return false;
        }

        // Reject updates if robot is moving too fast linearly
        if (linearSpeed > Params.maxLinearVelocityInchesSec) {
            return false;
        }

        return true;
    }

    /**
     * Toggle vision updates on or off (e.g. during specific auto maneuvers).
     */
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    public OTOSLocalizer getOtosLocalizer() {
        return otosLocalizer;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}