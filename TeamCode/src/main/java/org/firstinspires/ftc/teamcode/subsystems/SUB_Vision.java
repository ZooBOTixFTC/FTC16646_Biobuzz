package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * AprilTag detection with a live camera stream.
 *
 * Constructing this subsystem (e.g. during init) starts the camera, so the stream is available on
 * the Driver Station (3-dot menu -> Camera Stream) and on FTC Dashboard (http://192.168.43.1:8080/dash)
 * before the match starts. Call close() when finished to release the camera.
 */
@Config     // MAX_ROLL_DEG shows on FTC Dashboard
public class SUB_Vision extends SubsystemBase {

    /**
     * A cluster tipped further than this is treated as upside down and ignored (Task 4.4).
     * Upright tags read roll near 0; a tipped-over hive's tags read near 180.
     */
    public static double MAX_ROLL_DEG = 90;

    /** Must match the webcam's name in the robot configuration. */
    private static final String CAMERA_NAME = "apriltag_cam";
    private static final int STREAM_FPS = 15;

    private final OpMode m_OpMode;
    private final AprilTagProcessor m_aprilTag;
    private final VisionPortal m_portal;

    public SUB_Vision(OpMode p_OpMode) {
        m_OpMode = p_OpMode;

        m_aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        m_portal = new VisionPortal.Builder()
                .setCamera(m_OpMode.hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(m_aprilTag)
                .enableLiveView(true)      // RC screen + Driver Station camera stream
                .setAutoStopLiveView(true) // stop streaming after start
                .build();

        // Also stream to FTC Dashboard (already a project dependency).
        FtcDashboard.getInstance().startCameraStream(m_portal, STREAM_FPS);
    }

    /** All tags currently seen (empty list if none). */
    public List<AprilTagDetection> getDetections() {
        List<AprilTagDetection> detections = m_aprilTag.getDetections();
        return detections != null ? detections : new ArrayList<>();
    }

    /** Single-tag detections only (SDK 12.0+: id/metadata/corners live on the single subclass). */
    public List<AprilTagSingleDetection> getSingleDetections() {
        List<AprilTagSingleDetection> singles = new ArrayList<>();
        for (AprilTagDetection d : getDetections()) {
            if (d instanceof AprilTagSingleDetection) singles.add((AprilTagSingleDetection) d);
        }
        return singles;
    }

    /** Cluster detections only (one per cluster). Not used yet, but ready for when clusters are. */
    public List<AprilTagClusterDetection> getClusterDetections() {
        List<AprilTagClusterDetection> clusters = new ArrayList<>();
        for (AprilTagDetection d : getDetections()) {
            if (d instanceof AprilTagClusterDetection) clusters.add((AprilTagClusterDetection) d);
        }
        return clusters;
    }

    /**
     * Our alliance's hive cluster, RIGHT SIDE UP, or null if there isn't one in view (Task 4.4).
     * - The name must start with the whole word "RED" or "BLUE" (not just R or B), so the other
     *   alliance's clusters are never returned.
     * - The name must contain Constants.VisionConstants.kHiveClusterKeyword (e.g. "SCORING").
     * - The cluster must be right side up (|roll| < MAX_ROLL_DEG). A tipped hive's tags are
     *   upside down, and we don't aim at those.
     */
    public AprilTagClusterDetection getHiveCluster(boolean red) {
        String alliance = red ? "RED" : "BLUE";
        for (AprilTagClusterDetection c : getClusterDetections()) {
            if (c.metadata == null || c.ftcPose == null) continue;
            String name = c.metadata.name.toUpperCase();
            boolean ours = name.startsWith(alliance)
                    && name.contains(Constants.VisionConstants.kHiveClusterKeyword);
            boolean rightSideUp = Math.abs(c.ftcPose.roll) < MAX_ROLL_DEG;
            if (ours && rightSideUp) {
                return c;
            }
        }
        return null;
    }

    /** The single-tag detection for a specific tag ID, or null if it isn't visible. */
    public AprilTagSingleDetection getTag(int id) {
        for (AprilTagSingleDetection d : getSingleDetections()) {
            if (d.id == id) return d;
        }
        return null;
    }

    public boolean hasTag(int id) {
        return getTag(id) != null;
    }

    /** Turn processing on/off (e.g. to save CPU while driving). The stream keeps running. */
    public void setDetectionEnabled(boolean enabled) {
        m_portal.setProcessorEnabled(m_aprilTag, enabled);
    }

    /** Stop streaming and free the camera. */
    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        m_portal.close();
    }

    @Override
    public void periodic() {
        m_OpMode.telemetry.addData("Vision tags", getDetections().size());
        for (AprilTagDetection d : getDetections()) {
            if (d instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) d;
                if (single.metadata != null && single.ftcPose != null) {
                    m_OpMode.telemetry.addData("Tag " + single.id + " (" + single.metadata.name + ")",
                            "range %.1f in, bearing %.1f deg, yaw %.1f deg",
                            single.ftcPose.range, single.ftcPose.bearing, single.ftcPose.yaw);
                } else {
                    m_OpMode.telemetry.addData("Tag " + single.id, "unknown tag");
                }
            } else if (d instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) d;
                String name = cluster.metadata != null ? cluster.metadata.name : "unknown";
                if (cluster.ftcPose != null) {
                    // roll near 0 = upright, near 180 = upside down (tipped hive)
                    m_OpMode.telemetry.addData("Cluster " + name,
                            "%d%% found, bearing %.1f deg, roll %.0f deg, range %.0f in",
                            cluster.percentClusterFound,
                            cluster.ftcPose.bearing, cluster.ftcPose.roll, cluster.ftcPose.range);
                } else {
                    m_OpMode.telemetry.addData("Cluster " + name, "%d%% found, no position",
                            cluster.percentClusterFound);
                }
            }
        }
    }
}