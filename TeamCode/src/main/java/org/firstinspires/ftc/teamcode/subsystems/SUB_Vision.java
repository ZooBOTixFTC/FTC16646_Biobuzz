package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
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
public class SUB_Vision extends SubsystemBase {

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
     * The hive cluster for the given alliance, or null if it isn't visible. The other alliance's
     * clusters are never returned. Cluster names start with "R" or "B" (e.g. "RED SCORING"),
     * and the name must contain Constants.VisionConstants.kHiveClusterKeyword.
     */
    public AprilTagClusterDetection getHiveCluster(boolean red) {
        String alliancePrefix = red ? "R" : "B";
        for (AprilTagClusterDetection c : getClusterDetections()) {
            if (c.metadata == null || c.ftcPose == null) continue;
            String name = c.metadata.name.toUpperCase();
            if (name.startsWith(alliancePrefix)
                    && name.contains(Constants.VisionConstants.kHiveClusterKeyword)) {
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
                m_OpMode.telemetry.addData("Cluster " + cluster.metadata.name,
                        "%d%% found, bearing %.1f deg",
                        cluster.percentClusterFound,
                        cluster.ftcPose != null ? cluster.ftcPose.bearing : Double.NaN);
            }
        }
    }
}
