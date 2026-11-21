package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * AprilTag detection with a live camera stream.
 *
 * Constructing this subsystem (e.g. during init) starts the camera, so the stream is available on
 * the Driver Station (3-dot menu -> Camera Stream) and on FTC Dashboard (http://192.168.43.1:8080/dash)
 * before the match starts. Call close() when finished to release the camera.
 *
 * Camera settings (exposure, gain, decimation) are tunable live from FTC Dashboard. They are applied
 * by updateCameraSettings(), which periodic() calls. periodic() does NOT run during init in a
 * CommandOpMode, so call updateCameraSettings() from your init loop if you want to tune before start.
 */
@Config     // these public static fields show on FTC Dashboard
public class SUB_Vision extends SubsystemBase {

    /**
     * A cluster tipped further than this is treated as upside down and ignored (Task 4.4).
     * Upright tags read roll near 0; a tipped-over hive's tags read near 180.
     */
    public static double MAX_ROLL_DEG = 90;

    /** true = use EXPOSURE_MS / GAIN below. false = camera's own auto exposure. */
    public static boolean MANUAL_EXPOSURE = true;
    /** Exposure time in ms. Lower = less motion blur but darker image. Start around 5-6. */
    public static int EXPOSURE_MS = 25;
    /** Brightens the image after lowering exposure. Range is camera-specific (often 0-255). */
    public static int GAIN = 150;
    /** AprilTag decimation: 1 = detects farthest but slowest, 2 = balanced, 3 = fast but short range. */
    public static double DECIMATION = 2;

    /** Must match the webcam's name in the robot configuration. */
    private static final String CAMERA_NAME = "apriltag_cam";
    private static final int STREAM_FPS = 15;

    private final OpMode m_OpMode;
    private final AprilTagProcessor m_aprilTag;
    private final VisionPortal m_portal;

    // Last values actually sent to the camera, so we only re-send when something changes.
    private Boolean m_appliedManual = null;
    private int m_appliedExposureMs = -1;
    private int m_appliedGain = -1;
    private double m_appliedDecimation = -1;

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

    /**
     * Sends exposure/gain/decimation to the camera if they changed. Safe to call every loop.
     * Camera controls only work once the camera is STREAMING, so this quietly does nothing until then.
     */
    public void updateCameraSettings() {
        if (DECIMATION != m_appliedDecimation) {
            m_aprilTag.setDecimation((float) DECIMATION);
            m_appliedDecimation = DECIMATION;
        }

        if (m_portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        boolean changed = !Boolean.valueOf(MANUAL_EXPOSURE).equals(m_appliedManual)
                || EXPOSURE_MS != m_appliedExposureMs
                || GAIN != m_appliedGain;
        if (!changed) return;

        ExposureControl exposure = m_portal.getCameraControl(ExposureControl.class);
        GainControl gain = m_portal.getCameraControl(GainControl.class);
        if (exposure == null) return;

        if (!MANUAL_EXPOSURE) {
            exposure.setMode(ExposureControl.Mode.Auto);
        } else {
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                // Switch modes this loop; set the values on the next call (avoids sleeping here).
                exposure.setMode(ExposureControl.Mode.Manual);
                return;
            }
            long minExp = exposure.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            long maxExp = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
            exposure.setExposure(clamp(EXPOSURE_MS, minExp, maxExp), TimeUnit.MILLISECONDS);

            if (gain != null) {
                gain.setGain((int) clamp(GAIN, gain.getMinGain(), gain.getMaxGain()));
            }
        }

        m_appliedManual = MANUAL_EXPOSURE;
        m_appliedExposureMs = EXPOSURE_MS;
        m_appliedGain = GAIN;
    }

    private static long clamp(long v, long lo, long hi) {
        return Math.max(lo, Math.min(hi, v));
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
                    && (name.contains("SCORING") || name.contains("AUDIENCE"));
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
        updateCameraSettings();

        m_OpMode.telemetry.addData("Camera", "%s, exposure %s, gain %d, decimation %.0f",
                m_portal.getCameraState(),
                MANUAL_EXPOSURE ? EXPOSURE_MS + " ms" : "auto", GAIN, DECIMATION);

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