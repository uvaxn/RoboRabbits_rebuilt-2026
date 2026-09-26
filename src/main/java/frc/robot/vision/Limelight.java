package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Landmarks;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final double FIELD_LENGTH_M = FIELD.getFieldLength();
    private static final double FIELD_WIDTH_M = FIELD.getFieldWidth();
    // do not correct gyro
    private static final double POS_STD_DEV = 0.7;
    private static final double ROTATION_STD_DEV = 9999.0;

    private static final double MAX_ESTIMATE_AGE_SECONDS = 0.25;

    private final String name;
    private final StructPublisher<Pose2d> posePublisher;
    private final IntegerArrayPublisher visibleTagIdsPublisher;

    private Optional<Pose2d> latestEstimate = Optional.empty();
    private double latestEstimateTimestamp = 0.0;
    private double latestHubDist = 0.0;
    public Limelight(String name) {
        this.name = name;
        NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(name);
        this.posePublisher = telemetryTable.getStructTopic("EstimatedPose", Pose2d.struct).publish();
        this.visibleTagIdsPublisher = telemetryTable.getIntegerArrayTopic("VisibleTagIDs").publish();
    }

    /**
     * Publishes the AprilTag IDs currently visible to this camera to NetworkTables
     * (empty array when none are in view). Runs every {@link #periodic()} tick,
     * independent of whether a full pose estimate is available, so it reflects what
     * the camera sees right now.
     */
    private void publishVisibleTags(PoseEstimate estimate) {
        LimelightHelpers.RawFiducial[] fiducials =
            (estimate != null && estimate.rawFiducials != null) ? estimate.rawFiducials : new LimelightHelpers.RawFiducial[0];
        long[] ids = new long[fiducials.length];
        for (int i = 0; i < fiducials.length; i++) {
            ids[i] = fiducials[i].id;
        }
        visibleTagIdsPublisher.set(ids);
    }

    @Override
    public void periodic() {
        publishVisibleTags(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
    }
    private static boolean isInField(Translation2d t) {
        return t.getX() >= 0 && t.getX() <= FIELD_LENGTH_M
                && t.getY() >= 0 && t.getY() <= FIELD_WIDTH_M;
    }

    /**
     * @param currentRobotPose current pose estimate.
     */
    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt1 == null || mt2 == null || mt1.tagCount == 0 || mt2.tagCount == 0) {
            return Optional.empty();
        }

        final Translation2d mt2Translation = mt2.pose.getTranslation();
        if (!isInField(mt2Translation)) {
            DriverStation.reportWarning("Limelight pose out of bounds", false);
            return Optional.empty();
        }

        final Pose2d fusedPose = new Pose2d(mt2Translation, mt1.pose.getRotation());
        final Matrix<N3, N1> standardDeviations =
                VecBuilder.fill(POS_STD_DEV, POS_STD_DEV, ROTATION_STD_DEV);

        posePublisher.set(fusedPose);
        latestEstimate = Optional.of(fusedPose);
        latestEstimateTimestamp = mt2.timestampSeconds;
        latestHubDist = mt2Translation.getDistance(Landmarks.getTeamHubTranslation());

        return Optional.of(new Measurement(fusedPose, mt2.timestampSeconds, standardDeviations));
    }

    public double getDistanceToHub() {
        if (latestEstimate.isEmpty()
                || Timer.getFPGATimestamp() - latestEstimateTimestamp > MAX_ESTIMATE_AGE_SECONDS) {
            return 3.0; // default fallback
        }
        return latestHubDist;
    }

    public Optional<Pose2d> getEstimatedPose() {
        if (latestEstimate.isEmpty()) {
            return Optional.empty();
        }
        if (Timer.getFPGATimestamp() - latestEstimateTimestamp > MAX_ESTIMATE_AGE_SECONDS) {
            return Optional.empty();
        }
        return latestEstimate;
    }

    public static class Measurement {
        public final Pose2d pose;
        public final double timestamp;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviations) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.standardDeviations = standardDeviations;
        }
    }
}