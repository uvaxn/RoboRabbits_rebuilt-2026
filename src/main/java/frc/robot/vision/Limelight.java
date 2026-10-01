package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Landmarks;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

    private static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final double FIELD_LENGTH_M = FIELD.getFieldLength();
    private static final double FIELD_WIDTH_M = FIELD.getFieldWidth();

    private static final double MAX_ESTIMATE_AGE_SECONDS = 0.25;

    private static final double ROTATION_STD_DEV = 9999.0;

    // A tag farther than this from the CAMERA is dropped outright. 
    private static final double MAX_TAG_DISTANCE_METERS = 3.0;

    // A tag whose solvePnP ambiguity exceeds this is dropped outright.
    private static final double AMBIGUITY_REJECT_THRESHOLD = 0.3;

    // A LONE tag must clear a strictly tighter bar than the general thresholds above, because with
    // only one tag there's no second observation to catch a bad solve. These only gate whether a
    // single-tag measurement is used AT ALL; they never apply once 2+ tags are valid.
    private static final double SINGLE_TAG_MAX_DISTANCE_METERS = 2.0;
    private static final double SINGLE_TAG_MAX_AMBIGUITY = 0.1;

    private static final double STD_DEV_BASE_1_TAG = 1.5;       // barely trusted
    private static final double STD_DEV_BASE_2_TAG = 0.5;       // moderately trusted
    private static final double STD_DEV_BASE_3_PLUS_TAG = 0.1;  // strongly trusted

    // ow much worse (mutiplicatively) the std dev gets as avg. valid-tag distance/ambiguity walks
    // from "perfect" (0) up to that quantity's own reject threshold. 1.0x at the perfect end, scaling
    private static final double DISTANCE_PENALTY_AT_MAX_RANGE = 3.0;
    private static final double AMBIGUITY_PENALTY_AT_THRESHOLD = 2.0;

    private final String name;
    private final StructPublisher<Pose2d> posePublisher;
    // Why the last frame was accepted (with its tag count/stddev) or dropped. Meant for tuning the
    // constants above from the dashboard rather than guessing.
    private final StringPublisher statusPublisher;

    private Optional<Pose2d> latestEstimate = Optional.empty();
    private double latestEstimateTimestamp = 0.0;
    private double latestHubDist = 0.0;

    public Limelight(String name) {
        this.name = name;
        NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(name);
        this.posePublisher = telemetryTable.getStructTopic("EstimatedPose", Pose2d.struct).publish();
        this.statusPublisher = telemetryTable.getStringTopic("VisionStatus").publish();
    }

    private static boolean isInField(Translation2d t) {
        return t.getX() >= 0 && t.getX() <= FIELD_LENGTH_M
                && t.getY() >= 0 && t.getY() <= FIELD_WIDTH_M;
    }

    /**
     * @param currentRobotPose current pose estimate, used to seed MegaTag2's yaw input.
     */
    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt1 == null || mt2 == null || mt1.tagCount == 0 || mt2.tagCount == 0) {
            return drop("no tags");
        }
        final List<RawFiducial> validTags = filterValidTags(mt2.rawFiducials);
        if (validTags.isEmpty()) {
            return drop("no tags passed distance/ambiguity filtering");
        }
        if (validTags.size() == 1) {
            RawFiducial onlyTag = validTags.get(0);
            if (onlyTag.distToCamera > SINGLE_TAG_MAX_DISTANCE_METERS
                    || onlyTag.ambiguity > SINGLE_TAG_MAX_AMBIGUITY) {
                return drop(String.format(
                        "single tag not high-quality/close enough (%.2f m, ambiguity %.3f)",
                        onlyTag.distToCamera, onlyTag.ambiguity));
            }
        }

        final Translation2d mt2Translation = mt2.pose.getTranslation();
        if (!isInField(mt2Translation)) {
            DriverStation.reportWarning("Limelight pose out of bounds", false);
            return drop("pose out of field bounds");
        }

        final Pose2d fusedPose = new Pose2d(mt2Translation, mt1.pose.getRotation());
        final Matrix<N3, N1> standardDeviations = calculateStandardDeviations(mt2);

        posePublisher.set(fusedPose);
        statusPublisher.set(String.format("ok: %d/%d tag(s) valid, xyStdDev %.3f m",
                validTags.size(), mt2.tagCount, standardDeviations.get(0, 0)));

        latestEstimate = Optional.of(fusedPose);
        latestEstimateTimestamp = mt2.timestampSeconds;
        latestHubDist = mt2Translation.getDistance(Landmarks.getTeamHubTranslation());

        return Optional.of(new Measurement(fusedPose, mt2.timestampSeconds, standardDeviations));
    }

    private List<RawFiducial> filterValidTags(RawFiducial[] rawFiducials) {
        List<RawFiducial> valid = new ArrayList<>();
        if (rawFiducials == null) {
            return valid; // no per-tag data to inspect -> nothing can be confirmed valid
        }
        for (RawFiducial tag : rawFiducials) {
            if (tag.distToCamera > MAX_TAG_DISTANCE_METERS) continue;
            if (tag.ambiguity > AMBIGUITY_REJECT_THRESHOLD) continue;
            valid.add(tag);
        }
        return valid;
    }

    private Matrix<N3, N1> calculateStandardDeviations(PoseEstimate estimate) {
        List<RawFiducial> validTags = filterValidTags(estimate.rawFiducials);

        double avgDistance = validTags.stream().mapToDouble(t -> t.distToCamera).average().orElse(0.0);
        double avgAmbiguity = validTags.stream().mapToDouble(t -> t.ambiguity).average().orElse(0.0);

        double xyStdDev = baseStdDevForTagCount(validTags.size())
                * distancePenalty(avgDistance)
                * ambiguityPenalty(avgAmbiguity);

        return VecBuilder.fill(xyStdDev, xyStdDev, ROTATION_STD_DEV);
    }

    private static double baseStdDevForTagCount(int validTagCount) {
        if (validTagCount >= 3) return STD_DEV_BASE_3_PLUS_TAG;
        if (validTagCount == 2) return STD_DEV_BASE_2_TAG;
        return STD_DEV_BASE_1_TAG;
    }

    private static double distancePenalty(double avgDistanceMeters) {
        double ratio = avgDistanceMeters / MAX_TAG_DISTANCE_METERS;
        return 1.0 + ratio * ratio * (DISTANCE_PENALTY_AT_MAX_RANGE - 1.0);
    }
    private static double ambiguityPenalty(double avgAmbiguity) {
        double ratio = avgAmbiguity / AMBIGUITY_REJECT_THRESHOLD;
        return 1.0 + ratio * (AMBIGUITY_PENALTY_AT_THRESHOLD - 1.0);
    }


    private Optional<Measurement> drop(String reason) {
        statusPublisher.set("dropped: " + reason);
        return Optional.empty();
    }

    public double getDistanceToHub() {
        if (latestEstimate.isEmpty()
                || Timer.getFPGATimestamp() - latestEstimateTimestamp > MAX_ESTIMATE_AGE_SECONDS) {
            return 2.5; // default fallback
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