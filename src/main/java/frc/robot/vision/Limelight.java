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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private static final double FIELD_LENGTH_M = FIELD.getFieldLength();
    private static final double FIELD_WIDTH_M = FIELD.getFieldWidth();
    private static final double STD_DEV_BASE_1_TAG = 2.5;       // barely trusted
    private static final double STD_DEV_BASE_2_TAG = 0.7;       // moderately trusted
    private static final double STD_DEV_BASE_3_PLUS_TAG = 0.1;  // strongly trusted
    private CommandSwerveDrivetrain drivetrain;

    private static final double ROTATION_STD_DEV = 9999.9;

    private final String name;
    private final StructPublisher<Pose2d> posePublisher;
        public Limelight(String name, CommandSwerveDrivetrain drivetrain) {
            this.name = name;
            this.drivetrain = drivetrain;
        NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(name);
        this.posePublisher = telemetryTable.getStructTopic("EstimatedPose", Pose2d.struct).publish();
    }
    private static boolean isInField(Translation2d t) {
        return t.getX() >= 0 && t.getX() <= FIELD_LENGTH_M
                && t.getY() >= 0 && t.getY() <= FIELD_WIDTH_M;
    }
    private static double baseStdDevForTagCount(int tagCount) {
        if (tagCount >= 3) return STD_DEV_BASE_3_PLUS_TAG;
        if (tagCount == 2) return STD_DEV_BASE_2_TAG;
        return STD_DEV_BASE_1_TAG;
    }

    /**
     * @param currentRobotPose current pose estimate.
     */
    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        final PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt2 == null || mt2.tagCount == 0) {
            return Optional.empty();
        }

        final Translation2d mt2Translation = mt2.pose.getTranslation();
        if (!isInField(mt2Translation)) {
            DriverStation.reportWarning("Limelight pose out of bounds", false);
            return Optional.empty();
        }
        // mt2 supplies the translation and timestamp, so its tag count is the one that matters.
        final double xyStdDev = baseStdDevForTagCount(mt2.tagCount);
        final Matrix<N3, N1> standardDeviations =
                VecBuilder.fill(xyStdDev, xyStdDev, ROTATION_STD_DEV);

        posePublisher.set(mt2.pose);

        return Optional.of(new Measurement(mt2.pose, mt2.timestampSeconds, standardDeviations));
    }
    public double getDistToHub() {
        return drivetrain.getState().Pose.getTranslation()
                .getDistance(Landmarks.getTeamHubTranslation());
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