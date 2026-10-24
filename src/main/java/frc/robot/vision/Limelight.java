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
    // Citrus Circuits style: std-dev = (meters of noise per meter of tag distance) * avg tag distance.
    private static final double AUTO_STD_PER_METER   = 0.3; // 1678's value, used in auto/disabled
    private static final double TELEOP_STD_PER_METER = 0.2; // lower = trust vision more in teleop
    private static final double MIN_TAG_DIST_M       = 1.0; // floor so very close tags don't give ~0 std-dev
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
        // Trust grows as the tags get closer; teleop uses a smaller per-meter value so vision pulls harder.
        final double stdPerMeter = DriverStation.isTeleopEnabled()
                ? TELEOP_STD_PER_METER : AUTO_STD_PER_METER;
        final double xyStdDev = stdPerMeter * Math.max(mt2.avgTagDist, MIN_TAG_DIST_M);
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