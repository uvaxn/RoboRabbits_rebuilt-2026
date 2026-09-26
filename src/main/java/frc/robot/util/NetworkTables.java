package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.Variables;

public class NetworkTables {
    private static final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private static final NetworkTable rbtTable = ntInst.getTable("Robot");
    private static final NetworkTable infoTable = ntInst.getTable("Info");

    // robot
    private static final StringPublisher robotState = rbtTable.getStringTopic("State").publish();
    // autoalign
    private static final DoublePublisher targetAngle = rbtTable.getDoubleTopic("AutoAlign/TargetAngleDeg").publish();

    // ease of life
    private static final StringPublisher currentShift = infoTable.getStringTopic("EaseofLife/CurrentShift").publish();
    private static final DoublePublisher shiftTimeRemaining = infoTable.getDoubleTopic("EaseofLife/ShiftTimeRemaining").publish();
    private static final BooleanPublisher isHubActive = infoTable.getBooleanTopic("EaseofLife/IsHubActive").publish();
    private static final DoublePublisher distToHubPublisher = infoTable.getDoubleTopic("EaseofLife/DistanceToHub").publish();
    private static final DoublePublisher shooterSpeed = infoTable.getDoubleTopic("EaseofLife/ShooterSpeed").publish();
    private static final DoublePublisher shooterActualSpeed = infoTable.getDoubleTopic("EaseofLife/ShooterActualSpeedRPS").publish();
    // PID publishers
    private static final DoublePublisher AligntoHubP = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubP").publish();
    private static final DoublePublisher AligntoHubI = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubI").publish();
    private static final DoublePublisher AligntoHubD = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubD").publish();

    // PID subscribers
    private static final DoubleSubscriber alignPSub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubP").subscribe(Variables.AlignToHubP);
    private static final DoubleSubscriber alignISub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubI").subscribe(Variables.AlignToHubI);
    private static final DoubleSubscriber alignDSub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubD").subscribe(Variables.AlignToHubD);
    private static final DoublePublisher AligntoWallP = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToWallP").publish();
    private static final DoublePublisher AligntoWallI = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToWallI").publish();
    private static final DoublePublisher AligntoWallD = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToWallD").publish();
    private static final DoubleSubscriber alignWallPSub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToWallP").subscribe(Variables.AlignToAllianceWallP);
    private static final DoubleSubscriber alignWallISub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToWallI").subscribe(Variables.AlignToAllianceWallI);
    private static final DoubleSubscriber alignWallDSub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToWallD").subscribe(Variables.AlignToAllianceWallD);
    private static final DoublePublisher shooterSpeedSetpointPub =
        rbtTable.getDoubleTopic("EaseofLife/ManualShooterSpeedSetpoint").publish();
    private static final DoubleSubscriber shooterSpeedSetpointSub =
        rbtTable.getDoubleTopic("EaseofLife/ManualShooterSpeedSetpoint").subscribe(Variables.SHOOTER_SPEED);

    private static final BooleanPublisher manualShooterOverridePub =
        rbtTable.getBooleanTopic("EaseofLife/ManualShooterOverride").publish();
    private static final BooleanSubscriber manualShooterOverrideSub =
        rbtTable.getBooleanTopic("EaseofLife/ManualShooterOverride").subscribe(false);

    static {
        shooterSpeedSetpointPub.set(Variables.SHOOTER_SPEED);
        manualShooterOverridePub.set(false);
    }
    public static void putTargetAngle(double deg)   { targetAngle.set(deg); }
    public static void putCurrentShift(String shift){ currentShift.set(shift); }
    public static void putShiftTime(double time)    { shiftTimeRemaining.set(time); }
    public static void isHubActive(boolean active)  { isHubActive.set(active); }
    public static void putRobotState(String state)  { robotState.set(state); }
    public static void putDisttoHub(double dist)    { distToHubPublisher.set(dist); }
    public static void putTargetShooterSpeed(double speed){ shooterSpeed.set(speed); }
    public static void putShooterSpeed(double rps){ shooterActualSpeed.set(rps); }
    // PID setters
    public static void putAlignP(double P) { AligntoHubP.set(P); }
    public static void putAlignI(double I) { AligntoHubI.set(I); }
    public static void putAlignD(double D) { AligntoHubD.set(D); }

    public static void putAlignWallP(double P) { AligntoWallP.set(P); }
    public static void putAlignWallI(double I) { AligntoWallI.set(I); }
    public static void putAlignWallD(double D) { AligntoWallD.set(D); }



    // PID getters (reads live value from dashboard)
    public static double getAlignP() { return alignPSub.get(); }
    public static double getAlignI() { return alignISub.get(); }
    public static double getAlignD() { return alignDSub.get(); }

    public static double getAlignWallP() { return alignWallPSub.get(); }
    public static double getAlignWallI() { return alignWallISub.get(); }
    public static double getAlignWallD() { return alignWallDSub.get(); }
    
    public static double getManualShooterSpeed() { return shooterSpeedSetpointSub.get(); }
    public static boolean isManualShooterOverride() { return manualShooterOverrideSub.get(); }
}