package frc.robot;
import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.util.batterychecker;
import frc.robot.vision.LimelightHelpers;
import com.ctre.phoenix6.SignalLogger;
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer robotContainer;
    private batterychecker battery = new batterychecker();
    private static final String LL_NAME = Constants.LL_NAME;
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        battery.check(); // man i wonder what this does
    }
    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();


        CommandScheduler.getInstance().run();

        String mode = DriverStation.isDisabled() ? "Disabled"
            : DriverStation.isAutonomous() ? "Autonomous"
            : DriverStation.isTeleop() ? "Teleop"
            : "Test";
        SignalLogger.writeString("Robot/Mode", mode);
    }
    @Override
    public void autonomousInit() {
        m_autonomousCommand = robotContainer.getAutonomousCommand();
        SignalLogger.writeString("Robot/SelectedAuto",
            m_autonomousCommand != null ? m_autonomousCommand.getName() : "None");
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        LimelightHelpers.SetIMUMode(LL_NAME, 4); // use the ll4 imu, and SWERVE gyro
    }
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        robotContainer.easeOfLife.teleopInit();
        robotContainer.mechanisms.StopShoot();
        LimelightHelpers.SetIMUMode(LL_NAME, 4); // use the ll4 imu, and swerves gyro.
    }
    @Override public void disabledInit() {
        LimelightHelpers.SetIMUMode(LL_NAME, 1); // re-seed 
    }
    @Override public void disabledPeriodic() {}
    @Override public void disabledExit() {}
    @Override public void autonomousPeriodic() {
        addVision();
    }
    @Override public void autonomousExit() {}
    @Override public void teleopPeriodic() {
        addVision();
    }
    @Override public void teleopExit() {}
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testPeriodic() {}
    @Override public void testExit() {}
    @Override public void simulationInit() {
    }
    @Override public void simulationPeriodic() {}
    private void addVision() {
        robotContainer.cameraSubsystem.getMeasurement(
            robotContainer.drivetrain.getState().Pose
        ).ifPresent(measurement ->
            robotContainer.drivetrain.addVisionMeasurement(
                measurement.pose,
                measurement.timestamp,
                measurement.standardDeviations
            )
        );
    }
}