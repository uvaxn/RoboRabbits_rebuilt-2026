package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.robot.FeedSubsystem;
import frc.robot.subsystems.robot.IntakeSubsystem;
import frc.robot.subsystems.robot.ShooterSubsystem;
import frc.robot.util.NetworkTables;

public class Mechanisms extends SubsystemBase {
    private ShooterSubsystem shooters;
    private IntakeSubsystem intakes;
    private FeedSubsystem feeds;
    private boolean wantIntake = false;
    private boolean isIntakeOn = false;
    private boolean isFeeding = false;
    private boolean armRaised = false;
    private double startTimeafterSpinning = 0.6;
    private static final double FEED_RAISE_TIME = 2.0; // seconds to hold the intake up before feeding fully starts
    private final Timer jamTimer = new Timer();
    private final Timer feedTimer = new Timer();
    private boolean shooterReady = false;

    public Mechanisms(ShooterSubsystem ShooterSubsystem, IntakeSubsystem IntakeSubsystem, FeedSubsystem FeedSubsystem) {
        this.shooters = ShooterSubsystem;
        this.intakes = IntakeSubsystem;
        this.feeds = FeedSubsystem;
    }
    public void StartShooting() {
        NetworkTables.putRobotState("SPINNING UP");
        shooterReady = false;
        shooters.start();
        jamTimer.restart();
        jamTimer.start();
    }

    public void intakeFeed() {
        isFeeding = true;
        armRaised = false;
        shooterReady = false; // re-arm so periodic() re-fires this mode's entry action
        intakes.requestUp();
        feedTimer.restart();
        feedTimer.start();
    }

    /** spin up and go straight to intake-feed mode, purely for auto. */
    public void intakeFeedShoot() {
        StartShooting();
        intakeFeed();
     }

    public void Intake() {
        NetworkTables.putRobotState("INTAKE");
        wantIntake = true;  
        intakes.requestDown();
    }

    public void StopIntake() {
        NetworkTables.putRobotState("STOPPED INTAKE");
        wantIntake = false;
        intakes.stop();
        isIntakeOn = false;
    }

    public void StopShoot() {
        NetworkTables.putRobotState("STOPPED FIRING");
        shooters.stop();
        feeds.stop();
        isFeeding = false;
        armRaised = false;
        shooterReady = false;
    }

    @Override

    public void periodic() {
        if (wantIntake && intakes.isAtBottom() && !isIntakeOn) {
            intakes.start();
            isIntakeOn = true;
        }
        if (isFeeding && !armRaised && feedTimer.hasElapsed(FEED_RAISE_TIME)) {
            armRaised = true;
        }
        if (isFeeding && armRaised && (shooters.atSpeed() || jamTimer.hasElapsed(startTimeafterSpinning)) && !shooterReady) {
            NetworkTables.putRobotState("SHOOTER READY");
            feeds.start();
            shooterReady = true;
        }
    }
}
