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
    private boolean raising = false;
    private double startTimeafterSpinning = 0.5;
    private static final double RAISE_HOLD_TIME = 2.0; // wait this long after requesting up, then request down
    private final Timer jamTimer = new Timer();
    private final Timer raiseTimer = new Timer();
    private boolean shooterReady = false;

    private enum RaiseStage { UP1, DOWN }
    private RaiseStage raiseStage = RaiseStage.UP1;

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
        raising = false;
        shooterReady = false; // re-arm so periodic() re-fires this mode's entry action
    }

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
        raising = false;
        shooterReady = false;
    }

    @Override

    public void periodic() {
        if (wantIntake && intakes.isAtBottom() && !isIntakeOn) {
            intakes.start();
            isIntakeOn = true;
        }
        if (isFeeding && !shooterReady && (shooters.atSpeed() || jamTimer.hasElapsed(startTimeafterSpinning))) {
            NetworkTables.putRobotState("SHOOTER READY");
            feeds.start();
            shooterReady = true;
        }
        if (isFeeding && shooterReady && !raising) {
            raising = true;
            raiseStage = RaiseStage.UP1;
            intakes.requestUp();
            raiseTimer.restart();
            raiseTimer.start();
        }
        if (raising) {
            switch (raiseStage) {
                case UP1 -> {
                    if (raiseTimer.hasElapsed(RAISE_HOLD_TIME)) {
                        raiseStage = RaiseStage.DOWN;
                        intakes.requestDown();
                    }
                }
                case DOWN -> {
                    if (intakes.isAtBottom()) {
                        intakes.requestUp();
                        raising = false; // sequence done
                    }
                }
            }
        }
    }
}