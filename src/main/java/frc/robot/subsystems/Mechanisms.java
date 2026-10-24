
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.robot.FeedSubsystem;
import frc.robot.subsystems.robot.IntakeSubsystem;
import frc.robot.subsystems.robot.ShooterSubsystem;
import frc.robot.util.NetworkTables;

public class Mechanisms extends SubsystemBase {
    private final ShooterSubsystem shooters;
    private final IntakeSubsystem intakes;
    private final FeedSubsystem feeds;

    private boolean wantIntake = false;
    private boolean isIntakeOn = false;
    private boolean wantShoot = false;
    private boolean shooterReady = false;
    private final Timer spinupTimer = new Timer();
    private static final double MAX_SPINUP_TIME = 1.0;
    // After feeding starts, wait this long before the intake arm starts agitating.
    private static final double AGITATE_DELAY_SECONDS = 1.0;
    private final Timer agitateDelayTimer = new Timer();
    private boolean agitatePending = false;
    public Mechanisms(
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            FeedSubsystem feedSubsystem) {
        this.shooters = shooterSubsystem;
        this.intakes = intakeSubsystem;
        this.feeds = feedSubsystem;
    }
    public void StartShooting() {
        if (wantShoot) {
            return;
        }
        wantShoot = true;
        shooterReady = false;
        NetworkTables.putRobotState("SPINNING UP");
        shooters.start();
        spinupTimer.restart();
    }

    public void Intake() {
        NetworkTables.putRobotState("INTAKE");
        wantIntake = true;
        intakes.requestDown();
    }

    public void StopIntake() {
        NetworkTables.putRobotState("STOPPED INTAKE");
        wantIntake = false;
        isIntakeOn = false;
        intakes.stop();
    }

    public void StopShoot() {
        wantShoot = false;
        shooterReady = false;

        NetworkTables.putRobotState("STOPPED FIRING");

        spinupTimer.stop();
        agitatePending = false;
        agitateDelayTimer.stop();
        agitateDelayTimer.reset();
        feeds.stop();
        shooters.stop();
        intakes.stopAgitate();
    }

    @Override
    public void periodic() {
        if (wantIntake && intakes.isAtBottom() && !isIntakeOn) {
            intakes.start();
            isIntakeOn = true;
        }
        if (wantShoot && !shooterReady
                && (shooters.atSpeed()
                    || spinupTimer.hasElapsed(MAX_SPINUP_TIME))) {
            NetworkTables.putRobotState("FIRING");
            feeds.start();
            agitateDelayTimer.restart();
            agitatePending = true;

            shooterReady = true;
            spinupTimer.stop();
        }
        if (wantShoot && agitatePending
                && agitateDelayTimer.hasElapsed(AGITATE_DELAY_SECONDS)) {
            intakes.startAgitate();
            agitatePending = false;
            agitateDelayTimer.stop();
        }
    }
}
