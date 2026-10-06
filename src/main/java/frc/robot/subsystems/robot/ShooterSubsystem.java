package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
import frc.robot.util.ShooterCalculation;
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterR;
    private final TalonFX shooterL;
    EaseofLife MotorMode;
    private boolean running = false;
    public ShooterSubsystem(TalonFX shooterR, TalonFX shooterL, EaseofLife EaseOfLife) {
        this.shooterR  = shooterR;
        this.shooterL  = shooterL;
        this.MotorMode = EaseOfLife;
        configureShooter(shooterR, shooterL);
    }

    private void configureShooter(TalonFX right, TalonFX left) {
        TalonFXConfiguration sharedConfig = new TalonFXConfiguration();
        sharedConfig.Slot0.kS = 0.5;
        sharedConfig.Slot0.kV = 0.1035;
        sharedConfig.Slot0.kA = 0.003;
        sharedConfig.Slot0.kP = 0.05;
        sharedConfig.Slot0.kI = 0.0;
        sharedConfig.Slot0.kD = 0.0;

        sharedConfig.Feedback.SensorToMechanismRatio = 1.0; // direct drive

        applyCurrentLimitsAndNeutral(sharedConfig);
        applyConfig(right, sharedConfig);
        applyConfig(left, sharedConfig);
    }

    private void applyCurrentLimitsAndNeutral(TalonFXConfiguration config) {
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
    private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = null;
        boolean success = false;
        for (int attempt = 0; attempt < 5 && !success; attempt++) { // attempts to apply configs 5 times, before returning an error message
            status = motor.getConfigurator().apply(config);
            success = status.isOK();
        }
        if (!success) {
            DriverStation.reportWarning(
                "Shooter motor " + motor.getDeviceID() + " failed to configure: " + status,
                false);
        }
    }

    public void start() {
        running = true;
        MotorMode.setVelocity(shooterR, -Variables.SHOOTER_SPEED);
        MotorMode.setVelocity(shooterL, Variables.SHOOTER_SPEED);
        Variables.requestSpeedLimit("shooters", 0.1);
    }
    public void stop() {
        running = false;
        MotorMode.setVelocity(shooterR, -30);
        MotorMode.setVelocity(shooterL, 30);
        Variables.clearSpeedLimit("shooters");
    }

    public boolean isRunning() {
        return running;
    }

    public boolean atSpeed() {
        double currentR = shooterR.getVelocity().getValueAsDouble();
        double currentL = shooterL.getVelocity().getValueAsDouble();
        boolean rightAtSpeed = Math.abs(currentR - (-Variables.SHOOTER_SPEED)) < 5.0;
        boolean leftAtSpeed  = Math.abs(currentL - (Variables.SHOOTER_SPEED)) < 5.0;
        return rightAtSpeed && leftAtSpeed;
    }


    public void periodic() {
        NetworkTables.putShooterSpeed(shooterR.getVelocity().getValueAsDouble());
        NetworkTables.putShooterRRPS(shooterR.getVelocity().getValueAsDouble());
        NetworkTables.putShooterLRPS(shooterL.getVelocity().getValueAsDouble());

        if (!running) {
            return; 
        }

        if (NetworkTables.isManualShooterOverride()) {
            Variables.SHOOTER_SPEED = NetworkTables.getManualShooterSpeed();
        } else {
            Variables.SHOOTER_SPEED = ShooterCalculation.calculateShooterSpeed(MotorMode.getDistToHub());
        }
        NetworkTables.putTargetShooterSpeed(Variables.SHOOTER_SPEED);
        MotorMode.setVelocity(shooterR, -Variables.SHOOTER_SPEED);
        MotorMode.setVelocity(shooterL, Variables.SHOOTER_SPEED);
    }
}