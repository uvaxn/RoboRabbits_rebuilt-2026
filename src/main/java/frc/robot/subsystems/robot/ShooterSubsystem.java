package frc.robot.subsystems.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.MotorGains;
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
        MotorGains.configureShooter(shooterR, shooterL);
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