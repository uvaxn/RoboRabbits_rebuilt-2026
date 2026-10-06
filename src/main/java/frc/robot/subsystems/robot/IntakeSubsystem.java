package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.DriveInputs;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;
    private final DigitalInput upperSensor;
    private final DigitalInput lowerSensor;
    EaseofLife MotorMode;
    private static final double DROP_SPEED = 0.15;
    private static final double LIFT_SPEED = 0.15;

    private static final double INTAKE_COLLECT_SPEED = -0.8; // collecting from ground
    private static final double INTAKE_FEED_SPEED = -0.65; // for pushing balls to shooter
    private final CoastOut    coastOut    = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP }

    private DropState state = DropState.IDLE;
    // what "Seeded" means in these variables is basically has it set off the top sensor (it stores that) 
    private boolean hasSeededTop    = true; // the nail is somewhat bent at the top, so it wont set off the top sensor too well. Best to leave this true.
    // same thing except for it's the bottom sensor
    private boolean hasSeededBottom = false;



    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
                        DigitalInput upperSensor, DigitalInput lowerSensor, EaseofLife EaseOfLife, DriveInputs DriveInputs) {
        this.intakeMotor = intakeMotor;
        this.dropMotor   = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;
        this.MotorMode = EaseOfLife;     
        applyCurrentLimits(intakeMotor, "Intake", 70.0, 55.0, 25.0, 1.0);
        applyCurrentLimits(dropMotor, "Intake drop", 50.0, 40.0, 10.0, 1.0);
        dropMotor.setPosition(0.0);
        dropMotor.setControl(staticBrake);
    }
    private void applyCurrentLimits(TalonFX motor, String label, double statorAmps, double supplyAmps,
                                    double supplyLowerAmps, double supplyLowerTimeSec) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = statorAmps;
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = supplyAmps;
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLowerLimit = supplyLowerAmps;
        limits.SupplyCurrentLowerTime = supplyLowerTimeSec;

        StatusCode status = null;
        boolean success = false;
        for (int attempt = 0; attempt < 5 && !success; attempt++) {
            status = motor.getConfigurator().apply(limits);
            success = status.isOK();
        }
        if (!success) {
            DriverStation.reportWarning(
                label + " motor " + motor.getDeviceID() + " failed to apply current limits: " + status,
                false);
        }
    }

    public void requestDown() {
        if (isAtBottom() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;

    }
    public void requestUp() {
        if (isAtTop() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_UP;

    }
    /** Spins the intake roller to push balls up toward the feed rollers. */
    public void startFeed() {
        MotorMode.setSpeed(intakeMotor, INTAKE_FEED_SPEED);
    }

    public void start() {
        MotorMode.setSpeed(intakeMotor, INTAKE_COLLECT_SPEED);
        NetworkTables.putRobotState("INTAKE");
    }
    public void stop() {
        MotorMode.setSpeed(intakeMotor, 0);
        NetworkTables.putRobotState("STOPPED INTAKE");
    }
 

    public boolean isIdle() {
        return state == DropState.IDLE;
    }

    public boolean isCollecting() {
        return state == DropState.IDLE && isAtBottom();
    }

    public boolean isFullyUp() {
        return state == DropState.IDLE && isAtTop();
    }
    /** @return true when the lower sensor has been set off. */
    public boolean isAtBottom() { return !lowerSensor.get(); }
    /** @return true when the upper sensor has been set off. */
    public boolean isAtTop()    { return !upperSensor.get(); }

    @Override
    public void periodic() {
        NetworkTables.putIntakeRPS(intakeMotor.getVelocity().getValueAsDouble());
        // position is in motor rotations (zeroed at startup), x360 = degrees
        NetworkTables.putIntakeDropDeg(dropMotor.getPosition().getValueAsDouble() * 360.0);

        if (isAtTop() && !hasSeededTop) {
            hasSeededTop    = true;
            hasSeededBottom = false;
        }
        if (isAtBottom() && !hasSeededBottom) {
            hasSeededBottom = true;
            hasSeededTop    = false;
        }
        
        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom()) {
                    dropMotor.setControl(coastOut);
                    state = DropState.IDLE;
                } else {
                    dropMotor.set(-DROP_SPEED);
                }
            }

            case MOVING_UP -> {
                if (isAtTop()) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE;
                } else {
                    dropMotor.set(LIFT_SPEED);
                }
            }

            case IDLE -> {}
        }
    }
}