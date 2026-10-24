package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;


public final class MotorGains {
    private MotorGains() {}


    public static final double kS = 0.5;    // volts to overcome static friction
    public static final double kV = 0.1035; // volts per (rotation/sec)
    public static final double kA = 0.003;  // volts per (rotation/sec^2)
    public static final double kP = 0.05;   // volts per (rotation/sec) of error
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double SHOOTER_GEAR_RATIO = 1.0; // direct drive
    public static final double FEED_GEAR_RATIO    = 1.0; 
    public static final double INTAKE_GEAR_RATIO  = 1.0; 

    public static final double SHOOTER_STATOR_LIMIT       = 60.0;
    public static final double SHOOTER_SUPPLY_LIMIT       = 40.0;
    public static final double SHOOTER_SUPPLY_LOWER_LIMIT = 40.0; // supply limit drops to this...
    public static final double SHOOTER_SUPPLY_LOWER_TIME  = 1.0;  // ...after sitting over the limit this many seconds

    public static final double FEED_STATOR_LIMIT = 60.0;
    public static final double FEED_SUPPLY_LIMIT = 40.0;

    public static final double INTAKE_STATOR_LIMIT = 50.0;
    public static final double INTAKE_SUPPLY_LIMIT = 30.0;

    public static void configureShooter(TalonFX right, TalonFX left) {
        TalonFXConfiguration config = baseConfig(
            SHOOTER_GEAR_RATIO, SHOOTER_STATOR_LIMIT, SHOOTER_SUPPLY_LIMIT);
        config.CurrentLimits.SupplyCurrentLowerLimit = SHOOTER_SUPPLY_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime  = SHOOTER_SUPPLY_LOWER_TIME;

        applyConfig(right, config);
        applyConfig(left, config);
    }

    /** Lower and upper feed rollers. */
    public static void configureFeed(TalonFX lower, TalonFX upper) {
        TalonFXConfiguration config = baseConfig(
            FEED_GEAR_RATIO, FEED_STATOR_LIMIT, FEED_SUPPLY_LIMIT);

        applyConfig(lower, config);
        applyConfig(upper, config);
    }

    /** The regular intake roller */
    public static void configureIntake(TalonFX intake) {
        TalonFXConfiguration config = baseConfig(
            INTAKE_GEAR_RATIO, INTAKE_STATOR_LIMIT, INTAKE_SUPPLY_LIMIT);

        applyConfig(intake, config);
    }


    /** Fresh config with the shared gains */
    private static TalonFXConfiguration baseConfig(double gearRatio, double statorLimit, double supplyLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration(); 

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;

        config.Feedback.SensorToMechanismRatio = gearRatio;

        config.CurrentLimits.StatorCurrentLimit        = statorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable  = true;
        config.CurrentLimits.SupplyCurrentLimit        = supplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable  = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        return config;
    }

    /** Applies a config, retrying a few times because the CAN bus can be flaky right at boott */
    private static void applyConfig(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 0; attempt < 5; attempt++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            DriverStation.reportWarning(
                "Motor " + motor.getDeviceID() + " failed to configure: " + status, false);
        }
    }
}
