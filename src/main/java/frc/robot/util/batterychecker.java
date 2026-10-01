package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;

public class batterychecker {

    private static final double LOW_BATTERY_VOLTAGE = 12.5;
    private static final double CRITICAL_BATTERY_VOLTAGE = 12.2;

private final Alert lowBattery =
        new Alert("be careful with this battery voltage.", AlertType.kWarning);
    private final Alert criticalBattery =
            new Alert("CHANGE THE BATTERY NOW!", AlertType.kError);

    public void check() {
         double batteryVoltage = RobotController.getBatteryVoltage();

        boolean critical = batteryVoltage < CRITICAL_BATTERY_VOLTAGE;
        boolean low = !critical && batteryVoltage < LOW_BATTERY_VOLTAGE;

        criticalBattery.set(critical);
        lowBattery.set(low);
    }
}