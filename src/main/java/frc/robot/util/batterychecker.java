
package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class batterychecker {

    private static final double LOW_BATTERY_VOLTAGE = 12.5;
    private static final double CRITICAL_BATTERY_VOLTAGE = 12.2;

    private boolean lowBatterySent = false;
    private boolean criticalBatterySent = false;

    public void check() {
        double batteryVoltage = RobotController.getBatteryVoltage();

        boolean critical = batteryVoltage < CRITICAL_BATTERY_VOLTAGE;
        boolean low = !critical && batteryVoltage < LOW_BATTERY_VOLTAGE;

        // Critical battery notification
        if (critical && !criticalBatterySent) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.ERROR,
                    "CRITICAL BATTERY VOLTAGE",
                    String.format(
                        "CHANGE THE BATTERY NOW! Voltage: %.2f V",
                        batteryVoltage
                    )
                )
            );

            criticalBatterySent = true;
            lowBatterySent = false;
        }

        // Low battery notification
        else if (low && !lowBatterySent) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "LOW BATTERY VOLTAGE",
                    String.format(
                        "Be careful with this battery. Voltage: %.2f V",
                        batteryVoltage
                    )
                )
            );

            lowBatterySent = true;
        }

        // Reset notification flags once voltage recovers
        else if (!low && !critical) {
            lowBatterySent = false;
            criticalBatterySent = false;
        }
    }
}
