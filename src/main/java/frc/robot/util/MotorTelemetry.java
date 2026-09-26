package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;

public class MotorTelemetry {
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Angle> position;

    private final DoublePublisher tempPub;
    private final DoublePublisher rpsPub;
    private final DoublePublisher degreesPub;

    /**
     * @param path  NT sub-path under the "Robot" table, e.g. {@code "Intake/Roller"} ->
     *              {@code robot/Intake/Roller/TempC}, {@code .../Rps}, {@code .../Degrees}.
     * @param motor The TalonFX to track.
     */
    public MotorTelemetry(String path, TalonFX motor) {
        this.temp = motor.getDeviceTemp();
        this.velocity = motor.getVelocity();
        this.position = motor.getPosition();

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot");
        this.tempPub = table.getDoubleTopic(path + "/TempC").publish();
        this.rpsPub = table.getDoubleTopic(path + "/Rps").publish();
        this.degreesPub = table.getDoubleTopic(path + "/Degrees").publish();
    }

    /** Refreshes this motor's signals and publishes them to NetworkTables. Call once per periodic(). */
    public void publish() {
        BaseStatusSignal.refreshAll(temp, velocity, position);
        tempPub.set(getTempC());
        rpsPub.set(getRps());
        degreesPub.set(getDegrees());
    }

    /** @return the temperature (Celsius) read at the last {@link #publish()} call. */
    public double getTempC() { return temp.getValueAsDouble(); }

    /** @return the velocity (rotations per second) read at the last {@link #publish()} call. */
    public double getRps() { return velocity.getValueAsDouble(); }

    /** @return the position (degrees) read at the last {@link #publish()} call. */
    public double getDegrees() { return position.getValueAsDouble() * 360.0; }
}