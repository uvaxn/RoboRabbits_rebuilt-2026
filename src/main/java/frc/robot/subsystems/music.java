package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MusicLib;

/**
 * Plays .chrp songs through every swerve motor
 * Each motor plays one track of the song; see MOTOR_TRACKS and Instrument below.
 */
public class music extends SubsystemBase {
    private final Orchestra orchestra = new Orchestra();
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot");
    private final BooleanEntry playEntry = table.getBooleanTopic("Music/Play").getEntry(false);
    private final BooleanPublisher playingPub = table.getBooleanTopic("Music/IsPlaying").publish();

    private String loaded = null; // song currently playing, null if none
    private double startedAt = 0;

    public record Instrument(TalonFX motor, int track) {}

    private static final int[][] SWERVE_TRACKS = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};

    public music(CommandSwerveDrivetrain drivetrain, Instrument... extras) {
        List<Instrument> instruments = new ArrayList<>(Arrays.asList(extras));
        int i = 0;
        for (var module : drivetrain.getModules()) {
            instruments.add(new Instrument(module.getDriveMotor(), SWERVE_TRACKS[i][0]));
            instruments.add(new Instrument(module.getSteerMotor(), SWERVE_TRACKS[i][1]));
            i++;
        }
        AudioConfigs audio = new AudioConfigs();
        audio.AllowMusicDurDisable = true; // lets music play while disabled
        for (Instrument inst : instruments) {
            inst.motor().getConfigurator().apply(audio, 0.25);
            StatusCode status = orchestra.addInstrument(inst.motor(), inst.track());
            if (!status.isOK()) {
                DriverStation.reportWarning("Music: motor " + inst.motor().getDeviceID()
                    + " failed to join: " + status.getName(), false);
            }
        }

        chooser.setDefaultOption("None", "");
        MusicLib.findSongs().forEach(s -> chooser.addOption(MusicLib.name(s), s));
        SmartDashboard.putData("Music/Song", chooser);
        MusicLib.warnAboutMidi();
        playEntry.set(false); // never auto-play on boot
    }

    @Override
    public void periodic() {
        String song = chooser.getSelected();
        boolean play = playEntry.get() && !song.isEmpty() && !DriverStation.isFMSAttached();

        if (!play) {
            if (loaded != null) { orchestra.stop(); loaded = null; }
            if (playEntry.get()) playEntry.set(false); // no song or FMS attached
        } else if (!song.equals(loaded)) { // start
            orchestra.stop();
            StatusCode status = orchestra.loadMusic(MusicLib.absolutePath(song));
            if (status.isOK()) status = orchestra.play();
            if (status.isOK()) {
                loaded = song;
                startedAt = Timer.getFPGATimestamp();
            } else {
                DriverStation.reportWarning("Music: " + status.getName(), false);
                playEntry.set(false);
            }
        } else if (Timer.getFPGATimestamp() - startedAt > 0.5 && !orchestra.isPlaying()) {
            orchestra.stop(); // song finished
            loaded = null;
            playEntry.set(false);
        }
        playingPub.set(loaded != null);
    }
}