package frc.robot.util;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

//Finds .chrp songs in the deploy folder
public final class MusicLib {
    private MusicLib() {}
    public static List<String> findSongs() { return scan(".chrp"); }

    public static String absolutePath(String song) {
        return new File(Filesystem.getDeployDirectory(), song).getAbsolutePath();
    }

    public static String name(String song) { return song.substring(0, song.lastIndexOf('.')); }

    public static void warnAboutMidi() {
        List<String> songs = findSongs().stream().map(MusicLib::name).toList();
        for (String midi : scan(".mid")) {
            if (!songs.contains(name(midi))) {
                DriverStation.reportWarning("Music: " + midi + " needs converting to .chrp in Tuner X", false);
            }
        }
    }
    private static List<String> scan(String ext) {
        Path root = Filesystem.getDeployDirectory().toPath();
        try (Stream<Path> walk = Files.walk(root, 3)) {
            return walk.filter(p -> p.toString().toLowerCase().endsWith(ext))
                .map(p -> root.relativize(p).toString().replace(File.separatorChar, '/'))
                .sorted(String.CASE_INSENSITIVE_ORDER)
                .toList();
        } catch (IOException e) {
            return List.of();
        }
    }
}