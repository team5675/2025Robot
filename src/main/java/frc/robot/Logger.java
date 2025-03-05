package frc.robot;

import dev.doglog.DogLog;

public class Logger extends DogLog {
    public static void logString(String key, String value) {
        log(key, value);
    }
}