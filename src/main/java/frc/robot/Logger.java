package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class Logger extends DogLog {
    public static void logString(String key, String value) {
        log(key, value);
    }
}