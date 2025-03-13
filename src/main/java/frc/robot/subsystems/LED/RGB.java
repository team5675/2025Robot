package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color;

public class RGB {
    public int r;
    public int g;
    public int b;

    public RGB(int red, int green, int blue) {
        if (red > 255 || green > 255 || blue > 255 || red < 0 || green < 0 || blue < 0) {
            throw new IllegalArgumentException("RGB values must be between 0 and 255");
        }
        this.r = red;
        this.g = green;
        this.b = blue;
    }

    public RGB(Color color) {
        this.r = (int) (color.red * 255);
        this.g = (int) (color.green * 255);
        this.b = (int) (color.blue * 255);
    }
}
