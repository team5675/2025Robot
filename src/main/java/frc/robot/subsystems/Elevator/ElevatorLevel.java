package frc.robot.subsystems.Elevator;

public enum ElevatorLevel {
    L4_HEIGHT("L4_HEIGHT", 91.5),
    L3_HEIGHT("L3_HEIGHT", 48),
    L2_HEIGHT("L2_HEIGHT", 19),
    L1_HEIGHT("L1_HEIGHT", 4),
    RESET_HEIGHT("RESET_HEIGHT", 0),
    ALGAE_LOW_HEIGHT("ALGAE_LOW_HEIGHT", 48),
    ALGAE_HIGH_HEIGHT("ALGAE_HIGH_HEIGHT", 70);

    private final String name;
    private final double value;

    ElevatorLevel(String name, double value) {
        this.name = name;
        this.value = value;
    }

    public String getName() {
        return name;
    }

    public double getLevel() {
        return value;
    }
}