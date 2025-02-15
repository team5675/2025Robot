package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public SparkMaxConfig ClawMotorConfig;
    private SparkClosedLoopController ClawMotorPID;
    SparkMax ClawMotor = new SparkMax(10, MotorType.kBrushed);
    SparkAbsoluteEncoder angleEncoder = ClawMotor.getAbsoluteEncoder();

    Climber() {
        
        ClawMotorConfig = new SparkMaxConfig();
         ClawMotorConfig.smartCurrentLimit(15);
        ClawMotorConfig.idleMode(IdleMode.kBrake);
        ClawMotorConfig.closedLoop
                .pid(0, 0, 0.02);
        ClawMotor.configure(ClawMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void runClaw() {
        ClawMotor.set(0.1);
    }
}
