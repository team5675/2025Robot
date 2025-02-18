package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Climber extends SubsystemBase {
    private static Climber instance;
    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    SparkMax clawmotor = new SparkMax(32, MotorType.kBrushed);
    SparkMax climberMotor = new SparkMax(22, MotorType.kBrushless);
    public SparkMaxConfig climberMotorConfig;
    SparkAbsoluteEncoder angleEncoder;

    public Climber() {
        angleEncoder = climberMotor.getAbsoluteEncoder();
        climberMotorConfig = new SparkMaxConfig();
        climberMotorConfig.idleMode(IdleMode.kBrake);
        climberMotorConfig.smartCurrentLimit(15);
        climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //
    }

    public void runclimber() {
        clawmotor.set(0.5);
    }

    public void stopclimber() {
        clawmotor.set(0.0);
    }

    public void closeclaw() {
        climberMotor.setVoltage(5);
    }

    public void openclaw() {
        climberMotor.setVoltage(-5);
    }
}