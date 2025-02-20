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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Climber extends SubsystemBase {
    private static Climber instance;
    public static Climber getInstance;
    public DigitalInput LimitSwitch;
    public Trigger lsTripped;
    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public SparkMaxConfig clawmotorConfig;
    public SparkMaxConfig climberMotorConfig;
    public SparkMax clawmotor = new SparkMax(32, MotorType.kBrushed);
    public SparkMax ClimberMotor = new SparkMax(22, MotorType.kBrushless);
    public Climber() {
        
        clawmotorConfig = new SparkMaxConfig();
        clawmotorConfig.idleMode(IdleMode.kBrake);
        clawmotorConfig.smartCurrentLimit(15);
        climberMotorConfig = new SparkMaxConfig();
        climberMotorConfig.idleMode(IdleMode.kBrake);
        climberMotorConfig.smartCurrentLimit(15);
        ClimberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //
        LimitSwitch = new DigitalInput(2);
        lsTripped = new Trigger(LimitSwitch::get);
    }
}