package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {
    
    public DigitalInput limitSwitch;
    public Trigger isTripped;
    public RelativeEncoder climberEncoder;

    public SparkClosedLoopController climberPID;
    public SparkMaxConfig clawMotorConfig; //Johnson motor to open-close claw
    public SparkMaxConfig climberMotorConfig; //NEO motor to climb and unclimb
    
    public SparkMax clawMotor;
    public SparkMax climberMotor;
    
    public Climber() {
        clawMotor = new SparkMax(ClimberConstants.clawMotorID, MotorType.kBrushed);
        
        clawMotorConfig = new SparkMaxConfig();
        clawMotorConfig.idleMode(IdleMode.kBrake);
        clawMotorConfig.smartCurrentLimit(ClimberConstants.voltsStallLimit);
        
        climberMotor = new SparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        climberPID = climberMotor.getClosedLoopController();
        climberMotorConfig = new SparkMaxConfig();
        climberMotorConfig.idleMode(IdleMode.kBrake);
        climberMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ClimberConstants.climberMotorP, ClimberConstants.climberMotorI, ClimberConstants.climberMotorD);
        climberMotorConfig.smartCurrentLimit(ClimberConstants.voltsStallLimit);
        climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        

        limitSwitch = new DigitalInput(ClimberConstants.limitSwitchChannel);
        isTripped = new Trigger(limitSwitch::get);
    }

    public void SetTarget(double ticks) {
        climberPID.setReference(ticks, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        SmartDashboard.putBoolean("ClimberLimitSwitch" , isTripped.getAsBoolean());
        SmartDashboard.putNumber("climber ticks", climberEncoder.getPosition());
    }

    private static Climber instance;
    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

}