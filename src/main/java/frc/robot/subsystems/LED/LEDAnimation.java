package frc.robot.subsystems.LED;

public interface LEDAnimation {
    /**
     * Initialize the animation with the LED subsystem.
     * This is called when the animation is set as the current animation.
     * 
     * @param ledSubsystem The LED subsystem
     */
    void init(LED ledSubsystem);
    
    /**
     * Execute one frame of the animation.
     * This is called periodically from the LED subsystem's periodic method.
     */
    void execute();
    
    /**
     * End the animation.
     * This is called when another animation is being set.
     * Use this to clean up any resources or reset the LED strip if needed.
     */
    void end();
}