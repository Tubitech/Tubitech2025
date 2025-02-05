package frc.robot.subsystems.climb;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ClimbConstants.CLIMB_PORT1;
import static frc.robot.Constants.ClimbConstants.CLIMB_ENCODER_CONFIG;
import static frc.robot.Constants.ClimbConstants.CLIMB_START_POSE;
import static frc.robot.Constants.ClimbConstants.CLIMB_TARGET_POSE;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ThroughBoreEncoder;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax motor = new SparkMax(CLIMB_PORT1, MotorType.kBrushless);
    
    private final ThroughBoreEncoder encoder = new ThroughBoreEncoder(CLIMB_ENCODER_CONFIG);
    private final PIDController pidController = new PIDController(0.1, 0.0, 0.01);
    private double targetRotations = 0.0;
    public ClimbSubsystem(){}

    private void setTargetRotation(double degrees) {
        targetRotations = degrees / 360.0; 
        pidController.setSetpoint(targetRotations); 
    }

    public void startPose() {
        setTargetRotation(CLIMB_START_POSE);
    }

    public void climbPose() {
        setTargetRotation(CLIMB_TARGET_POSE);
    }

    @Override
    public void periodic() {
        double currentPosition = encoder.getAbsolutePos();

        double pidOutput = pidController.calculate(currentPosition);
        motor.set(pidOutput);

        if (Math.abs(targetRotations - currentPosition) < 0.01) {
            motor.set(0); 
        }
    }

    
}
