package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.ClimbConstants.CLIMB_PORT1;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.ClimbConstants.CLIMB_ENCODER_CONFIG;
import static frc.robot.Constants.ClimbConstants.CLIMB_START_POSE;
import static frc.robot.Constants.ClimbConstants.CLIMB_TARGET_POSE;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    private TalonFX motor = new TalonFX(5, "Canivore"); // TODO
    
    private final PIDController pidController = new PIDController(0.1, 0.0, 0.01);
    private double targetRadians = 0.0;
    public ClimbSubsystem(){
    }

    private void setTargetRotation(double degrees) {
        targetRadians = degrees / 180 * Math.PI; 
        pidController.setSetpoint(targetRadians); 
    }

    public double getAngleInRadians() {
        return motor.getPosition().getValue().in(Radians);
    }

    public boolean inPosition() {
        return Math.abs(targetRadians - getAngleInRadians()) < 0.01;
    }

    public void startPose() {
        setTargetRotation(CLIMB_START_POSE);
    }

    public void climbPose() {
        setTargetRotation(CLIMB_TARGET_POSE);
    }

    @Override
    public void periodic() {
        double currentPosition = getAngleInRadians();

        double pidOutput = pidController.calculate(currentPosition);

        if (inPosition()) {
            motor.set(0); 
        } else {
            motor.set(pidOutput);
        }
    }

    
}