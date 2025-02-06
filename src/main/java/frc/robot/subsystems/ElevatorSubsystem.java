package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    private final TrapezoidProfile.Constraints elevatoConstraints = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController controllerElevator = new ProfiledPIDController(1, 1, 1, elevatoConstraints, 1);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(1, 1, 1,1,1);
    private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);

    double targetH = 0;
    private void setTargetH(double h){
        targetH = h;
    }
    public void setElevator(double h){
        setTargetH(h);
    }
    private double getElevatorPosition(){
        return elevatorEncoder.get();
    }
    public ElevatorSubsystem(){

    }
    @Override
    public void periodic(){
        setElevatorPosition(targetH);
    }
    private void setElevatorPositionPeriodic(double height){
        controllerElevator.setGoal(height);
        setElevatorMotor();
    }
    private void setElevatorMotor(){
        elevatorMotor.setVoltage(controllerElevator.calculate(getElevatorPosition())+elevatorFeedforward.calculate(controllerElevator.getSetpoint().velocity));
    }
    public boolean isInPosition(){
        return (controllerElevator.atGoal());
    }
    public Command setElevatorPosition(double targetH){
        return runOnce(()-> setElevator(targetH));
    }
    public Command WaitUntilElevatorFinishes(){
        return new WaitUntilCommand(()->isInPosition());
    }
    public Command SetElevAndWaitUntilFinishes(double targetH){
        return new ParallelCommandGroup(setElevatorPosition(targetH),WaitUntilElevatorFinishes());
    }
}
