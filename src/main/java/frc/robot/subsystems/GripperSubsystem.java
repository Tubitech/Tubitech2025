package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import com.revrobotics.spark.SparkMax;

public class GripperSubsystem extends SubsystemBase{
    private SparkMax spark1 = new SparkMax(12, MotorType.kBrushless);
    private SparkMax spark2 = new SparkMax(14, MotorType.kBrushless);

    private Ultrasonic sensor = new Ultrasonic(null, null); // TODO

    private RobotState robotState;
    public GripperSubsystem(RobotState robotState){
        this.robotState = robotState;
    }

    public void pull() {
        spark1.set(1);
        spark2.set(1);
    }
    public void push(){
        spark1.set(-1);
        spark2.set(-1);
    }
    public void hold(){
        spark1.set(0.1);
        spark2.set(0.1);
    }
    public void stop() {
        spark1.set(0);
        spark2.set(0);
    }
    @Override
    public void periodic(){
        
    }
    public boolean isSensorDetected(){
        return sensor.getRangeInches() > 0 && sensor.getRangeInches() < 1;
    }
    public Command pullCoral(String direction){
        return startEnd(() -> pull(), () -> { hold(); robotState.isCoralIn = true; robotState.coralDirection = direction; }).until(() -> isSensorDetected()).withTimeout(1);
    }
    public Command pushCoral(){
        return startEnd(() -> push(), () -> { stop(); robotState.isCoralIn = false; }).until(() -> !isSensorDetected());
    }
    public Command pullAlg(){
        return startEnd(() -> pull(), () -> { hold(); robotState.isAlgIn = true; }).until(() -> isSensorDetected()).withTimeout(1);
    }
    public Command pushAlg(){
        return startEnd(() -> push(), () -> { stop(); robotState.isAlgIn = false; }).until(() -> !isSensorDetected());
    }
    
}