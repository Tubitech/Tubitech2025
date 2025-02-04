package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.GRIPPER_PORT1;
import static frc.robot.Constants.GripperConstants.GRIPPER_PORT2;

import com.revrobotics.spark.SparkMax;

public class GripperSubsystem extends SubsystemBase{
    private SparkMax spark1 = new SparkMax(GRIPPER_PORT1, MotorType.kBrushless);
    private SparkMax spark2 = new SparkMax(GRIPPER_PORT2, MotorType.kBrushless);
    
    public GripperSubsystem(){}

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
}
