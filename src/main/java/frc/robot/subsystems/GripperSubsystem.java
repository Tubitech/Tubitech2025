package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import static frc.robot.Constants.GripperConstants.GRIPPER_PORT1;
import static frc.robot.Constants.GripperConstants.GRIPPER_PORT2;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class GripperSubsystem extends SubsystemBase{
    private SparkMax spark1 = new SparkMax(GRIPPER_PORT1, MotorType.kBrushless);
    private SparkMax spark2 = new SparkMax(GRIPPER_PORT2, MotorType.kBrushless);

    // New hold func
    private RelativeEncoder encoder1 = spark1.getEncoder();
    private RelativeEncoder encoder2 = spark2.getEncoder();
    private PIDController pidController1 = new PIDController(0.1, 0, 0.01);
    private PIDController pidController2 = new PIDController(0.1, 0, 0.01);
    private double targetPosition1 = 0.0;
    private double targetPosition2 = 0.0;
 

    private Ultrasonic sensor = new Ultrasonic(null, null);

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

    public void hold() {
        double position1 = encoder1.getPosition();
        double position2 = encoder2.getPosition();
        
        double output1 = pidController1.calculate(position1, targetPosition1);
        double output2 = pidController2.calculate(position2, targetPosition2);

        spark1.set(output1);
        spark2.set(output2);
    }
    
    /*
    public void hold(){
        spark1.set(0.1);
        spark2.set(0.1);
    }
    */

    public void stop() {
        spark1.set(0);
        spark2.set(0);
    }
    @Override
    public void periodic(){
        if (robotState.isAlgIn || robotState.isCoralIn) {
            hold();
        }
    }

    public boolean isSensorDetected(){
        return sensor.getRangeInches()>0&&sensor.getRangeInches()<1;
    }
    public Command pullCoral(String direction) {
        return startEnd(
            () -> pull(),
            () -> {
                targetPosition1 = encoder1.getPosition();
                targetPosition2 = encoder2.getPosition();
                hold();
                robotState.isCoralIn = true;
                robotState.coralDirection = direction;
            }
        ).until(() -> isSensorDetected()).withTimeout(1);
    }
    public Command pushCoral(){
        return startEnd(()->push(), ()->{stop();robotState.isCoralIn=false;}).until(()->!isSensorDetected());
    }

    public Command pullAlg() {
        return startEnd(
            () -> pull(),
            () -> {
                targetPosition1 = encoder1.getPosition();
                targetPosition2 = encoder2.getPosition();
                hold();
                robotState.isAlgIn = true;
            }
        ).until(() -> isSensorDetected()).withTimeout(1);
    }

    public Command pushAlg(){
        return startEnd(()->push(), ()->{stop();robotState.isAlgIn=false;}).until(()->!isSensorDetected());
    }
    
}
