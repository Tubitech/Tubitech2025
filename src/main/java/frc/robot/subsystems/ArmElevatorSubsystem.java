package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElevatorSubsystem extends SubsystemBase{
    private final TrapezoidProfile.Constraints constraintsL1 = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController controllerL1 = new ProfiledPIDController(1, 1, 1, constraintsL1, 1);
    private final ArmFeedforward feedforwardL1 = new ArmFeedforward(1, 1, 1);

    private final TrapezoidProfile.Constraints constraintsL2 = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController controllerL2 = new ProfiledPIDController(1, 1, 1, constraintsL2, 1);
    private final ArmFeedforward feedforwardL2 = new ArmFeedforward(1, 1, 1);

    private final SparkMax spark1 = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax spark2 = new SparkMax(0, MotorType.kBrushless);

    private DutyCycleEncoder throughbore1 = new DutyCycleEncoder(0);
    private DutyCycleEncoder throughbore2 = new DutyCycleEncoder(1);

    private final SparkMax elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    private final TrapezoidProfile.Constraints elevatoConstraints = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController controllerElevator = new ProfiledPIDController(1, 1, 1, elevatoConstraints, 1);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(1, 1, 1,1,1);
    private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);
    /*Constants şimdilik burda */
    private final static double L1 = 10;
    private final static double L2 = 10;
    private static final double THETA1_MIN = Math.toRadians(-90.0);
    private static final double THETA1_MAX = Math.toRadians(0.0);
    private static final double THETA2_MIN = Math.toRadians(-120.0);
    private static final double THETA2_MAX = Math.toRadians(120.0);


    private double x;
    private double y;
    private double theta;

    double targetX = 0;
    double targetY = 0;
    double targetH = 0;
    public void setTargetX(double x){
        targetX = x;
    }
    public void setTargetY(double y){
        targetY = y;
    }
    public void setTargetH(double h){
        targetH = h;
    }
    public ArmElevatorSubsystem(){

    }
    private double getFirstJointAngle(){
        return throughbore1.get();
    }
    private double getSecondJointAngle(){
        return throughbore2.get();
    }
    private double getElevatorPosition(){
        return elevatorEncoder.get();
    }
    @Override
    public void periodic(){
        getGripperPos();
        setArmPosition(targetX, targetY);
        setElevatorPosition(targetH);

    }
    /*Forward Kinematics */
    private void getGripperPos(){
        double a = L1*Math.cos(getFirstJointAngle());
        double b = L1*Math.sin(getFirstJointAngle());
        theta = getFirstJointAngle() + getSecondJointAngle();
        x = a + L2*Math.cos(theta);
        y = b + L2*Math.sin(theta);
    }
    /*Inverse Kinematics */
    private double[][] findGripperPos(double desiredx, double desiredy){
        /*q2 pos */
        if (Math.sqrt(x * x + y * y) > (L1 + L2)) {
            return null; // Ulaşılamaz
        }

        double cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        double theta2Up = Math.acos(cosTheta2);  // Dirsek yukarıda
        double theta2Down = -Math.acos(cosTheta2);

        double k1Up = L1 + L2 * Math.cos(theta2Up);
        double k2Up = L2 * Math.sin(theta2Up);
        double theta1Up = Math.atan2(y, x) - Math.atan2(k2Up, k1Up);

        double k1Down = L1 + L2 * Math.cos(theta2Down);
        double k2Down = L2 * Math.sin(theta2Down);
        double theta1Down = Math.atan2(y, x) - Math.atan2(k2Down, k1Down);

        theta1Down = clampAngle(theta1Down, THETA1_MIN, THETA1_MAX);
        theta2Down = clampAngle(theta2Down, THETA2_MIN, THETA2_MAX);
        return new double[][] {
            {theta1Up, theta2Up}, // Dirsek yukarıda
            {theta1Down, theta2Down} // Dirsek aşağıda
        };

        // double q2 =  Math.acos((Math.pow(desiredx, 2)+Math.pow(desiredy, 2)-Math.pow(distBetweenTwoJoints, 2)-Math.pow(distBetweenSecondJointAndGripper, 2))/(2*distBetweenSecondJointAndGripper*distBetweenTwoJoints));
        // double q1 = Math.atan((desiredy/desiredx))-Math.atan((distBetweenSecondJointAndGripper*q2)/(distBetweenTwoJoints+distBetweenSecondJointAndGripper*Math.cos(q2)));
        // Map<String, Number> degrees = new HashMap<>();
        // degrees.put("q1", q1);
        // degrees.put("q2", q2);
        // return degrees;  
    }
    private void setArmPosition(double desiredX, double desiredY){
        double[][] radianAngles = findGripperPos(desiredX, desiredY);
        controllerL1.setGoal(radianAngles[0][0]);
        controllerL2.setGoal(radianAngles[0][1]);
        setL1Motor();
        setL2Motor();
    }
    private void setElevatorPosition(double height){
        controllerElevator.setGoal(height);
        setElevatorMotor();
    }
    private void setL1Motor(){
        spark1.setVoltage(controllerL1.calculate(getFirstJointAngle())+feedforwardL1.calculate(controllerL1.getSetpoint().position,controllerL1.getSetpoint().velocity));
    }
    private void setL2Motor(){
        spark2.setVoltage(controllerL2.calculate(getSecondJointAngle())+feedforwardL2.calculate(controllerL2.getSetpoint().position,controllerL2.getSetpoint().velocity));
    }
    private void setElevatorMotor(){
        elevatorMotor.setVoltage(controllerElevator.calculate(getElevatorPosition())+elevatorFeedforward.calculate(controllerElevator.getSetpoint().velocity));
    }
    private double clampAngle(double angle, double minAngle, double maxAngle) {
        if (angle < minAngle) {
            angle = minAngle;
        }
        if (angle > maxAngle) {
            angle = maxAngle;
        }
        return angle;
    }
}
