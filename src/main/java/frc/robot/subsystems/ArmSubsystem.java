package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    // private SparkMax spark1 = new SparkMax(0, MotorType.kBrushless);
    // private SparkMax spark2 = new SparkMax(0, MotorType.kBrushless);


    private DutyCycleEncoder throughbore1 = new DutyCycleEncoder(0);
    private DutyCycleEncoder throughbore2 = new DutyCycleEncoder(1);

    /*Constants şimdilik burda */
    private double L1 = 10;
    private double L2 = 10;

    private double x;
    private double y;
    private double theta;

    public ArmSubsystem(){

    }
    private double getFirstJointAngle(){
        return throughbore1.get();
    }
    private double getSecondJointAngle(){
        return throughbore2.get();
    }
    @Override
    public void periodic(){
        getGripperPos();
    }
    /*Forward Kinematics */
    private void getGripperPos(){
        double a = L1*Math.cos(getFirstJointAngle());
        double b = L1*Math.sin(getFirstJointAngle());
        theta = getSecondJointAngle() + getSecondJointAngle();
        x = a + L2*Math.cos(theta);
        y = b + L2*Math.sin(theta);
    }
    /*Inverse Kinematics */
    public double[][] findGripperPos(double desiredx, double desiredy){
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

}
