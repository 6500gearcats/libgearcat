package frc.team6500.subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.team6500.args.NeckArgs;

public class temp {
    SparkMax neckMotor;
    AbsoluteEncoder neckEncoder;

    private final SparkMax m_neckMotor;
    public temp(NeckArgs args) {
        m_neckMotor = new SparkMax(args.kNeckMotorPort, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        neckEncoder = m_neckMotor.getAbsoluteEncoder();

    }

    public double getAngle() {
        double angle = neckEncoder.getPosition();
        if(angle < 0.9) {
            angle = 0;
        }
        return angle;
    }

    public double setSpeed(double speed) {
        m_neckMotor.set(speed);
        return speed;
    }



    
}
