package frc.team6500.args;

public class NeckArgs {
    public int kNeckMotorPort = 0;

    //ArmFeedforward Constants
    public double kS = 1.7;
    public double kG = 0.5;  
    public double kV = 0.0;

    public double EncoderThereshold = 0.1;

    public double NeckFeedForwardSpeed = 0.5;

    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;

    public double kP2 = 1.0;
    public double kI2 = 0.0;
    public double kD2 = 0.0;
    public double kRatio = 0.25;

    public NeckArgs() {}
    
}
