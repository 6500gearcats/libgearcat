package frc.robot.team6500;

import frc.robot.LimelightHelpers;


public class GCLimelight {
    // Limelight name
    private String name;

    public GCLimelight(String theName)
    {
        name = theName;
    }

    public double getYawDegrees(){
        if(LimelightHelpers.getTV(name))
        {
            return LimelightHelpers.getTX(name);
        }
        return 0;
    }

    public double getPitchDegrees(){
        if(LimelightHelpers.getTV(name))
        {
            return LimelightHelpers.getTY(name);
        }
        return 0;
    }

    public double getTargetAreaPercent()
    {
        if(LimelightHelpers.getTV(name))
        {
            return LimelightHelpers.getTA(name);
        }
        return 0;
    }

    public double getTargetDistanceX()
    {
        if(LimelightHelpers.getTV(name))
        {
            double x = (6.5/(2*getTargetAreaPercent()))/Math.tan(90*Math.PI/180);
            double distance = x/Math.cos(getYawDegrees()*Math.PI/180);
            return distance;
        }
        return 0;
    }
}
