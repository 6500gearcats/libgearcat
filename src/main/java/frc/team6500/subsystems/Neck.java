// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6500.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team6500.args.NeckArgs;

public class Neck extends SubsystemBase {
  /** Creates a new Neck. */
  SparkMax neckMotor;
  AbsoluteEncoder neckEncoder;
  ArmFeedforward feedforward;
  SparkClosedLoopController sparkClosedLoopController;
  PIDController pidController;
  NeckArgs args;

  private final SparkMax m_neckMotor;
  public Neck(NeckArgs args) {
    this.args = args;
      m_neckMotor = new SparkMax(args.kNeckMotorPort, SparkLowLevel.MotorType.kBrushless);

    neckEncoder = m_neckMotor.getAbsoluteEncoder();
    //Feedfordward is used to calculate the added power required to 
    //counter act things like gravity when moving the motor
    feedforward = new ArmFeedforward(args.kS, args.kG, args.kV);

      SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.pid(args.kP, args.kI, args.kD);
        m_neckMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
    sparkClosedLoopController = m_neckMotor.getClosedLoopController();

    pidController = new PIDController(args.kP2, args.kI2, args.kD2);
  }

  public double getAngle() {
    double angle = neckEncoder.getPosition();
    if(angle < 0.9) {
        angle = 0;
    }
    return angle;
  }

  public double move(double speed) {
    m_neckMotor.set(speed);
    return speed; 
  }
  public void moveTo(Rotation2d target) {
    //setRefrence sets the target angle for the motor to move to
    //kPosition is the control type and is used for moving to a specific angle
    sparkClosedLoopController.setReference(
                                    target.getRadians(), ControlType.kPosition,
                                    ClosedLoopSlot.kSlot0,
                                    feedforward.calculate(target.getRadians(), 0)
                                    );
  }

  public void moveTo(double target) {
    move(pidController.calculate(target, getAngle() * args.NeckFeedForwardSpeed));
  }


  public void stablize() {
    //Creates a unit circle form the current angle
    Rotation2d m_target = Rotation2d.fromDegrees(getAngle());
    //Checks for deviation from the target angle
    if(Math.abs(m_target.getDegrees() - getAngle()) > args.EncoderThereshold) {
      moveTo(m_target);
      SmartDashboard.putString("Neck Status", "Stablizing");
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Neck Angle in Degrees", getAngle());
  }


}
