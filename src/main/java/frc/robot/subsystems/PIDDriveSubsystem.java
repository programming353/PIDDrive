// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PIDDriveSubsystem extends SubsystemBase {
  public final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
  public RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private SparkMaxPIDController leftFrontPIDCon = leftFrontMotor.getPIDController();
  int smartMotionSlot = 0;
  int allowedErr;
  int minVel;

  public PIDDriveSubsystem() {
    leftFrontMotor.restoreFactoryDefaults();
    initializePID(leftFrontPIDCon);
  }
  public void initializePID(SparkMaxPIDController p){
    p.setP(Constants.kP);
    p.setI(Constants.kI);
    p.setD(Constants.kD);
    p.setIZone(Constants.kIz);
    p.setFF(Constants.kFF);
    p.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    p.setSmartMotionMaxVelocity(Constants.maxVel, smartMotionSlot);
    p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    p.setSmartMotionMaxAccel(Constants.maxAcc, smartMotionSlot);
    p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
