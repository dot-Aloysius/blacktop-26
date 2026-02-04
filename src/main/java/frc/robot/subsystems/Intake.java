// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  //NOTE: CanID's are not inputted, do that !
  SparkMax armAngleMotor = new SparkMax(IntakeConstants.armAngleMotorID, MotorType.kBrushless);
  SparkMax wheelMotor = new SparkMax(IntakeConstants.intakeWheelMotorID, MotorType.kBrushless);
  SparkMax netAngleMotor = new SparkMax(IntakeConstants.netAngleMotorID, MotorType.kBrushless);

  SparkClosedLoopController AnglePID = armAngleMotor.getClosedLoopController();

  public Intake() {
    armAngleMotor.getAbsoluteEncoder().getPosition();
    AnglePID.

    
  }

  @Override
  public void periodic() {
    
  }

  public double getArmAngle() {
    return armAngleMotor.getAbsoluteEncoder().getPosition(); 
  }

  public void setArmAngle() {
    armAngleMotor.set
  }
}