// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  //NOTE: CanID's are not inputted, do that !
  SparkMax armAngleMotor = new SparkMax(IntakeConstants.armAngleMotorID, MotorType.kBrushless);
  SparkMax wheelMotor = new SparkMax(IntakeConstants.intakeWheelMotorID, MotorType.kBrushless);
  SparkMax netAngleMotor = new SparkMax(IntakeConstants.netAngleMotorID, MotorType.kBrushless);

  SparkClosedLoopController armPID = armAngleMotor.getClosedLoopController();
  SparkClosedLoopController wheelPID = wheelMotor.getClosedLoopController();
  SparkClosedLoopController netPID = netAngleMotor.getClosedLoopController();

  public Intake() {
    //TODO: add encoder to net angle motor thing
    armAngleMotor.configure(IntakeConstants.armAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    netAngleMotor.configure(IntakeConstants.netAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wheelMotor.configure(IntakeConstants.wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
  }

  @Override
  public void periodic() {
    
  }

  public double getArmAngle() {
    return armAngleMotor.getAbsoluteEncoder().getPosition(); 
  }

  public void moveArm(double angle) {
    armPID.setSetpoint(angle, ControlType.kPosition);
  }

  public double getWheelVelocity() {
    return wheelMotor.getAbsoluteEncoder().getVelocity();
  }

  public void setWheelVelocity(double velocity) {
    wheelPID.setSetpoint(velocity, ControlType.kVelocity);
  }

  public double getNetAngle() {
    return netAngleMotor.getAbsoluteEncoder().getPosition(); 
  }

  public void moveNet(double angle) {
    netPID.setSetpoint(angle, ControlType.kPosition);
  }
}