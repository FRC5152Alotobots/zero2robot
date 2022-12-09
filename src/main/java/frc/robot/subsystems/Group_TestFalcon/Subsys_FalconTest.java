// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Group_TestFalcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Subsys_FalconTest extends SubsystemBase {
  private final TalonFX m_testFalconMotor = new TalonFX(Constants.DriveConstants.k_TestMotor);

  /** Creates a new testFalconSys. */
  public Subsys_FalconTest() {
  }

  @Override
  public void periodic() {
  }

  public void setPrecentOutput(double precent) {
    m_testFalconMotor.set(ControlMode.PercentOutput, precent);
  }

  public void setPosition(double position) {
    m_testFalconMotor.set(ControlMode.Position, position);
  }

}