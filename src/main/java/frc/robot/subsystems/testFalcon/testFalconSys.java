// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testFalcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class testFalconSys extends SubsystemBase {
  private final TalonFX m_testFalcon = new TalonFX(Constants.DriveConstants.k_TestMotor);

  /** Creates a new testFalconSys. */
  public testFalconSys() {
  }

  @Override
  public void periodic() {
  }

  public void setPrecentOutput(double precent) {
    m_testFalcon.set(ControlMode.PercentOutput, precent);
  }

}