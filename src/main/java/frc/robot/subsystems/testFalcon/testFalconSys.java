// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testFalcon;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class testFalconSys extends SubsystemBase {
  private final WPI_TalonFX m_testFalcon = new WPI_TalonFX(Constants.DriveConstants.k_TestMotor);
  /** Creates a new testFalconSys. */
  public testFalconSys() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}