// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.lib.SimMotor;

public class Indexer extends SubsystemBase {

  private SimMotor motor;

  /** Creates a new Indexer. */
  public Indexer() {
    motor = new SimMotor(RobotMap.INDEXER_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    motor.set(power);
  }

  public void stop() {
    setPower(0d);
  }

}
