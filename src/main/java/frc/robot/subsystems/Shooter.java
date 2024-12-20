// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.lib.SimMotor;

public class Shooter extends SubsystemBase {

  private SimMotor motor;

  public Shooter() {
    motor = new SimMotor(RobotMap.FLYWHEEL_ID);
  }

  @Override
  public void periodic() {
  }

  public void setPower(double power) {
    motor.set(power);
  }

  public void stop() {
    setPower(0d);
  }
}
