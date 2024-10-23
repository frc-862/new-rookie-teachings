// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  private Talon motor;

  /** Creates a new Collector. */
  public Collector() {
    motor = new Talon(5);
  }

  public void setPower(double power) {
    motor.set(power);
  }

  public void stop() {
    setPower(0d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
