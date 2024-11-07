// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated wheel mechanism. */
public class WheelSim extends LinearSystemSim<N2, N1, N2> {
  // Gearbox for the wheel.
  private final DCMotor m_gearbox;

  // The gearing from the motors to the output.
  private final double m_gearing;

  /**
   * Creates a simulated wheel mechanism.
   *
   * @param plant   The linear system that represents the wheel. This system can
   *                be created with
   *                {@link edu.wpi.first.math.system.plant.LinearSystemId#createwheelSystem(DCMotor, double,
   *                double)}.
   * @param gearbox The type of and number of motors in the wheel gearbox.
   * @param gearing The gearing of the wheel (numbers greater than 1 represent
   *                reductions).
   */
  public WheelSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing) {
    super(plant);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  // /**
  // * Creates a simulated wheel mechanism.
  // *
  // * @param plant The linear system that represents the wheel.
  // * @param gearbox The type of and number of motors in the wheel gearbox.
  // * @param gearing The gearing of the wheel (numbers greater than 1 represent
  // reductions).
  // * @param measurementStdDevs The standard deviations of the measurements.
  // */
  // public WheelSim(
  // LinearSystem<N2, N1, N2> plant,
  // DCMotor gearbox,
  // double gearing,
  // Matrix<N1, N1> measurementStdDevs) {
  // super(plant, measurementStdDevs);
  // m_gearbox = gearbox;
  // m_gearing = gearing;
  // }

  /**
   * Creates a simulated wheel mechanism.
   *
   * @param gearbox          The type of and number of motors in the wheel
   *                         gearbox.
   * @param gearing          The gearing of the wheel (numbers greater than 1
   *                         represent reductions).
   * @param jKgMetersSquared The moment of inertia of the wheel. If this is
   *                         unknown, use the
   *                         {@link #wheelSim(LinearSystem, DCMotor, double, Matrix)}
   *                         constructor.
   */
  public WheelSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
    super(LinearSystemId.createDCMotorSystem(gearbox, jKgMetersSquared, gearing));

    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  // /**
  // * Creates a simulated wheel mechanism.
  // *
  // * @param gearbox The type of and number of motors in the wheel gearbox.
  // * @param gearing The gearing of the wheel (numbers greater than 1 represent
  // reductions).
  // * @param jKgMetersSquared The moment of inertia of the wheel. If this is
  // unknown, use the
  // * {@link #wheelSim(LinearSystem, DCMotor, double, Matrix)} constructor.
  // * @param measurementStdDevs The standard deviations of the measurements.
  // */
  // public WheelSim(
  // DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N1, N1>
  // measurementStdDevs) {
  // super(
  // LinearSystemId.createDCMotorSystem(gearbox, jKgMetersSquared, gearing));
  // m_gearbox = gearbox;
  // m_gearing = gearing;
  // }

  /**
   * Sets the wheel's state.
   *
   * @param position The new position.
   */
  public void setState(Rotation2d position, double velocity) {
    setState(MatBuilder.fill(Nat.N2(), Nat.N1(), position.getRadians(), velocity));
  }

  public void setVelocity(double velocity) {
    setState(MatBuilder.fill(Nat.N2(), Nat.N1(), getOutput(1), velocity));
  }

  /**
   * Returns the wheel position.
   *
   * @return The wheel position.
   */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(getOutput(0));
  }

  /**
   * Returns the wheel velocity.
   *
   * @return The wheel velocity.
   */
  public double getVelocity() {
    return getOutput(1);
  }

  // /**
  // * Returns the wheel current draw.
  // *
  // * @return The wheel current draw.
  // */
  // @Override
  // public double getCurrentDrawAmps() {
  // // I = V / R - omega / (Kv * R)
  // // Reductions are output over input, so a reduction of 2:1 means the motor is
  // spinning
  // // 2x faster than the wheel
  // return m_gearbox.getCurrent(getAngularVelocityRadPerSec() * m_gearing,
  // m_u.get(0, 0))
  // * Math.signum(m_u.get(0, 0));
  // }

  /**
   * Sets the input voltage for the wheel.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }
}
