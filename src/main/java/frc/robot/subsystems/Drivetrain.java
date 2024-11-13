// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.lib.LightningShuffleboard;

public class Drivetrain extends SubsystemBase {
    // Create the simulation model of our drivetrain.
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2), // 2 FALCON motors on each side of the drivetrain.
            7.29, // 7.29:1 gearing reduction.
            7.5, // MOI of 7.5 kg m^2 (from CAD model).
            60.0, // The mass of the robot is 60 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            0.7112, // The track width is 0.7112 meters.

            // The standard deviations for measurement noise:
            // x and y: 0.001 m
            // heading: 0.001 rad
            // l and r velocity: 0.1 m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.01, 0.05, 0.005, 0.005));

    private TalonFX m_leftMotor = new TalonFX(RobotMap.DRIVE_LEFT_ID);
    private TalonFX m_rightMotor = new TalonFX(RobotMap.DRIVE_RIGHT_ID);

    private Field2d m_field = new Field2d();

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7112);

    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            Rotation2d.fromRotations(0),
            0, 0,
            new Pose2d(0, 0, new Rotation2d()));


    public Drivetrain() {
        AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new ReplanningConfig(), // The robot configuration
                () -> false,
                this // Reference to this subsystem to set requirements
    );    
    }

    public void simulationPeriodic() {
        m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
                m_rightMotor.get() * RobotController.getInputVoltage());

        m_driveSim.update(0.02);
    }

    public void setVelocities(double xVelocity, double omegaVelocity ) {
        var chassisSpeeds = new ChassisSpeeds(xVelocity, 0, omegaVelocity);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        m_leftMotor.set(wheelSpeeds.leftMetersPerSecond / RobotMap.MAX_SPEED);
        m_rightMotor.set(wheelSpeeds.rightMetersPerSecond / RobotMap.MAX_SPEED);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        m_leftMotor.set(wheelSpeeds.leftMetersPerSecond / RobotMap.MAX_SPEED);
        m_rightMotor.set(wheelSpeeds.rightMetersPerSecond / RobotMap.MAX_SPEED);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(pose.getRotation(), m_driveSim.getLeftPositionMeters(), m_driveSim.getRightPositionMeters(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_driveSim.getLeftVelocityMetersPerSecond(), m_driveSim.getRightVelocityMetersPerSecond());

        return kinematics.toChassisSpeeds(wheelSpeeds);
    }

    public void periodic() {
        m_odometry.update(m_driveSim.getHeading().times(1),
                m_driveSim.getLeftPositionMeters(),
                m_driveSim.getRightPositionMeters());
        m_field.setRobotPose(m_odometry.getPoseMeters());

        LightningShuffleboard.setDouble("drivetrain", "sped", m_driveSim.getLeftVelocityMetersPerSecond());


        LightningShuffleboard.send("drivetrain", "pose", m_field);
    }
}
