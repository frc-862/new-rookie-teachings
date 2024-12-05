// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotMap;
import frc.robot.commands.Collect;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    private Drivetrain drivetrain = new Drivetrain();
    private Collector collector = new Collector();
    private Shooter shooter = new Shooter();
    private Indexer indexer = new Indexer();



    private final SendableChooser<Command> autoChooser;

    private final XboxController controller = new XboxController(Constants.OperatorConstants.DRIVER);

    public RobotContainer() {


        configureBindings();
        configureDefaultCommands();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
    }

    private void configureBindings() {
        new Trigger(() -> controller.getAButton()).whileTrue(new Shoot(shooter, indexer));
        new Trigger(() -> controller.getRawAxis(2) > 0.1).whileTrue(new Collect(collector, () -> controller.getRawAxis(2)));
    }

    public void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.setVelocities(-controller.getRawAxis(1) * RobotMap.MAX_SPEED, -controller.getRawAxis(0) * RobotMap.MAX_SPEED), drivetrain));

        NamedCommands.registerCommand("Collect", new Collect(collector, () -> 1));
        NamedCommands.registerCommand("Shoot", new Shoot(shooter, indexer));
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
