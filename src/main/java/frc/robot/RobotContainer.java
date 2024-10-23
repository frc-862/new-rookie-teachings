// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.RunCollector;
import frc.robot.subsystems.Collector;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    private Collector collector = new Collector();

    private final XboxController controller = new XboxController(0);

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
    }

    private void configureBindings() {
        
    }

    public void configureDefaultCommands() {
        collector.setDefaultCommand(new RunCollector(collector, () -> controller.getRawAxis(1) - controller.getRawAxis(2)));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
