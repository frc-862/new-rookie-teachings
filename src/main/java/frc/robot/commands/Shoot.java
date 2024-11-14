// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    /* **REQUIREMENTS**
     * Set shooter power to 1
     * Set indexer power to 0.5
     * 
     * ON END:
     * Stop shooter
     * Stop indexer
     * 
     * TODO: create indexer, implement shooter
    */

    private Shooter shooter;
    private Indexer indexer;

    public Shoot(Shooter shooter, Indexer indexer) {
        //add requirements
        this.shooter = shooter;
        this.indexer = indexer;


    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //do you need to do anything here?
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
