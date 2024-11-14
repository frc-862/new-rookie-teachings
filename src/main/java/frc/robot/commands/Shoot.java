// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
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

    private Timer timer;
    private boolean hasShot;

    public Shoot(Shooter shooter, Indexer indexer) {
        //add requirements
        this.shooter = shooter;
        this.indexer = indexer;

        timer = new Timer();
        hasShot = false;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {
        if (!hasShot) {
            shooter.setPower(1d);

            if (timer.get() > ShooterConstants.TIME_TO_SHOOT){
                indexer.setPower(0.5d);
                hasShot = true;
                timer.stop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
