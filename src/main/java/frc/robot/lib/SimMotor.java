// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import java.util.ArrayList;
import java.lang.Exception;

/** Add your docs here. */
public class SimMotor {
    DCMotor motor = DCMotor.getFalcon500(1);
    private WheelSim wheelSim = new WheelSim(motor, 1, 1);
    private Mechanism2d mech2d = new Mechanism2d(90, 90);
    private MechanismRoot2d root = mech2d.getRoot("root", 45, 45);
    private MechanismLigament2d ligament = root.append(new MechanismLigament2d("ligma", 10, 0));
    private MechanismLigament2d ligament1 = root.append(new MechanismLigament2d("ligma1", 10, 90));
    private MechanismLigament2d ligament2 = root.append(new MechanismLigament2d("ligma2", 10, -90));
    private MechanismLigament2d ligament3 = root.append(new MechanismLigament2d("ligma3", 10, 180));


    private String id;
    private double maxSpeed;

    private static ArrayList<SimMotor> motors = new ArrayList<SimMotor>();
    public static void updateStates() {
        for (SimMotor motor : motors) {
            motor.update();
        }
    }

    public SimMotor(int id) {
        this.id = String.valueOf(id);
        //bad code but womp womp
        switch(id) {
            case 5:
                wheelSim = new WheelSim(motor, 1, 0.1);
                maxSpeed = 18;
            break;

            case 4: //flywheel
                wheelSim = new WheelSim(motor, 1, 0.05);
                maxSpeed = 5800;
            break;

            default:
                System.out.println("you're being silly");
                wheelSim = new WheelSim(motor, 1, 0.5);
                maxSpeed = 5800;
            break;
        }
        motors.add(this);
    }

    public void update() {
        wheelSim.update(0.02);
        ligament.setAngle(wheelSim.getPosition().getDegrees());
        ligament1.setAngle(wheelSim.getPosition().getDegrees() + 90);
        ligament2.setAngle(wheelSim.getPosition().getDegrees() - 90);
        ligament3.setAngle(wheelSim.getPosition().getDegrees() + 180);
        LightningShuffleboard.setDouble(id, "curr veloc", wheelSim.getVelocity());
        LightningShuffleboard.send(id, "ligament", mech2d);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wheelSim.getCurrentDrawAmps()));    
    }
    
    public void set(double power) {
        //cut power when veloc above motor limits
        if(Math.abs(wheelSim.getVelocity()) < maxSpeed) {
            wheelSim.setInputVoltage(power * 12);
        } else {
            wheelSim.setInputVoltage(0);
        }
    }

    public double getVelocity() {
        return wheelSim.getVelocity();
    }

    public void stop() {
        wheelSim.setInputVoltage(0);
    }

    
    public double getPosition() {
        return wheelSim.getPosition().getRotations();
    }
}
