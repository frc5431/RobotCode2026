package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ShooterConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class Shooter extends CTREMechanism {

    

    public static class ShooterConfig extends Config {
        public ShooterConfig() {
            super("Shooter", ShooterConstants.id, "make canbus"); //TODO

            configNeutralBrakeMode(ShooterConstants.breakType);
            configStatorCurrentLimit(ShooterConstants.stallLimit, true);
            configSupplyCurrentLimit(ShooterConstants.supplyLimit, true);
            configForwardSoftLimit(ShooterConstants.maxForwardOutput, true);
            configReverseSoftLimit(ShooterConstants.maxReverseOutput, true);
        }
    }

    public Shooter(TalonFX motor, boolean attached) {
        super(motor, attached);

        this.setConfig();

    }



    @Override
    public Config setConfig() {
        // configure motors/sensors here
        this.config = new ShooterConfig();
        return this.config;
    }
}
