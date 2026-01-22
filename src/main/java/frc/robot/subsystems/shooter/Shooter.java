package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.team5431.titan.core.subsystem.CTREMechanism;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends CTREMechanism {

    @Getter @Setter private ShooterState shooterState;
    @Getter @Setter private ShooterModes shooterMode;
    private boolean attached;
    private TalonFX motor;

  public static class ShooterConfig extends Config {
    public ShooterConfig() {
      super("Shooter", ShooterConstants.id, Constants.CANBUS);
            configNeutralBrakeMode(ShooterConstants.breakType);
            configStatorCurrentLimit(ShooterConstants.stallLimit, true);
            configSupplyCurrentLimit(ShooterConstants.supplyLimit, true);
            configForwardSoftLimit(ShooterConstants.maxForwardOutput, true);
            configReverseSoftLimit(ShooterConstants.maxReverseOutput, true);
            configPIDGains(ShooterConstants.p, ShooterConstants.i, ShooterConstants.d);
            configPeakOutput(ShooterConstants.maxForwardOutput, ShooterConstants.maxReverseOutput);
            configGearRatio(ShooterConstants.gearRatio);
            configMotorInverted(ShooterConstants.inverted);
        }
    }

    public Shooter(TalonFX motor, boolean attached) {
        super(motor, attached);
        this.attached = attached;
        this.motor = motor;
        this.shooterMode = ShooterModes.IDLE;
        this.shooterState = ShooterState.IDLE;

        config.applyTalonConfig(motor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Shooter Mode", getShooterMode().toString());
        

        switch (this.shooterMode) {
            case IDLE:
                setShooterState(ShooterState.IDLE);
                break;
            case SHOOTER:
                setShooterState(ShooterState.SHOOTER);
                break;
            case REVERSE:
                setShooterState(ShooterState.REVERSE);
                break;
        }

    }

    public Command runShooterCommand(ShooterModes shooterModes) {
        return new RunCommand(() -> setVelocity(shooterModes.speed), this).withName("Shooter.runEnum");
    }

    @Override
    public Config setConfig() {
        // configure motors/sensors here
        this.config = new ShooterConfig();
        return this.config;
    }
}
