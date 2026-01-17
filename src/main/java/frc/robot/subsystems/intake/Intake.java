package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.team5431.titan.core.subsystem.CTREMechanism;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends CTREMechanism {

  public static class IntakeConfig extends Config {
    public IntakeConfig() {
      super("Intake", IntakeConstants.id, Constants.canbus);
      configPIDGains(IntakeConstants.p, IntakeConstants.i, IntakeConstants.d);
      configNeutralBrakeMode(IntakeConstants.breakType);
      configFeedbackSensorSource(IntakeConstants.feedbackSensor);
      configGearRatio(IntakeConstants.gearRatio);
      configGravityType(IntakeConstants.gravityType);
      configSupplyCurrentLimit(IntakeConstants.supplyLimit, IntakeConstants.useSupplyLimit);
      configStatorCurrentLimit(IntakeConstants.stallLimit, IntakeConstants.useStallLimit);
      configReverseSoftLimit(
          IntakeConstants.maxReverseRotation.in(Rotation), IntakeConstants.useRMaxRotation);
      configForwardSoftLimit(
          IntakeConstants.maxFowardRotation.in(Rotation), IntakeConstants.useFMaxRotation);
    }
  }

  private TalonFX motor;
  @Getter private IntakeModes mode;

  public Intake(TalonFX motor, boolean attached) {
    super(motor, attached);

    this.mode = IntakeModes.IDLE;
    this.motor = motor;
    this.attached = attached;
    config.applyTalonConfig(motor);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Setpoint", getMode().speed.in(RPM));
  }

  public void runEnum(IntakeModes intakemode, boolean rpm) {
    this.mode = intakemode;
    if (rpm) {
      setVelocity(intakemode.speed);
    } else {
      setPercentOutput(intakemode.output);
    }
  }

  public Command runIntakeCommand(IntakeModes intakeModes) {
    return new RunCommand(() -> this.runEnum(intakeModes, IntakeConstants.useRpm), this)
        .withName("Intake.runEnum");
  }

  public Command runIntakeCommand(IntakeModes intakeModes, boolean rpm) {
    return new RunCommand(() -> this.runEnum(intakeModes, rpm), this).withName("Intake.runEnum");
  }

  @Override
  protected Config setConfig() {
    this.config = new IntakeConfig();
    return config;
  }
}
