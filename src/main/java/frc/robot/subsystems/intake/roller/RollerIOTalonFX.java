package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerIOConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon = new TalonFX(IntakeRollerIOConstants.id, Constants.CANBUS);

  public static class RollerTalonFXConfig extends CTREMechanism.Config {
    public RollerTalonFXConfig() {
      super("RollerTalonFX", IntakeRollerIOConstants.id, Constants.CANBUS);
      configPIDGains(IntakeRollerIOConstants.p, IntakeRollerIOConstants.i, IntakeRollerIOConstants.d);
      configNeutralBrakeMode(IntakeRollerIOConstants.breakType);
      configFeedbackSensorSource(IntakeRollerIOConstants.feedbackSensorCTRE);
      configGearRatio(IntakeRollerIOConstants.gearRatio);
      configGravityType(IntakeRollerIOConstants.gravityType);
      configSupplyCurrentLimit(IntakeRollerIOConstants.supplyLimit, IntakeRollerIOConstants.useSupplyLimit);
      configStatorCurrentLimit(IntakeRollerIOConstants.stallLimit, IntakeRollerIOConstants.useStallLimit);
      configReverseSoftLimit(
          IntakeRollerIOConstants.maxReverseRotation.in(Rotation), IntakeRollerIOConstants.useRMaxRotation);
      configForwardSoftLimit(
          IntakeRollerIOConstants.maxFowardRotation.in(Rotation), IntakeRollerIOConstants.useFMaxRotation);
    }
  }

  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<AngularVelocity> rollerRPM;
  private StatusSignal<Double> rollerOutput;

  private RollerTalonFXConfig config;

  public RollerIOTalonFX(TalonFX talon) {
    appliedVoltage = talon.getMotorVoltage();
    rollerRPM = talon.getVelocity();
    rollerOutput = talon.getClosedLoopOutput();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, rollerOutput, rollerRPM);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVoltage, rollerOutput, rollerRPM);

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.RPM = rollerRPM.getValue().in(RPM);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    talon.setVoltage(voltage);
  }

  @Override
  public void setRPM(double rpm) {
    VelocityVoltage output = config.velocityControl.withVelocity(Units.RPM.of(rpm));
    talon.setControl(output);
  }
}
