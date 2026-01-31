package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon = new TalonFX(IntakeRollerConstants.id, Constants.CANBUS);

  public static class RollerTalonFXConfig extends CTREMechanism.Config {
    public RollerTalonFXConfig() {
      super("RollerTalonFX",Constants.CANBUS);
      configPIDGains(IntakeRollerConstants.p, IntakeRollerConstants.i, IntakeRollerConstants.d);
      configNeutralBrakeMode(IntakeRollerConstants.breakType);
      configFeedbackSensorSource(IntakeRollerConstants.feedbackSensorCTRE);
      // configGearRatio(IntakeRollerConstants.gearRatio);
      // configGravityType(IntakeRollerConstants.gravityType);
      configSupplyCurrentLimit(IntakeRollerConstants.supplyLimit);
      configReverseSoftLimit(
          IntakeRollerConstants.maxReverseRotation.in(Rotation), IntakeRollerConstants.useRMaxRotation);
      configForwardSoftLimit(
          IntakeRollerConstants.maxFowardRotation.in(Rotation), IntakeRollerConstants.useFMaxRotation);
    }
  }

  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<AngularVelocity> rollerRPM;
  private StatusSignal<Current> currentAmps;

  private RollerTalonFXConfig config = new RollerTalonFXConfig();

  public RollerIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    rollerRPM = talon.getVelocity();
    currentAmps = talon.getStatorCurrent();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, rollerRPM);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, rollerRPM);

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.RPM = rollerRPM.getValue().in(RPM);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    talon.setVoltage(voltage);
  }
}
