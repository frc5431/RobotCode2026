package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon = new TalonFX(IntakeRollerConstants.id, Constants.RIO_CANBUS);

  public static class RollerTalonFXConfig extends CTREMechanism.Config {
    public RollerTalonFXConfig() {
      super("RollerTalonFX", Constants.RIO_CANBUS);
      configPIDGains(IntakeRollerConstants.p, IntakeRollerConstants.i, IntakeRollerConstants.d);
      configNeutralBrakeMode(IntakeRollerConstants.breakType);
      configFeedbackSensorSource(IntakeRollerConstants.feedbackSensorCTRE);
      // configGearRatio(IntakeRollerConstants.gearRatio);
      // configGravityType(IntakeRollerConstants.gravityType);
      configSupplyCurrentLimit(IntakeRollerConstants.supplyLimit);
    }
  }

  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<AngularVelocity> rollerRPM;
  private StatusSignal<Current> currentAmps;

  // No clue what this means copied from ModuleIO
  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

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
    var rollerStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, rollerRPM);

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.RPM = rollerRPM.getValue().in(RPM);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    talon.setVoltage(voltage);
  }
}
