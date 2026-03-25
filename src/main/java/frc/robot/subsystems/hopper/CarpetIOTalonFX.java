package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetRollerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class CarpetIOTalonFX implements CarpetIO {
  private final TalonFX talon = new TalonFX(CarpetRollerConstants.id, Constants.RIO_CANBUS);

  public static class CarpetIOTalonFXConfig extends CTREMechanism.Config {
    public CarpetIOTalonFXConfig() {
      super("RollerTalonFX",Constants.RIO_CANBUS);
      configPIDGains(CarpetRollerConstants.p, CarpetRollerConstants.i, CarpetRollerConstants.d);
      configNeutralBrakeMode(CarpetRollerConstants.breakType);
      configFeedbackSensorSource(CarpetRollerConstants.feedbackSensorCTRE);
      configSupplyCurrentLimit(CarpetRollerConstants.supplyLimit);
    }
  }

  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<AngularVelocity> rollerRPM;
  private StatusSignal<Current> currentAmps;

  // No clue what this means copied from ModuleIO
  private final Debouncer carpetConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private CarpetIOTalonFXConfig config = new CarpetIOTalonFXConfig();

  public CarpetIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    rollerRPM = talon.getVelocity();
    currentAmps = talon.getSupplyCurrent();
    config.applyTalonConfig(talon);
    
    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, rollerRPM);
  }

  @Override
  public void updateInputs(CarpetIOInputs inputs) {
    var carpetStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, rollerRPM);

    inputs.rollerConnected = carpetConnectedDebounce.calculate(carpetStatus.isOK());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.RPM = rollerRPM.getValue().in(RPM);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    talon.setVoltage(voltage);
  }
}
