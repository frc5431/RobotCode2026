package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX talon = new TalonFX(ClimberConstants.id, Constants.CANIVORE_CANBUS);

  public static class ClimberTalonFXConfig extends CTREMechanism.Config {
    public ClimberTalonFXConfig() {
      super("ClimberTalonFX", Constants.CANIVORE_CANBUS);
      configPIDGains(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
      configNeutralBrakeMode(ClimberConstants.breakType);
      configFeedbackSensorSource(ClimberConstants.feedbackSensorCTRE);
      configGearRatio(ClimberConstants.gearRatio);
      configSupplyCurrentLimit(ClimberConstants.supplyLimit);
    }
  }

  
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Angle> pivotPosition;
  private StatusSignal<Current> currentAmps;

  // No clue stole from ModuleIO
  private final Debouncer climberConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private ClimberTalonFXConfig config = new ClimberTalonFXConfig();

  public ClimberIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    pivotPosition = talon.getPosition();
    currentAmps = talon.getSupplyCurrent();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVoltage, currentAmps, pivotPosition);

  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var climberStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, pivotPosition);

    inputs.climberConnected = climberConnectedDebounce.calculate(climberStatus.isOK());

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.positionAngle = pivotPosition.getValue().in(Rotation);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  // @Override
  // public void setClimberPosition(double positionAngle) {
  //   PositionVoltage mm = config.positionVoltage.withPosition(positionAngle);
  //     talon.setControl(mm);
  // }

  @Override
    public void setVoltage(double voltage) {
        talon.setVoltage(voltage);
    }
}
