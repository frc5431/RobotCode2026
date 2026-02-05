package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX talon = new TalonFX(IntakePivotConstants.id, Constants.CANBUS);

  public static class PivotTalonFXConfig extends CTREMechanism.Config {
    public PivotTalonFXConfig() {
      super("PivotTalonFX", Constants.CANBUS);
      configPIDGains(IntakePivotConstants.p, IntakePivotConstants.i, IntakePivotConstants.d);
      configNeutralBrakeMode(IntakePivotConstants.breakType);
      configFeedbackSensorSource(IntakePivotConstants.feedbackSensorCTRE);
      configGearRatio(IntakePivotConstants.gearRatio);
      configGravityType(IntakePivotConstants.gravityType);
      configSupplyCurrentLimit(IntakePivotConstants.supplyLimit);
      configReverseSoftLimit(
          IntakePivotConstants.maxReverseRotation.in(Rotation), IntakePivotConstants.useRMaxRotation);
      configForwardSoftLimit(
          IntakePivotConstants.maxFowardRotation.in(Rotation), IntakePivotConstants.useFMaxRotation);
    }
  }

  
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Angle> pivotPosition;
  private StatusSignal<Current> currentAmps;

  // No clue what this means copied from ModuleIO
  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private PivotTalonFXConfig config = new PivotTalonFXConfig();

  public PivotIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    pivotPosition = talon.getPosition();
    currentAmps = talon.getStatorCurrent();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, pivotPosition);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    var pivotStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, pivotPosition);

    inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
    
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.positionAngle = pivotPosition.getValue().in(Rotation);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    talon.setVoltage(voltage);
  }

  @Override
  public void setPosition(double positionAngle) {
    PositionVoltage mm = config.positionVoltage.withPosition(positionAngle);
      talon.setControl(mm);
  }

}
