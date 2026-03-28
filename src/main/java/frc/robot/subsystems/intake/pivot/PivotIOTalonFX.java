 package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX talon = new TalonFX(IntakePivotConstants.id, Constants.CANIVORE_CANBUS);
   private final CANcoder cancoder = new CANcoder(IntakePivotConstants.cancoderId, Constants.CANIVORE_CANBUS);

  public static class PivotTalonFXConfig extends CTREMechanism.Config {
    public PivotTalonFXConfig() {
      super("PivotTalonFX", Constants.CANIVORE_CANBUS);
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
  private StatusSignal<Angle> turnAbsolutePosition;
  private StatusSignal<Current> currentAmps;

  // No clue what this means copied from ModuleIO
  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Debouncer turnEncoderConnectedDebounce = 
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  
  private PivotTalonFXConfig config = new PivotTalonFXConfig();

  public PivotIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    pivotPosition = talon.getPosition();
    currentAmps = talon.getSupplyCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig =  new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = IntakePivotConstants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        IntakePivotConstants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);


    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, pivotPosition, turnAbsolutePosition);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    var pivotStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, pivotPosition);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.positionAngle = pivotPosition.getValue().in(Rotation);
    inputs.currentAmps = currentAmps.getValueAsDouble();

    inputs.absolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
  }

  @Override
  public void setPivotVoltage(double voltage) {
    // System.out.println(voltage);
    talon.setVoltage(voltage);
  }

  @Override
  public void setPosition(double positionAngle) {
    PositionVoltage mm = config.positionVoltage.withPosition(positionAngle);
      talon.setControl(mm);
  }

}
