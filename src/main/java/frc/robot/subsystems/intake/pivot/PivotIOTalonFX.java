 package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    
      talonConfig.Feedback.FeedbackRemoteSensorID = IntakePivotConstants.cancoderId;
      talonConfig.Feedback.RotorToSensorRatio = IntakePivotConstants.gearRatio;
      configGearRatio(1.0);
      configGravityType(IntakePivotConstants.gravityType);
   
      configPIDGains(
          IntakePivotConstants.p.get(), IntakePivotConstants.i.get(), IntakePivotConstants.d.get());
      configFeedForwardGains(
          IntakePivotConstants.s.get(), IntakePivotConstants.v.get(), 0.0, IntakePivotConstants.g.get());
     
      configMotionMagic(
          RotationsPerSecond.of(IntakePivotConstants.mmCruiseVelocity.get()),
          IntakePivotConstants.mmAcceleration.get(),
          0.0);
      configSupplyCurrentLimit(IntakePivotConstants.supplyLimit);

      talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
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

  
  private final SlewRateLimiter upRampLimiter = new SlewRateLimiter(12.0 / 0.15);

  private final Slot0Configs tunableGains = new Slot0Configs();

  private final MotionMagicConfigs tunableMM = new MotionMagicConfigs();

  public PivotIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    pivotPosition = talon.getPosition();
    currentAmps = talon.getSupplyCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    
    CANcoderConfiguration cancoderConfig =  new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = IntakePivotConstants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        IntakePivotConstants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);


    config.applyTalonConfig(talon);
    

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVoltage, currentAmps, pivotPosition, turnAbsolutePosition);
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

    updateTunableGains();
    updateTunableMotionMagic();
  }

  
  private void updateTunableGains() {
    double p = IntakePivotConstants.p.get();
    double i = IntakePivotConstants.i.get();
    double d = IntakePivotConstants.d.get();
    double s = IntakePivotConstants.s.get();
    double v = IntakePivotConstants.v.get();
    double g = IntakePivotConstants.g.get();
    if (p == tunableGains.kP
        && i == tunableGains.kI
        && d == tunableGains.kD
        && s == tunableGains.kS
        && v == tunableGains.kV
        && g == tunableGains.kG) {
      return;
    }
    tunableGains.kP = p;
    tunableGains.kI = i;
    tunableGains.kD = d;
    tunableGains.kS = s;
    tunableGains.kV = v;
    tunableGains.kG = g;
    talon.getConfigurator().apply(tunableGains);
  }

  
  private void updateTunableMotionMagic() {
    double cruise = IntakePivotConstants.mmCruiseVelocity.get();
    double accel = IntakePivotConstants.mmAcceleration.get();
    if (cruise == tunableMM.MotionMagicCruiseVelocity && accel == tunableMM.MotionMagicAcceleration) {
      return;
    }
    tunableMM.MotionMagicCruiseVelocity = cruise;
    tunableMM.MotionMagicAcceleration = accel;
    talon.getConfigurator().apply(tunableMM);
  }

  @Override
  public void setPivotVoltage(double voltage) {

    double output;
    if (voltage > 0) {
    
      output = voltage;
      upRampLimiter.reset(voltage);
    } else {
    
      output = upRampLimiter.calculate(voltage);
    }
    talon.setVoltage(output);
  }

  @Override
  public void setPosition(double positionAngle) {
    MotionMagicVoltage mm = config.mmPositionVoltage.withPosition(positionAngle);
    talon.setControl(mm);
  }

}
