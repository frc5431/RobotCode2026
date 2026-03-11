package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import static frc.robot.util.SparkUtil.*;

import frc.team5431.titan.core.subsystem.REVMechanism;

public class ClimberIOSparkFlex implements ClimberIO {
  private final SparkFlex sparkFlex = new SparkFlex(ClimberConstants.id, MotorType.kBrushless);
  private final RelativeEncoder encoder = sparkFlex.getEncoder();

  public static class PivotSparkFlexConfig extends REVMechanism.Config {
    public PivotSparkFlexConfig() {
      super("PivotSparkFlex", ClimberConstants.id);
      configPIDGains(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
      configFeedbackSensorSource(ClimberConstants.feedbackSensorREV);
      // configGear(ClimberConstants.gearRatio);
      // configGravity(ClimberConstants.gravityType);
      configSmartCurrentLimit(ClimberConstants.stallLimit, ClimberConstants.supplyLimit);
      configSmartStallCurrentLimit(ClimberConstants.stallLimit);
      configReverseSoftLimit(
          ClimberConstants.maxReverseRotation, ClimberConstants.useRMaxRotation);
      configForwardSoftLimit(
        ClimberConstants.maxFowardRotation, ClimberConstants.useFMaxRotation);
    }
  } 

  public ClimberIOSparkFlex() {
    sparkFlex.configure(new PivotSparkFlexConfig().sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(sparkFlex, encoder::getPosition, (value) -> inputs.positionAngle = value);
    ifOk(sparkFlex, sparkFlex::getBusVoltage, (value) -> inputs.appliedVoltage = value);
    ifOk(sparkFlex, sparkFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setClimberPosition(double positionAngle) {
    sparkFlex.getClosedLoopController().setSetpoint((positionAngle), ControlType.kPosition,
            ClosedLoopSlot.kSlot0);
  }  
}
