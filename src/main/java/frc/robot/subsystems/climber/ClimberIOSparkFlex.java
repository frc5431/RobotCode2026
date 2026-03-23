package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkFlex;

import static frc.robot.util.SparkUtil.*;

public class ClimberIOSparkFlex implements ClimberIO {
  private final SparkFlex sparkFlex = new SparkFlex(ClimberConstants.id, MotorType.kBrushless);
  private final RelativeEncoder encoder = sparkFlex.getEncoder();
  private final SparkFlexConfig config = new SparkFlexConfig();

  public ClimberIOSparkFlex() {
    // CONFIG
    config.closedLoop.feedbackSensor(ClimberConstants.feedbackSensorREV);
    config.smartCurrentLimit(
      (int) ClimberConstants.stallLimit.in(Units.Amps), (int) ClimberConstants.supplyLimit.in(Units.Amps));
    config.closedLoop.pid(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d, ClosedLoopSlot.kSlot0);
    config.softLimit.reverseSoftLimit(ClimberConstants.maxReverseRotation.in(Units.Rotations));
    config.softLimit.reverseSoftLimitEnabled(true);
    config.softLimit.forwardSoftLimit(ClimberConstants.maxReverseRotation.in(Units.Rotations));
    config.softLimit.forwardSoftLimitEnabled(true);
    
    sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(sparkFlex, encoder::getPosition, (value) -> inputs.positionAngle = value);
    ifOk(sparkFlex, sparkFlex::getBusVoltage, (value) -> inputs.appliedVoltage = value);
    ifOk(sparkFlex, sparkFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  // @Override
  // public void setClimberPosition(double positionAngle) {
  //   sparkFlex.getClosedLoopController().setSetpoint((positionAngle), ControlType.kPosition,
  //           ClosedLoopSlot.kSlot0);
  // }  
}
