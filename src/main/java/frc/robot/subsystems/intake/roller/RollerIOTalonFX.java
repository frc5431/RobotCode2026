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
import frc.robot.Constants.RollerIOConstants;
import frc.robot.Constants.RollerIOConstants.RollerIOModes;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon = new TalonFX(RollerIOConstants.id, Constants.CANBUS);

  public static class RollerTalonFXConfig extends CTREMechanism.Config {
    public RollerTalonFXConfig() {
      super("Intake", RollerIOConstants.id, Constants.CANBUS);
      configPIDGains(RollerIOConstants.p, RollerIOConstants.i, RollerIOConstants.d);
      configNeutralBrakeMode(RollerIOConstants.breakType);
      configFeedbackSensorSource(RollerIOConstants.feedbackSensor);
      configGearRatio(RollerIOConstants.gearRatio);
      configGravityType(RollerIOConstants.gravityType);
      configSupplyCurrentLimit(RollerIOConstants.supplyLimit, RollerIOConstants.useSupplyLimit);
      configStatorCurrentLimit(RollerIOConstants.stallLimit, RollerIOConstants.useStallLimit);
      configReverseSoftLimit(
          RollerIOConstants.maxReverseRotation.in(Rotation), RollerIOConstants.useRMaxRotation);
      configForwardSoftLimit(
          RollerIOConstants.maxFowardRotation.in(Rotation), RollerIOConstants.useFMaxRotation);
    }
  }

  private RollerIOModes mode;
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<AngularVelocity> rollerRPM;
  private StatusSignal<Double> rollerOutput;

  private RollerTalonFXConfig config;

  public RollerIOTalonFX(TalonFX talon) {
    this.mode = RollerIOModes.IDLE;
    appliedVoltage = talon.getMotorVoltage();
    rollerRPM = talon.getVelocity();
    rollerOutput = talon.getClosedLoopOutput();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(40, appliedVoltage, rollerOutput, rollerRPM);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVoltage, rollerOutput, rollerRPM);

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.output = rollerOutput.getValue();
    inputs.rpm = rollerRPM.getValue().in(RPM);
  }

  @Override
  public void setOutput(double output) {
    talon.set(output);
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

  // public void runEnum(RollerIOModes rollerIOModes, boolean rpm) {
  //     this.mode = rollerIOModes;
  //     if (rpm) {
  //         setVelocity(rollerIOModes.speed);
  //     } else {
  //         setPercentOutput(rollerIOModes.output);
  //     }
  // }

  // public Command runIntakeCommand(RollerIOModes rollerIOModes) {
  //     return new RunCommand(() -> this.runEnum(rollerIOModes, RollerIOConstants.useRpm), this)
  //             .withName("Intake.runEnum");
  // }

  // public Command runIntakeCommand(RollerIOModes rollerIOModes, boolean rpm) {
  //     return new RunCommand(() -> this.runEnum(rollerIOModes, rpm),
  // this).withName("Intake.runEnum");
  // }

}
