package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetRollerConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase {
  private final RollerIO rollerIO;
  private final PivotIO pivotIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  
  private IntakeMode intakeMode;
  // private IntakePivotModes pivotMode;
  

  public Intake(RollerIO rollerIO, PivotIO pivotIO) {
    this.rollerIO  = rollerIO;
    this.pivotIO = pivotIO;
    this.intakeMode = IntakeMode.STOW;
  }
  
  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
    
    Logger.recordOutput("Intake/Mode", intakeMode);
  }

  public void runIntakeEnum(IntakeMode intakeMode) {
    this.intakeMode = intakeMode;
    rollerIO.setRollerVoltage(intakeMode.voltage.baseUnitMagnitude());
    // pivotIO.setPosition(intakeMode.position.magnitude());
  }

  public Command runIntakeCommand(IntakeMode intakeMode) {
    return new RunCommand(() -> {
      this.runIntakeEnum(intakeMode);
    }, this).withName("Intake.runIntakeCommand" + intakeMode.toString());
  }

  /** Closed-loop hold of the pivot at {@code positionRotations} (mechanism rotations). */
  public Command runPivotPositionCommand(double positionRotations) {
    return new RunCommand(() -> pivotIO.setPosition(positionRotations), this)
        .withName("Intake.pivotPosition" + positionRotations);
  }

  /** Closed-loop hold of the pivot at a live target (re-read each loop, e.g. a tunable NT value). */
  public Command runPivotPositionCommand(DoubleSupplier positionRotations) {
    return new RunCommand(() -> pivotIO.setPosition(positionRotations.getAsDouble()), this)
        .withName("Intake.pivotPositionLive");
  }

  public Command runPivotVoltageCommand(double voltage){
    return runEnd(
            () -> pivotIO.setPivotVoltage(voltage),
            () -> pivotIO.setPivotVoltage(0))
        .withName("pivotVoltage" + voltage);
  }

  /**
   * Runs the roller at {@code mode} while pulsing the pivot up/down on a fixed period. Both the
   * roller and pivot live on this one subsystem, so they must be driven from a single command
   * (you cannot run them as two parallel commands that each require Intake).
   */
  public Command runIntakePivotPulseCommand(
      IntakeMode mode, double upVoltage, double downVoltage, double periodSeconds) {
    Timer timer = new Timer();
    double half = periodSeconds / 2.0;
    return runEnd(
            () -> {
              runIntakeEnum(mode);
              pivotIO.setPivotVoltage((timer.get() % periodSeconds) < half ? upVoltage : downVoltage);
            },
            () -> {
              rollerIO.setRollerVoltage(0);
              pivotIO.setPivotVoltage(0);
            })
        .beforeStarting(timer::restart)
        .withName("Intake.runIntakePivotPulse" + mode);
  }

  public Command runIntakeTuneCommand(){
    return Commands.run(() -> AngularVelocity.ofRelativeUnits(CarpetRollerConstants.tuneDesiredSpeed.get(), RPM));
  
  }

  public Command stop() {
    // return new RunCommand(() -> {
    //   this.pivotIO.setPivotVoltage(0);
    //   this.pivotIO.setPivotVoltage(0);
    // }, this).withName("Intake.stopAll");
    
    return run(() -> {
      rollerIO.setRollerVoltage(0);
      // pivotIO.setPivotVoltage(0);
    }).withName("Intake.Stop");

  }
}