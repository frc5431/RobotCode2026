package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants.FeederModes;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;


public class InhaleCommand extends ParallelCommandGroup {
  
  
  public InhaleCommand(Intake intake, Carpet carpet, Feeder feeder, boolean isInhaling) {
     if (isInhaling) {
      addCommands(
        intake.runIntakeCommand(IntakeMode.INTAKE),
        carpet.runCarpetCommand(CarpetModes.INTAKE),
        feeder.runFeederCommand(FeederModes.FEEDER)
      );
     } else {
      addCommands(
        intake.runIntakeCommand(IntakeMode.OUTTAKE),
        carpet.runCarpetCommand(CarpetModes.OUTTAKE),
        feeder.runFeederCommand(FeederModes.REVERSE)
      );
     }
    addRequirements(intake, carpet, feeder);
    }

}