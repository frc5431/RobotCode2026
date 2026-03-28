package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;


public class InhaleCommand extends ParallelCommandGroup {
  
  
  public InhaleCommand(Intake intake, Carpet carpet, boolean isInhaling) {

    
     if (isInhaling) {
      addCommands(
        intake.runIntakeCommand(IntakeMode.INTAKE),
        carpet.runCarpetRPM(Units.RPM.of(6000))
      );
     } 
     
     else {
      addCommands(
        intake.runIntakeCommand(IntakeMode.OUTTAKE),
        carpet.runCarpetCommand(CarpetModes.OUTTAKE)
      );
     }
    addRequirements(intake, carpet);
    }

}