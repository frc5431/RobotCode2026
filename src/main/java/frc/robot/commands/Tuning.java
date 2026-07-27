package frc.robot.commands;



import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants;
public class Tuning {



  
  public static Command everything(
    Shooter shooter, 
    Carpet carpet, 
    Intake intake) 
    
    {
    
        return Commands.parallel(
        
        shooter.runShooterCustom(Constants.TuningMode_Shooter_RPM::getAsDouble, Constants.Feeder_RPM::getAsDouble)
            .onlyWhile(Constants.ShooterEnabled::getAsBoolean),
        
        carpet.runCarpetRPM(Constants.Carpet_RPM::getAsDouble)
            .onlyWhile(Constants.CarpetEnabled::getAsBoolean),
        
        intake.runIntakeRPM(Constants.Intake_RPM::getAsDouble)
            .onlyWhile(Constants.IntakeEnabled::getAsBoolean)
    
    ).onlyWhile(Constants.TuningMode::getAsBoolean);
  
}
}
