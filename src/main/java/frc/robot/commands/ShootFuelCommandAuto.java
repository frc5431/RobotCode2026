package frc.robot.commands;



import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants.FeederModes;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFuelCommandAuto extends SequentialCommandGroup {
  public ShootFuelCommandAuto(Feeder feeder, Shooter shooter, DoubleSupplier dist) {
    addCommands(
      shooter.runShootAuto(dist.getAsDouble()).withName("ShootFuelCommand.ShootAuto").
        until(shooter.atSetpoint()).withTimeout(2),
      new ParallelCommandGroup(
        feeder.runFeederCommand(FeederModes.FEEDER).withName("FeederCommand.Feed"),
       shooter.runShootAuto(dist.getAsDouble()).withName("ShootFuelCommand.ShootAuto")
    ));  
      
    addRequirements(feeder, shooter);
  }

  //TODO: once shooter hits rpm run feeder
}
