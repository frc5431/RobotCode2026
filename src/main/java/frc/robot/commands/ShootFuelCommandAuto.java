package frc.robot.commands;



import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFuelCommandAuto extends SequentialCommandGroup {
  public ShootFuelCommandAuto(Shooter shooter, DoubleSupplier dist) {
    addCommands(
      shooter.runShootAuto(dist).withName("ShootFuelCommand.ShootAuto").
        until(shooter.atSetpoint()).withTimeout(2),
      new ParallelCommandGroup(
       shooter.runShootAuto(dist).withName("ShootFuelCommand.ShootAuto")
    ));  
      
    addRequirements(shooter);
  }

  //TODO: once shooter hits rpm run feeder
}
