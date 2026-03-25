package frc.robot.commands;



import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class ShootFuelCommandAuto extends SequentialCommandGroup {
  public ShootFuelCommandAuto(Shooter shooter, Vision vision, DoubleSupplier dist) {
    addCommands(
      vision.visionEnable(),

      shooter.runShootAuto(dist).withName("ShootFuelCommand.ShootAuto").
        until(shooter.atSetpoint()).withTimeout(2),
      new ParallelCommandGroup(
       shooter.runShootAuto(dist).withName("ShootFuelCommand.ShootAuto")
    )
    //  Commands.runOnce(vision.visionDisable(), vision);

  );  
      
    addRequirements(shooter, vision);
    // this.beforeStarting(vision.visionEnable()).finallyDo((interupted) -> vision.visionDisable());
  }

  //TODO: once shooter hits rpm run feeder
}
