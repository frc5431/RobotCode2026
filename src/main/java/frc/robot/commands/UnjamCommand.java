package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants.FeederModes;
import frc.robot.subsystems.shooter.Shooter;

public class UnjamCommand extends ParallelCommandGroup {
    public UnjamCommand(Feeder feeder, Shooter shooter){
        addCommands(
            feeder.runFeederCommand(FeederModes.REVERSE),
            shooter.runShooterVoltageCommand(-4)
        );
        addRequirements(feeder, shooter);
    }
}
