package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class UnjamCommand extends ParallelCommandGroup {
    public UnjamCommand(Shooter shooter){
        addCommands(
            shooter.runCustomVoltageCommand(-4, -4)
        );
        addRequirements(shooter);
    }
}
