// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.intake.Intake;

// public class IntakeJiggleCommand extends SequentialCommandGroup {
//     public IntakeJiggleCommand(Intake intake) {
//         addCommands(
//             new ParallelDeadlineGroup(new WaitCommand(.1), intake.runPivotVoltageCommand(-5)),
//             new ParallelDeadlineGroup(new WaitCommand(.1), intake.runPivotVoltageCommand(3)),
//             new ParallelDeadlineGroup(new WaitCommand(.1)), intake.runPivotVoltageCommand(-5)),
//             new ParallelDeadlineGroup(new WaitCommand(.1), intake.runPivotVoltageCommand(3))
//         );

//         addRequirements(intake);
//     }
// }
