package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederConstants.FeederModes;
import lombok.Getter;
import lombok.Setter;

public class Feeder extends SubsystemBase {

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

    @Getter @Setter private FeederModes feederMode;

    public Feeder(FeederIO feederIO) {
        this.feederIO = feederIO;
        this.feederMode = FeederModes.IDLE;
    }

    @Override
    public void periodic() {
        feederIO.updateInputs(feederInputs);
        Logger.processInputs("Feeder", feederInputs);
        Logger.recordOutput("Feeder/Mode", feederMode);
    }

    public void runFeederEnum(FeederModes feederMode) {
        this.feederMode = feederMode;
        feederIO.setFeederVoltage(feederMode.output);
    }

    public Command runFeederCommand(FeederModes feederMode) {
        return new RunCommand(() -> runFeederEnum(feederMode), this).withName("Feeder.runEnum" + feederMode.toString());
    }
}
