package frc.robot.subsystems.drive;

public class Module {
    private final ModuleIO io;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public Module(ModuleIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }
}