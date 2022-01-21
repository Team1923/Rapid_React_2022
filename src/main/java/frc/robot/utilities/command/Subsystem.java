package frc.robot.utilities.command;

public abstract class Subsystem implements edu.wpi.first.wpilibj2.command.Subsystem {
    protected Subsystem() {
        this.register();
    }

    public abstract void stop();
}
