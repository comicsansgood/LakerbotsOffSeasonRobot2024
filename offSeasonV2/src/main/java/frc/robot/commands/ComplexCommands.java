package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.feeder;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.launcher;

public class ComplexCommands {

    private static feeder m_feeder = new feeder();
    private static intake m_intake = new intake();
    private static launcher m_launcher = new launcher();

    private ComplexCommands(){}

    public static Command intakeCommand(){
        return Commands.sequence(
            m_feeder.feederIntakeSequence().raceWith(m_intake.intakeGo(0.1)),
            m_intake.intakeGo(0)
        );
    }

    public static Command launchCommand(){
        return Commands.sequence(
            m_launcher.launcherUptoSpeed(Constants.LauncherConstants.launcherSpeed, Constants.LauncherConstants.launcherSpeed),
            m_feeder.feederGo(0.1).withTimeout(0.5),
            m_launcher.launcherSetVelocity(0, 0),
            m_feeder.feederGo(0)
        );
    }


}
