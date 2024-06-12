package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;

public class NoteDetectCommand extends Command {

  private final Feeder feeder;
  private final FeederIO io;
  private final SimpleMotorFeedforward ffModel;

  public NoteDetectCommand(Feeder feeder, FeederIO io, SimpleMotorFeedforward ffModel) {
    this.feeder = feeder;
    this.io = io;
    this.ffModel = ffModel;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(1500);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    // do we need the {}-> thing?
  }

  @Override
  public void end(boolean interrupted) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    // do we need the {}-> thing?

  }

  @Override
  public boolean isFinished() {
    return feeder.isNoteDetected();
  }
}
