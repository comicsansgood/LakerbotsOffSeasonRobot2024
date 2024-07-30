// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  public final DigitalInput noteSensor = new DigitalInput(1);
  // private static final double feederVoltage = 8.0;

  /** Creates a new Flywheel. */
  public Feeder(FeederIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public Command feederCommand(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    return startEnd(
        () -> {
          io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
        },
        () -> {
          io.setVoltage(0);
        });
  }
  ;

  public Command feederGo(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    return runOnce(
        () -> {
          io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
        });
  }
  ;

  /*
   public FunctionalCommand feederRunUntilDetecteCommand(double velocityRPM){
     var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
     return FunctionalCommand(
       ()-> io.stop(),
        () -> {
           io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
         }
       ,
       () -> {
           io.setVelocity(0, ffModel.calculate(velocityRadPerSec));
         }
       ,
       ()-> {noteSensor.get();
        }
     );
   }
  */
  public Command intakeTest(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    return Commands.sequence(
        runOnce(
            () -> {
              io.setVelocity(velocityRPM, ffModel.calculate(velocityRadPerSec));
            }),
        Commands.waitSeconds(1.0),
        runOnce(
            () -> {
              io.setVelocity(0, ffModel.calculate(0));
            }),
        Commands.waitSeconds(1.0),
        runOnce(
            () -> {
              io.setVelocity(velocityRPM, ffModel.calculate(velocityRadPerSec));
            }),
        Commands.waitSeconds(1.0),
        runOnce(
            () -> {
              io.setVelocity(0, ffModel.calculate(0));
            }));
  }

  public Command intakeTest2(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    return Commands.sequence(
        runOnce(
            () -> {
              io.setVelocity(velocityRPM, ffModel.calculate(velocityRadPerSec));
            }),
        Commands.waitUntil(() -> isNoteDetected()),
        runOnce(
            () -> {
              io.setVelocity(250, ffModel.calculate(0));
            }),
        // Commands.waitSeconds(0.25),
        Commands.waitUntil(() -> isNoteNotDetected()),
        runOnce(
            () -> {
              io.setVelocity(-50, ffModel.calculate(0));
            }),

        // run once with negative
        // wait
        Commands.waitSeconds(0.5),
        runOnce(
            () -> {
              io.setVelocity(0, ffModel.calculate(0));
            }));
  }

  /*
  intake until sensor is tripped
  keep intaking until sensor is not tripped
  back off a little bit
  */

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean isNoteDetected() {
    return !noteSensor.get();
  }

  public boolean isNoteNotDetected() {
    return noteSensor.get();
  }
  // is note not detected create a boolean
}
