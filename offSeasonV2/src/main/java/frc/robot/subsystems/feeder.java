
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class feeder extends SubsystemBase {

  public CANSparkMax feederMotor = new CANSparkMax(12, MotorType.kBrushless);
  public final DigitalInput noteSensor = new DigitalInput(1);


  public feeder() {
    feederMotor.restoreFactoryDefaults();
    feederMotor.setCANTimeout(250);
    feederMotor.setInverted(false);
    feederMotor.enableVoltageCompensation(12.0);
    feederMotor.setSmartCurrentLimit(30);
    feederMotor.burnFlash();
   
    
  }

  public Command feederGo(double speed) {
    return runOnce(() -> feederMotor.set(speed));
  }

  public boolean isNoteDetected(){
    return !noteSensor.get();
  }

  public Command feederGoUntilNoteDetected(){
    return runOnce(()-> {
      feederGo(.10).andThen(Commands.waitUntil(() -> isNoteDetected()));
  });
  }

  public Command feederGoUntilNoteNotDetected(){
    return runOnce(()-> {
      feederGo(.10).andThen(Commands.waitUntil(() -> !isNoteDetected()));
  });
  }

  public Command feederIntakeSequence(){
  
    return Commands.sequence(
      feederGoUntilNoteDetected(), 
      feederGo(0), 
      feederGoUntilNoteNotDetected(), 
      feederGo(-0.1), 
      Commands.waitSeconds(0.5), 
      feederGo(0)
    );
  }

  public Command exampleMethodCommand() {

    return runOnce(
        () -> {
        });
  }

  
 
  

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
