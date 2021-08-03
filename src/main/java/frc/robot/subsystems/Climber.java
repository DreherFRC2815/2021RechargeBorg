package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
  private Compressor c;
  private DoubleSolenoid s;
  private DoubleSolenoid s2;

    public Climber() {
      c = new Compressor(0);
      c.start();
      s = new DoubleSolenoid(0, 0, 1);
      s2 = new DoubleSolenoid(0, 2, 3);
    }
    
    public void set(int control) {
      s.set(Value.kReverse);
    }

    public void init() {
      s2.set(Value.kOff);
      s.set(Value.kForward);
      s.toggle();
      print();
    }

    public void print() {
      if (s.get().equals(Value.kForward))  {
        SmartDashboard.putBoolean("climber up?", true);
      }else {
        SmartDashboard.putBoolean("climber up?", false);
      }
    }
    
    @Override
  public void periodic() {
  // This method will be called once per scheduler run
  }

	public void toggle() {
    if (s.get().equals(Value.kOff)) {
      s.set(Value.kForward);
    }else {
      s.toggle();
    }
    print();
  }

  public void switchPosition() {
    if (s2.get().equals(Value.kOff)) {
      s2.set(Value.kForward);
    }
    s2.toggle();
  }
  
  public void setUp() {
    s.set(Value.kForward);
  }
  
  public void setDown() {
    s.set(Value.kReverse);
  }
}