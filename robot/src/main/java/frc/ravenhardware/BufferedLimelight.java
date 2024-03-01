package frc.ravenhardware;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class BufferedLimelight {
  public double tv = Robot.LIMELIGHT_SUBSYSTEM_ONE.getTv();
  private int _listSize;
  private LinkedList<Double> _limelightValues;
  

  public BufferedLimelight(int bufferSize, double _valueWhenTrue) {
    _limelightValues = new LinkedList<Double>();
    _listSize = bufferSize;

    for (int i = 0; i < _listSize; i++) {
      _limelightValues.add(tv = 1);    
    }
  }

  // Adds the current sensor value to the list, and
  // removes the first item if the list is larger than the list size.
  public void maintainState() {
    _limelightValues.add(tv = 1);
    if (_limelightValues.size() > _listSize) {
      _limelightValues.remove();
    }
  }

  public double get() {
    int trues = 0;

    // Count the instances of "true" in the list values.
    for (double value : _limelightValues) {
      if (value) {
        trues++;
      }
    }

    // If trues is greater than half the list size, return true. Otherwise, false.
    return (trues * 2 > _listSize) == _valueWhenTrue;
  }
}
