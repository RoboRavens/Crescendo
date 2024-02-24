package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Gamepad {
	private Joystick _joystick;

	public Gamepad(Joystick joystick) {
		_joystick = joystick;
	}

	public Gamepad(int port) {
		_joystick = new Joystick(port);
	}

	public boolean getButtonValue(ButtonCode button) {
		return _joystick.getRawButton(getButtonNumber(button));
	}

	public JoystickButton getButton(ButtonCode button) {
		return new JoystickButton(_joystick, getButtonNumber(button));
	}

	public double getAxis(AxisCode axis) {
		double axisValue;

		switch (axis) {
		case LEFTSTICKX:
			axisValue = _joystick.getRawAxis(0);
			break;
		case LEFTSTICKY:
			axisValue = _joystick.getRawAxis(1);
			break;
		case LEFTTRIGGER:
			axisValue = _joystick.getRawAxis(2);
			break;
		case RIGHTTRIGGER:
			axisValue = _joystick.getRawAxis(3);
			break;
		case RIGHTSTICKX:
			axisValue = _joystick.getRawAxis(4);
			break;
		case RIGHTSTICKY:
			axisValue = _joystick.getRawAxis(5);
			break;
		default:
			axisValue = 0;
			break;
		}
		return axisValue;
	}

	public int getButtonNumber(ButtonCode button) {
		int buttonNumber;

		switch (button) {
		case SCORE_SPEAKER:
			buttonNumber = 1;
			break;
		case SCORE_AMP:
			buttonNumber = 2;
			break;
		case SCORE_TRAP:
			buttonNumber = 3;
			break;
		case SCORE_CLIMB:
			buttonNumber = 4;
			break;
		case INTAKE_GROUND:
			buttonNumber = 5;
			break;
		case INTAKE_SOURCE:
			buttonNumber = 6;
			break;
		case INTAKE_TRAP_SOURCE:
			buttonNumber = 7;
			break;
		case SIGNAL_CO_OP:
			buttonNumber = 8;
			break;
		case SIGNAL_AMP:
			buttonNumber = 9;
			break;
		case SOURCE_LANE_LEFT:
			buttonNumber = 10;
			break;
		case SOURCE_LANE_CENTER:
			buttonNumber = 11;
			break;
		case SOURCE_LANE_RIGHT:
			buttonNumber = 12;
			break;
		case TEMPORARY_ARM_UP:
			buttonNumber = 13;
			break;
		default:
			throw new IllegalArgumentException("Button code " + button.name() + " was not setup correctly.");
		}

		return buttonNumber;
	}

	public void setRumbleOn() {
        _joystick.setRumble(RumbleType.kLeftRumble, 1);
        _joystick.setRumble(RumbleType.kRightRumble, 1);
	}

	public void setRumbleOff() {
		_joystick.setRumble(RumbleType.kLeftRumble, 0);
        _joystick.setRumble(RumbleType.kRightRumble, 0);
	}

	public boolean getPOV(GamepadPOV pov) {
		return _joystick.getPOV() == pov.Angle;
	}

	public Trigger getPOVTrigger(GamepadPOV pov) {
		return new Trigger(() -> this.getPOV(pov));
	}
}