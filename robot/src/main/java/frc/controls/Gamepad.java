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
		case A:
			buttonNumber = 1;
			break;
		case B:
			buttonNumber = 2;
			break;
		case X:
			buttonNumber = 3;
			break;
		case Y:
			buttonNumber = 4;
			break;
		case LEFTBUMPER:
			buttonNumber = 5;
			break;
		case RIGHTBUMPER:
			buttonNumber = 6;
			break;
		case BACK:
			buttonNumber = 7;
			break;
		case START:
			buttonNumber = 8;
			break;
		case LEFTSTICK:
			buttonNumber = 9;
			break;
		case RIGHTSTICK:
			buttonNumber = 10;
			break;
		// Button locations will need to be tested and changed.
		case SUBSTATION_INTAKE:
			buttonNumber = 9;
			break;
		case SET_ARM_TO_SCORE_TARGET_STATE:
			buttonNumber = 2;
			break;
		case RETRACT_ARM_FULL:
			buttonNumber = 1;
			break;
		case EJECT_PIECE:
			buttonNumber = 8;
			break;
		case EXTEND_ARM:
			buttonNumber = 11;
			break;
		case RETRACT_ARM:
			buttonNumber = 9;
			break;
		case ROTATE_ARM_FORWARD:
			buttonNumber = 12;
			break;
		case ROTATE_ARM_BACKWARD:
			buttonNumber = 10;
			break;
		// Switch buttons
		case ARM_EXTENSION_MANUAL_OVERRIDE:
			buttonNumber = 2;
			break;
		case ARM_ROTATION_MANUAL_OVERRIDE:
			buttonNumber = 4;
			break;
		case ENDGAME_OVERRIDE:
			buttonNumber = 1;
			break;
		case IGNORE_ROTATION_LIMITS:
			buttonNumber = 8;
			break;
		case IGNORE_EXTENSION_LIMITS:
			buttonNumber = 6;
			break;
		case ODOMETRY_OVERRIDE:
			buttonNumber = 7;
			break;
		case PIECE_SENSOR_OVERRIDE:
			buttonNumber = 5;
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