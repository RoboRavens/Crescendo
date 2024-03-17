package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap;

public class ButtonCode {

	private boolean _buttonPanelEnabled = false;
	private Joystick OP_PAD_BUTTONS;
	private Joystick OP_PAD_TOGGLE;

	public ButtonCode(){
		if (_buttonPanelEnabled) {
			OP_PAD_BUTTONS = new Joystick(RobotMap.BUTTONS_CONTROLLER_PORT);
			OP_PAD_TOGGLE = new Joystick(RobotMap.OVERRIDES_CONTROLLER_PORT);
		}
	}

	public enum Buttons {
		GROUND_PICKUP_AND_SPEAKER_SCORING(1),
		DEFENDED_SPEAKER_SCORING(2),
		AMP_SCORING(3),
		// TRAP_SCORING(4),
		SOURCE_INTAKE(8),
		// TRAP_SOURCE_INTAKE(9),
		MOVE_ELBOW_UP(7),
		MOVE_ELBOW_DOWN(10),
		MOVE_WRIST_UP(6),
		MOVE_WRIST_DOWN(11),
		SPEAKER_CLOSE_SHOT(12),
		// SPEAKER_MID_SHOT(5),
		SPEAKER_FAR_SHOT(4),
		SHOOTER_REV(5),
		ARM_RELEASE_SNAPPER(9);

		private final int button;

		Buttons(int button) {
			this.button = button;
		}

		public int getButton() {
			return button;
		}
	}

	// "toggle" is in place for switch
	public enum Toggle {
		ARM_UP(1),
		START_SHOOTER(2), 
		SHOOTER_ANGLE_FROM_DISTANCE(3),
		MOVE_WITH_MANUAL_POWER(4);

		private final int toggle;

		Toggle(int toggle) {
			this.toggle = toggle;
		}

		public int getToggle() {
			return toggle;
		}
	}

	public Trigger getButton(Buttons button) {
		if (_buttonPanelEnabled) {
			return new JoystickButton(OP_PAD_BUTTONS, button.getButton()); 
		}

		return new Trigger(() -> false);
	}

	public Trigger getSwitch(Toggle toggle) {
		if (_buttonPanelEnabled) {
			return new JoystickButton(OP_PAD_TOGGLE, toggle.getToggle());
		}
		
		return new Trigger(() -> false);
	}
}