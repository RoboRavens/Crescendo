package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;

public class ButtonCode {

	private Joystick OP_PAD_BUTTONS = new Joystick(RobotMap.BUTTONS_CONTROLLER_PORT);
	private Joystick OP_PAD_TOGGLE = new Joystick(RobotMap.OVERRIDES_CONTROLLER_PORT);

	public enum Buttons {
		GROUND_PICKUP_AND_SPEAKER_SCORING(1),
		DEFENDED_SPEAKER_SCORING(2),
		AMP_SCORING(3),
		TRAP_SCORING(4),
		AMP_AND_SPEAKER_SOURCE_INTAKE(8),
		TRAP_SOURCE_INTAKE(9),
		MOVE_ELBOW_UP(7),
		MOVE_ELBOW_DOWN(10),
		MOVE_WRIST_UP(6),
		MOVE_WRIST_DOWN(11);

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
		START_SHOOTER(2);

		private final int toggle;

		Toggle(int toggle) {
			this.toggle = toggle;
		}

		public int getToggle() {
			return toggle;
		}
	}

	public JoystickButton getButton(Buttons button) {
		return new JoystickButton(OP_PAD_BUTTONS, button.getButton());
	}

	public JoystickButton getSwitch(Toggle toggle) {
		return new JoystickButton(OP_PAD_TOGGLE, toggle.getToggle());
	}
}