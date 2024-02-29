package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonCode {

	private Joystick OP_PAD_BUTTONS = new Joystick(0);
	private Joystick OP_PAD_TOGGLE = new Joystick(0);

	public enum Buttons {
		GROUND_PICKUP_AND_SPEAKER_SCORING(1),
		AMP_SCORING(2),
		TRAP_SCORING(3),
		AMP_AND_SPEAKER_SOURCE_INTAKE(4),
		TRAP_SOURCE_INTAKE(8),
		DEFENDED_SPEAKER_SCORING(9);


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
		ARM_UP(1);
	
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