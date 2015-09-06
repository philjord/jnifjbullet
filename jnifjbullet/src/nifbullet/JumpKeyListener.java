package nifbullet;

import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

import com.bulletphysics.dynamics.character.KinematicCharacterController;

import nifbullet.NavigationProcessorBullet.NbccProvider;

public class JumpKeyListener extends KeyAdapter
{
	private long jumpKeyDownTime = -1;

	private NbccProvider clientPhysicsSystem;

	public JumpKeyListener(NbccProvider clientPhysicsSystem)
	{
		this.clientPhysicsSystem = clientPhysicsSystem;
	}

	@Override
	public void keyReleased(KeyEvent event)
	{
		if (event.getKeyCode() == KeyEvent.VK_SPACE)
		{
			long timePressed = System.currentTimeMillis() - jumpKeyDownTime;
			float multi = (float) Math.min(timePressed / 750d, 0.75);
			multi += 0.25;

			if (clientPhysicsSystem.getNBControlledChar() != null)
			{
				KinematicCharacterController kcc = clientPhysicsSystem.getNBControlledChar().getCharacterController();
				if (kcc.canJump())
				{
					kcc.jump(multi);
				}
			}

			jumpKeyDownTime = -1;

		}
	}

	@Override
	public void keyPressed(KeyEvent e)
	{
		if (e.getKeyCode() == KeyEvent.VK_SPACE)
		{
			if (jumpKeyDownTime == -1)
			{
				jumpKeyDownTime = System.currentTimeMillis();
			}
		}
	}
}
