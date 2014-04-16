package nifbullet;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.dynamics.character.KinematicCharacterController3;

import nifbullet.cha.NBControlledChar;
import tools3d.navigation.AvatarLocation;
import tools3d.navigation.NavigationProcessorInterface;
import tools3d.utils.YawPitch;

/**
 * 
 * Note another object must call process on a regular basis for the processor to run this is normally
 * NavigationTemporalBehaviour, but could be a simple thread
 * 
 * @author Administrator
 * @param <KCCProvider>
 * 
 */
public class NavigationProcessorBullet implements NavigationProcessorInterface
{
	/** The source and destination for transform changes */
	private AvatarLocation avatarLocation;

	// NOTE: rotattion per sec over rides absolute rotation
	/** The amount to move the view in mouse coords up/down per second */
	private float rotationYPerSec = 0;

	/** The amount to move the view in mouse coords left/right per second */
	private float rotationXPerSec = 0;

	/** The absolute rotation up/down */
	private float rotationY = 0;

	/** The absolute rotation left/right */
	private float rotationX = 0;

	/** The amount to translate the view in coords per axis per second */
	private float zChangePerSec = 0; // move

	private float xChangePerSec = 0; // straf

	private float yChangePerSec = 0; // float

	private boolean active = false;

	private boolean noPitch = false;

	private NbccProvider nbccProvider;

	public NavigationProcessorBullet(NbccProvider nbccProvider, AvatarLocation avatarLocation)
	{
		this.nbccProvider = nbccProvider;
		this.avatarLocation = avatarLocation;
	}

	public boolean isNoPitch()
	{
		return noPitch;
	}

	public void setNoPitch(boolean noPitch)
	{
		this.noPitch = noPitch;
	}

	@Override
	public void setZChange(float zChangeMulti)
	{
		zChangePerSec = zChangeMulti;
	}

	@Override
	public void setXChange(float xChangeMulti)
	{
		xChangePerSec = xChangeMulti;
	}

	@Override
	public void setYChange(float yChangeMulti)
	{
		yChangePerSec = yChangeMulti;
	}

	@Override
	public void setRotationPerSec(float newRotationX, float newRotationY)
	{
		this.rotationXPerSec = newRotationX;
		this.rotationYPerSec = newRotationY;

		this.rotationX = Float.MIN_VALUE;
		this.rotationY = Float.MIN_VALUE;
	}

	@Override
	public void changeRotation(double addRotationX, double addRotationY)
	{
		this.rotationX += addRotationX;
		this.rotationY += addRotationY;
	}

	// deburners
	/** starting trans */
	private Vector3f avatarTranslation = new Vector3f();

	/** starting rotation*/
	private Quat4f avatarRot = new Quat4f();

	/** A working value for the current frame's translation of the eye */
	private Vector3f oneFrameTranslation = new Vector3f();

	/** a temp   */
	private YawPitch tempYawPitch = new YawPitch();

	/** a temp */
	private Transform3D tempRotator = new Transform3D();

	public void process(long timeElapsedSinceLastProcess)
	{
		if (active)
		{
			// if it's been more than a second we need to discard this frame of changes as it's unlikely to be a nice result
			if (timeElapsedSinceLastProcess < 1000)
			{
				// get the values
				avatarLocation.get(avatarRot, avatarTranslation);

				double rotY = 0;
				double rotX = 0;

				// is there anything to do?
				if (rotationYPerSec != 0 || rotationXPerSec != 0)
				{
					// work out how much change should happen by how long since we last updated, in seconds
					float motionDelay = timeElapsedSinceLastProcess / 1000f;
					rotY = rotationYPerSec * motionDelay;
					rotX = rotationXPerSec * motionDelay;
				}
				else if (rotationY != Float.MIN_VALUE || rotationX != Float.MIN_VALUE)
				{
					rotY = rotationY;
					rotX = rotationX;

					// now empty the rotation holders, as the required amount of rot has been done
					rotationY = 0;
					rotationX = 0;
				}

				if (rotY != 0 || rotX != 0)
				{
					// *********ROTATION HANDLING ****************
					tempYawPitch.set(avatarRot);
					tempYawPitch.setYaw(tempYawPitch.getYaw() + rotY);
					tempYawPitch.setPitch(noPitch ? 0 : tempYawPitch.getPitch() + rotX);

					tempYawPitch.get(avatarRot);
					avatarLocation.setRotation(avatarRot);//we also set this to make the screen nice and smooth for now
				}

				if (xChangePerSec != 0 || yChangePerSec != 0 || zChangePerSec != 0)
				{
					// *********TRANSLATION HANDLING ****************
					oneFrameTranslation.set(xChangePerSec, yChangePerSec, -zChangePerSec);// -ve z is forward

					// NOTE the below removes the rotX (pitch) so flying doesn't agree with this
					// we need to tranlate the local x,y,z moves on the current axis, but without any pitch, as we are not
					// flying yet.
					tempYawPitch.set(avatarRot);
					tempRotator.rotY(tempYawPitch.getYaw());
					tempRotator.transform(oneFrameTranslation);
				}
				else
				{
					oneFrameTranslation.set(0, 0, 0);
				}

				if (nbccProvider.getNBControlledChar() != null)
				{
					KinematicCharacterController3 kcc = nbccProvider.getNBControlledChar().getCharacterController();
					//kcc.setWalkDirection(currentOneFrameTranslation);						
					kcc.setRotation(avatarRot);
					kcc.setVelocityForTimeInterval(oneFrameTranslation, 1.0f);
				}
			}
		}
	}

	public boolean isActive()
	{
		return active;
	}

	public void setActive(boolean active)
	{
		this.active = active;
	}

	public interface NbccProvider
	{
		public NBControlledChar getNBControlledChar();
	}

}