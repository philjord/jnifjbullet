package nifbullet.cha;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.jogamp.java3d.BranchGroup;
import org.jogamp.java3d.Transform3D;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.character.KinematicCharacterController.CharacterPositionListener;
import com.bulletphysics.linearmath.Transform;

import nifbullet.util.NifBulletUtil;

public class NBNonControlledChar extends BranchGroup implements NifBulletChar
{
	private int recordId = -1;

	private float characterHeight = 1f;//notice half extents so 2m tall

	private float characterWidth = 0.5f;//something odd in the shape here?

	private RigidBody rigidBody;

	private DynamicsWorld dynamicsWorld = null;

	private CharacterPositionListener listener;

	public NBNonControlledChar(Transform3D rootTrans, float mass, int recordId)
	{
		this.recordId = recordId;
		setCapability(BranchGroup.ALLOW_DETACH);

		BoxShape boxShape = new BoxShape(new Vector3f(characterWidth, characterHeight, characterWidth));
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, null, boxShape);

		rigidBody = new RigidBody(rbInfo);
		rigidBody.setCollisionFlags(CollisionFlags.KINEMATIC_OBJECT);
		rigidBody.setUserPointer(this);
		setTransform(rootTrans);
	}

	public int getRecordId()
	{
		return recordId;
	}

	@Override
	public void setCharacterPositionListener(CharacterPositionListener listener)
	{
		this.listener = listener;
	}

	@Override
	public void setTransform(Quat4f q, Vector3f v)
	{
		Transform3D t = new Transform3D(q, v, 1f);
		setTransform(t);
	}

	@Override
	public void setTransform(Transform3D trans)
	{
		Transform worldTransform = NifBulletUtil.createTrans(trans);
		rigidBody.setWorldTransform(worldTransform);
		rigidBody.activate(true);
	}

	public void applyRelCentralForce(Vector3f linearForce)
	{
		//tranlate force to local coords
		Transform wt = new Transform();
		rigidBody.getWorldTransform(wt);
		Quat4f currRotation = new Quat4f();
		wt.getRotation(currRotation);
		Transform3D t = new Transform3D();
		t.set(currRotation);
		t.transform(linearForce);

		rigidBody.applyCentralForce(linearForce);
		rigidBody.activate(true);

	}

	public void applyRelTorque(Vector3f rotationalForce)
	{
		//tranlate force to local coords
		Transform wt = new Transform();
		rigidBody.getWorldTransform(wt);
		Quat4f currRotation = new Quat4f();
		wt.getRotation(currRotation);
		Transform3D t = new Transform3D();
		t.set(currRotation);
		t.transform(rotationalForce);

		rigidBody.applyTorque(rotationalForce);
		rigidBody.activate(true);
	}

	public RigidBody getRigidBody()
	{
		return rigidBody;
	}

	@Override
	public void destroy()
	{
		if (dynamicsWorld != null)
		{
			new Throwable("destroy called whilst in dynamic world");
		}
		rigidBody.destroy();

	}

	/**
	 * Basically a set enabled true
	 */
	@Override
	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld)
	{
		this.dynamicsWorld = dynamicsWorld;
		if (rigidBody != null)
		{
			if (dynamicsWorld != null)
			{
				synchronized (dynamicsWorld)
				{
					dynamicsWorld.addRigidBody(rigidBody);
				}
			}
		}

	}

	/** basically a set enabled false
	 * 
	 */
	@Override
	public void removeFromDynamicsWorld()
	{// check for double remove or no add yet
		if (dynamicsWorld != null)
		{
			synchronized (dynamicsWorld)
			{
				if (rigidBody != null)
				{
					dynamicsWorld.removeRigidBody(rigidBody);
				}
			}
			dynamicsWorld = null;
		}
	}

	@Override
	public String toString()
	{
		return "NBNoncontrollerCharModel in class of " + this.getClass();
	}

}
