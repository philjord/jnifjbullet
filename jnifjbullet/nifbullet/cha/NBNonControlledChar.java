package nifbullet.cha;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nifbullet.util.NifBulletUtil;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.character.KinematicCharacterController3.CharacterPositionListener;
import com.bulletphysics.linearmath.Transform;

public class NBNonControlledChar extends BranchGroup implements NifBulletChar
{
	private float characterHeight = 0.9f;// capsule shape height is height+(2*radius)

	private float characterWidth = 0.5f;

	private RigidBody rigidBody;

	private DynamicsWorld dynamicsWorld = null;

	private CharacterPositionListener listener;

	public NBNonControlledChar(Transform3D rootTrans, float mass)
	{
		setCapability(BranchGroup.ALLOW_DETACH);

		CapsuleShape capsuleShape = new CapsuleShape(characterWidth, characterHeight);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, null, capsuleShape);

		rigidBody = new RigidBody(rbInfo);
		rigidBody.setCollisionFlags(CollisionFlags.KINEMATIC_OBJECT);
		rigidBody.setUserPointer(this);
		setTransform(rootTrans);
	}

	public void setCharacterPositionListener(CharacterPositionListener listener)
	{
		this.listener = listener;
	}

	public void setTransform(Quat4f q, Vector3f v)
	{
		Transform3D t = new Transform3D(q, v, 1f);
		setTransform(t);
	}

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
	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld)
	{
		dynamicsWorld.addRigidBody(rigidBody);
		this.dynamicsWorld = dynamicsWorld;
	}

	/** basically a set enabled false
	 * 
	 */
	public void removeFromDynamicsWorld()
	{
		dynamicsWorld.removeRigidBody(rigidBody);
		dynamicsWorld = null;
	}

	public String toString()
	{
		return "NBNoncontrollerCharModel in class of " + this.getClass();
	}

}
