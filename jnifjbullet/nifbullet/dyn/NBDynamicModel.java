package nifbullet.dyn;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nifbullet.BulletNifModel;

import com.bulletphysics.dynamics.DynamicsWorld;

/**
* @param fileName
* @param meshSource
* @param forcedMass
*/

public abstract class NBDynamicModel extends BranchGroup implements BulletNifModel
{

	private String fileName = "";

	protected NBDynamicRigidBody rootDynamicBody;

	private NifBulletTransformListener transformChangeListener;

	private boolean isInDynamicWorld = false;

	public NBDynamicModel(String fileName)
	{
		this.fileName = fileName;
		setCapability(BranchGroup.ALLOW_DETACH);
	}

	/**
	 * Registers a listener that will get transform update from NBSimpleDynamicRigidBody
	 * @param transformChangeListener
	 */
	public void setTransformChangeListener(NifBulletTransformListener transformChangeListener)
	{
		this.transformChangeListener = transformChangeListener;
	}

	/*
	 * Called externally to tell the nifbullet it has moved 
	 */
	public void forceUpdate(Transform3D trans, Vector3f linearVelocity, Vector3f rotationalVelocity)
	{
		if (rootDynamicBody != null)
		{
			rootDynamicBody.forceUpdate(trans, linearVelocity, rotationalVelocity);
		}
	}

	public void forceUpdate(Vector3f linearVelocity, Vector3f rotationalVelocity)
	{
		if (rootDynamicBody != null)
		{
			rootDynamicBody.forceUpdate(linearVelocity, rotationalVelocity);
		}
	}

	/*
	 * Called externally to tell the nifbullet it has moved 
	 */
	public void setTransform(Quat4f q, Vector3f v)
	{
		forceUpdate(q, v);
	}

	private void forceUpdate(Quat4f q, Vector3f v)
	{
		if (rootDynamicBody != null)
		{
			Transform3D trans = new Transform3D(q, v, 1f);
			rootDynamicBody.forceUpdate(trans);
		}
	}

	public void applyRelForces(Vector3f linearForce, Vector3f rotationalForce)
	{
		if (this.rootDynamicBody != null)
		{
			rootDynamicBody.applyRelCentralForce(linearForce);
			rootDynamicBody.applyRelTorque(rotationalForce);
		}
	}

	public NBDynamicRigidBody getRootNifBulletbhkCollisionObject()
	{
		return rootDynamicBody;
	}

	public void destroy()
	{
		if (isInDynamicWorld)
		{
			new Throwable("destroy called whilst in dynamic world");
		}

		rootDynamicBody.destroy();

	}

	/**
	 * Basically a set enabled true
	 */
	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld)
	{
		dynamicsWorld.addRigidBody(rootDynamicBody.getRigidBody());
		isInDynamicWorld = true;
	}

	/** basically a set enabled false
	 * 
	 */
	public void removeFromDynamicsWorld(DynamicsWorld dynamicsWorld)
	{
		dynamicsWorld.removeRigidBody(rootDynamicBody.getRigidBody());
		isInDynamicWorld = false;
	}

	public String toString()
	{
		return "NifBullet, file: " + fileName + " class;" + this.getClass().getSimpleName();
	}

	protected class NifBulletTransformListenerDelegate implements NifBulletTransformListener
	{
		@Override
		/**
		 * Sending changes from teh NBSimpleDynamicRigidBody out to the registered listener
		 * @see nifbullet.dyn.NifBulletTransformListener#transformChanged(javax.media.j3d.Transform3D, javax.vecmath.Vector3f, javax.vecmath.Vector3f)
		 */
		public void transformChanged(Transform3D trans, Vector3f linearVelocity, Vector3f rotationalVelocity)
		{
			// if we are being listened to update listener
			if (transformChangeListener != null)
			{
				transformChangeListener.transformChanged(trans, linearVelocity, rotationalVelocity);
			}
		}
	}
}
