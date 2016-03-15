package nifbullet.dyn;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nif.NifFile;
import nif.NifToJ3d;
import nif.niobject.NiNode;
import nif.niobject.NiObject;
import nif.niobject.bhk.bhkCollisionObject;
import nifbullet.PartedBulletNifModel;
import utils.source.MeshSource;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DynamicsWorld;

/**
* @param fileName
* @param meshSource
* @param forcedMass
*/

public abstract class NBDynamicModel extends BranchGroup implements PartedBulletNifModel
{

	private String fileName = "";

	protected NBDynamicRigidBody rootDynamicBody;

	private NifBulletTransformListener transformChangeListener;

	private DynamicsWorld dynamicsWorld = null;

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
			synchronized (dynamicsWorld)
			{
				rootDynamicBody.applyRelCentralForce(linearForce);
				rootDynamicBody.applyRelTorque(rotationalForce);
			}
		}
	}

	public NBDynamicRigidBody getRootNifBulletbhkCollisionObject()
	{
		return rootDynamicBody;
	}

	public void destroy()
	{
		if (dynamicsWorld != null)
		{
			new Throwable("destroy called whilst in dynamic world");
		}

		//can't synch on dynamics world as null
		rootDynamicBody.destroy();

	}

	/**
	 * Basically a set enabled true
	 */
	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld1)
	{
		this.dynamicsWorld = dynamicsWorld1;
		if (rootDynamicBody != null && rootDynamicBody.getRigidBody() != null)
		{
			synchronized (dynamicsWorld1)
			{
				dynamicsWorld1.addRigidBody(rootDynamicBody.getRigidBody());
			}
		}
	}

	/** basically a set enabled false
	 * 
	 */
	public void removeFromDynamicsWorld()
	{
		// not dynamicsWorld!= in case we have already been removed or not added yet
		if (rootDynamicBody != null && rootDynamicBody.getRigidBody() != null && dynamicsWorld != null)
		{
			synchronized (dynamicsWorld)
			{
				dynamicsWorld.removeRigidBody(rootDynamicBody.getRigidBody());
			}
		}
		dynamicsWorld = null;
	}

	@Override
	public void addPart(String partFileName, MeshSource meshSource, Object pointer, Transform3D rootTrans)
	{
		// no check is all cases just the collision sahpe is used
		NifFile nifFile = NifToJ3d.loadNiObjects(partFileName, meshSource);

		if (nifFile != null)
		{
			if (nifFile.blocks.root() instanceof NiNode)
			{
				for (NiObject niObject : nifFile.blocks.getNiObjects())
				{
					if (niObject instanceof bhkCollisionObject)
					{
						bhkCollisionObject bhkCollisionObject = (bhkCollisionObject) niObject;
						//throws an error if 2 pointers
						//synchronized (dynamicsWorld) can be null
						{
							rootDynamicBody.addPart(bhkCollisionObject, nifFile.blocks, pointer, rootTrans);
						}
					}
				}
			}
		}

	}

	@Override
	public void setPartTransform(Object pointer, Transform3D t)
	{
		synchronized (dynamicsWorld)
		{
			rootDynamicBody.setPartTransform(pointer, t);
		}
	}

	@Override
	public Object getPartPointer(CollisionShape collisionShape)
	{
		return rootDynamicBody.getPartPointer(collisionShape);
	}

	@Override
	public void removePart(Object pointer)
	{
		synchronized (dynamicsWorld)
		{
			rootDynamicBody.removePart(pointer);
		}
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
