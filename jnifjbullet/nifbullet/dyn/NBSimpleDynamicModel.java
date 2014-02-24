package nifbullet.dyn;

import java.util.ArrayList;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nif.NifFile;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.niobject.NiNode;
import nif.niobject.NiObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nifbullet.BulletNifModel;
import nifbullet.BulletNifModelClassifier;
import utils.source.MeshSource;

import com.bulletphysics.dynamics.DynamicsWorld;

public class NBSimpleDynamicModel extends BranchGroup implements BulletNifModel
{
	private ArrayList<NBSimpleDynamicRigidBody> nifBulletbhkCollisionObjects = new ArrayList<NBSimpleDynamicRigidBody>();

	private String fileName = "";

	private NBSimpleDynamicRigidBody rootDynamicBody;

	private NifBulletTransformListener transformChangeListener;

	private boolean isInDynamicWorld = false;

	public NBSimpleDynamicModel(String fileName, MeshSource meshSource)
	{

		this.fileName = fileName;
		setCapability(BranchGroup.ALLOW_DETACH);

		if (BulletNifModelClassifier.isSimpleDynamicModel(fileName, meshSource))
		{
			NifFile nifFile = NifToJ3d.loadNiObjects(fileName, meshSource);

			if (nifFile != null)
			{
				if (nifFile.blocks.root() instanceof NiNode)
				{
					for (NiObject niObject : nifFile.blocks.getNiObjects())
					{
						if (niObject instanceof bhkCollisionObject)
						{
							//TODO: check for collision being off the root node, otherwise we should be a complex dynamic

							bhkCollisionObject bhkCollisionObject = (bhkCollisionObject) niObject;
							bhkRigidBody bhkRigidBody = (bhkRigidBody) nifFile.blocks.get(bhkCollisionObject.body);
							int layer = bhkRigidBody.layerCopy.layer;
							if (layer == OblivionLayer.OL_CLUTTER || layer == OblivionLayer.OL_PROPS)
							{
								if (bhkRigidBody.mass != 0)
								{
									rootDynamicBody = new NBSimpleDynamicRigidBody(new NifBulletTransformListenerDelegate(),
											bhkCollisionObject, nifFile.blocks, this, 1.0f);
									nifBulletbhkCollisionObjects.add(rootDynamicBody);
								}
								else
								{
									new Throwable("bhkRigidBody.mass == 0 " + this).printStackTrace();
								}
							}
							else
							{
								new Throwable("what is this layer being given to me for? " + layer + " " + this).printStackTrace();
							}
						}
					}
				}
			}
		}
		else
		{
			new Exception("NifBulletClasser.isSimpleDynamic = false " + fileName).printStackTrace();
		}

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

	public NBSimpleDynamicRigidBody getRootNifBulletbhkCollisionObject()
	{
		return rootDynamicBody;
	}

	public void destroy()
	{
		if (isInDynamicWorld)
		{
			new Throwable("destroy called whilst in dynamic world");
		}

		for (NBSimpleDynamicRigidBody nbbco : nifBulletbhkCollisionObjects)
		{
			nbbco.destroy();
		}
		nifBulletbhkCollisionObjects.clear();
	}

	/**
	 * Basically a set enabled true
	 */
	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld)
	{
		for (NBSimpleDynamicRigidBody nbbco : nifBulletbhkCollisionObjects)
		{
			dynamicsWorld.addRigidBody(nbbco.getRigidBody());
		}
		isInDynamicWorld = true;
	}

	/** basically a set enabled false
	 * 
	 */
	public void removeFromDynamicsWorld(DynamicsWorld dynamicsWorld)
	{
		for (NBSimpleDynamicRigidBody nbbco : nifBulletbhkCollisionObjects)
		{
			dynamicsWorld.removeRigidBody(nbbco.getRigidBody());
		}
		isInDynamicWorld = false;
	}

	public String toString()
	{
		return "NifBullet, file: " + fileName + " class;" + this.getClass().getSimpleName();
	}

	private class NifBulletTransformListenerDelegate implements NifBulletTransformListener
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
