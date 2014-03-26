package nifbullet.dyn;

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

/**
 * //my ship needs to have a root rigid body that is compound so I can add all apras to it!
*probably a custom thing, note that the apar models probabyl should be rigid? 
*No in fact they should have transformable and static parts? Because a gun added on has a turret base that doesn't swivel
*dynamic rigid bodies need to be simple singles or
*compound, where coumpounfd have child col shapes that can kinematically animate
* much like a kinematic RB, but each child is just a col shape and can have more col shapes
* then I need ot intersect with them to find which is pointed at
* So I really need to be able to "add" file name to this model (or a compound version of it)
* Then when intersections happen find out which one it was (maybe a user object pointer)
* Then be able to change the transform for each sub collision object

* arrayList of static bodies, therefore they never move and other things a jointed from them
* arrayList of dynamic children - rigid bodies with constraints
* and array list of transformable children bodies that are in a non jointed constraint with parent
* 
* A The static plus dynamic is training dummy style
* B dynamic plus jointed dynamic is the clothes fall to floor model (one is master and hence holds "location")
* C one dynamic animated collision shapes 
* 
* Can A have B? yes A is B with a static attached
* Can A have C? yes dual col shapes can exist for static and dynamics
* 
* 
 * @param fileName
 * @param meshSource
 * @param forcedMass
 */

public class NBComplexDynamicModel extends BranchGroup implements BulletNifModel
{

	private String fileName = "";

	private NBSimpleDynamicRigidBody rootDynamicBody;

	private NifBulletTransformListener transformChangeListener;

	private boolean isInDynamicWorld = false;

	public NBComplexDynamicModel(String fileName, MeshSource meshSource)
	{
		this(fileName, meshSource, 0);

	}

	public NBComplexDynamicModel(String fileName, MeshSource meshSource, float forcedMass)
	{

		this.fileName = fileName;
		setCapability(BranchGroup.ALLOW_DETACH);

		if (forcedMass != 0 || BulletNifModelClassifier.isSimpleDynamicModel(fileName, meshSource))
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
							if (forcedMass != 0 || layer == OblivionLayer.OL_CLUTTER || layer == OblivionLayer.OL_PROPS)
							{
								bhkRigidBody.mass = forcedMass != 0 ? forcedMass : bhkRigidBody.mass;
								if (bhkRigidBody.mass != 0)
								{
									//TODO: why is this changing constantly only to end up as the last one?
									rootDynamicBody = new NBSimpleDynamicRigidBody(new NifBulletTransformListenerDelegate(),
											bhkCollisionObject, nifFile.blocks, this, 1.0f, forcedMass);

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
