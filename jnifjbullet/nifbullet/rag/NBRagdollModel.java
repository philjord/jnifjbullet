package nifbullet.rag;

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
import nifbullet.dyn.NBSimpleDynamicRigidBody;
import nifbullet.dyn.NifBulletTransformListener;
import utils.source.MeshSource;

import com.bulletphysics.dynamics.DynamicsWorld;

/**
 * All complex dynamic objects will be called ragdoll.
Ragdolls basically simply have constraints (or not for skeletons that fall apart) 
Ragdolls can be connected to bones (and hence visuals will look skinish and lovely) 
or ninodes (and visuals will be storm actronach or chain doll or chandelier) physics doesn’t care.
 
Ragdolls can be anchored or un anchored, chandelier or chain doll anchored, character with foot 
caught in bear trap anchored. Basically physics has a constraint attached to a static (or even kinematic) 
results are just whatever is determined by physics. 

Unanchored (and in fact anchored because it’s no cost) have a ninode (or nibone) that is considered the “root”;
updates to this root’s position need to be sent back into model exactly like a simple dynamic  

Note a complex dynamic that is unanchored and has one ninode and no constraints is the current simple dynamic.
So at some point these will merge as 1 model called dynamic. The anchoring to beartraps and kinematics would be
 across 2 instrecos and so would be “above and outside” the nifbulletmodel system.
 
 Each rigid body in a ragdoll needs to be joined via it's NBDynRigid
 Body to it's particular J3dNinode. That J3dNinode is either rigid (ninode based) or bone (ninode with the bone marker (or nibone?)
  Picking however wants the rigidbody to point to the instreco, so I think the user object pointer should point to the NRigidBody 
  and that should hold a model ref, and that should hold an instref
 
 Note for the unconstrained ragdolls (storm atronach) I still need all transforms to come from the root node so the rendering 
 bounds system works, I can't have every ninode trmsformed in teh world coords and a root node at 0,0,0

 * @author philip
 *
 */
public class NBRagdollModel extends BranchGroup implements BulletNifModel, NifBulletTransformListener
{
	private ArrayList<NBSimpleDynamicRigidBody> nifBulletbhkCollisionObjects = new ArrayList<NBSimpleDynamicRigidBody>();

	private String fileName = "";

	private NBSimpleDynamicRigidBody rootDynamicBody;

	private NifBulletTransformListener transformChangeListener;

	private boolean isInDynamicWorld = false;

	public NBRagdollModel(String fileName, MeshSource meshSource)
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
									//							rootDynamicBody = new NBSimpleDynamicRigidBody(this, bhkCollisionObject, nifFile.blocks);
									//							nifBulletbhkCollisionObjects.add(rootDynamicBody);
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
			dynamicsWorld.addRigidBody(nbbco.getRigidBody());
		}
		isInDynamicWorld = false;
	}

	public String toString()
	{
		return "NifBullet, file: " + fileName + " in class of " + this.getClass();
	}
}
