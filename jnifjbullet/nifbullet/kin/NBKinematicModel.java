package nifbullet.kin;

import java.util.ArrayList;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;

import nif.NifJ3dHavokRoot;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.j3d.J3dNiAVObject;
import nif.j3d.NiToJ3dData;
import nif.j3d.animation.J3dNiControllerManager;
import nif.niobject.NiObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nifbullet.BulletNifModel;
import nifbullet.BulletNifModelClassifier;
import nifbullet.stat.NBStaticRigidBody;
import utils.source.MeshSource;

import com.bulletphysics.dynamics.DynamicsWorld;

public class NBKinematicModel extends BranchGroup implements BulletNifModel
{
	private ArrayList<NBStaticRigidBody> staticRigidBodys = new ArrayList<NBStaticRigidBody>();

	private ArrayList<NBSimpleKinematicRigidBody> simpleKinematicRigidBodys = new ArrayList<NBSimpleKinematicRigidBody>();

	private NifJ3dHavokRoot nifJ3dRoot;

	private String fileName = "";

	private boolean isInDynamicWorld = false;

	public NBKinematicModel(String fileName, MeshSource meshSource, Transform3D rootTrans)
	{
		this.fileName = fileName;
		setCapability(BranchGroup.ALLOW_DETACH);

		if (BulletNifModelClassifier.isKinematicModel(fileName, meshSource))
		{
			nifJ3dRoot = NifToJ3d.loadHavok(fileName, meshSource);
			J3dNiAVObject j3dNiNodeRoot = nifJ3dRoot.getHavokRoot();
			NiToJ3dData niToJ3dData = nifJ3dRoot.getNiToJ3dData();

			//needed for animations to occur
			addChild(j3dNiNodeRoot);

			for (NiObject niObject : niToJ3dData.getNiObjects())
			{
				if (niObject instanceof bhkCollisionObject)
				{
					bhkCollisionObject bhkCollisionObject = (bhkCollisionObject) niObject;
					bhkRigidBody bhkRigidBody = (bhkRigidBody) niToJ3dData.get(bhkCollisionObject.body);
					int layer = bhkRigidBody.layer.layer;
					if (layer == OblivionLayer.OL_STATIC || layer == OblivionLayer.OL_LINE_OF_SIGHT
							|| layer == OblivionLayer.OL_UNIDENTIFIED || layer == OblivionLayer.OL_STAIRS
							|| layer == OblivionLayer.OL_TERRAIN || layer == OblivionLayer.OL_TRANSPARENT)
					{
						float sf = (float) rootTrans.getScale();
						rootTrans.setScale(1.0f);
						NBStaticRigidBody sb = new NBStaticRigidBody(bhkCollisionObject, niToJ3dData.getNiObjects(), rootTrans, this, sf);
						staticRigidBodys.add(sb);
					}
					else if (layer == OblivionLayer.OL_ANIM_STATIC)
					{
						float sf = (float) rootTrans.getScale();
						rootTrans.setScale(1.0f);
						NBSimpleKinematicRigidBody kb = new NBSimpleKinematicRigidBody(this, j3dNiNodeRoot, bhkCollisionObject,
								niToJ3dData, rootTrans, this, sf);
						simpleKinematicRigidBodys.add(kb);
					}
					else
					{
						// skipped 
						new Throwable("what is this layer being given to me for? " + layer + " " + this).printStackTrace();
					}
				}
			}
		}
		else
		{
			new Throwable("NifBulletClasser.isSimpleKinematic = false  " + fileName).printStackTrace();
		}
	}

	public J3dNiControllerManager getJ3dNiControllerManager()
	{
		return nifJ3dRoot.getHavokRoot().getJ3dNiControllerManager();
	}

	public void destroy()
	{
		if (isInDynamicWorld)
		{
			new Throwable("destroy called whilst in dynamic world");
		}

		for (NBStaticRigidBody sb : staticRigidBodys)
		{
			sb.destroy();
		}
		staticRigidBodys.clear();
		for (NBSimpleKinematicRigidBody kb : simpleKinematicRigidBodys)
		{
			kb.destroy();
		}
		simpleKinematicRigidBodys.clear();
	}

	/**
	 * Basically a set enabled true
	 */
	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld)
	{

		for (NBStaticRigidBody sb : staticRigidBodys)
		{
			dynamicsWorld.addRigidBody(sb.getRigidBody());
		}

		for (NBSimpleKinematicRigidBody kb : simpleKinematicRigidBodys)
		{
			dynamicsWorld.addRigidBody(kb.getRigidBody());
		}
		isInDynamicWorld = true;
	}

	/** basically a set enabled false
	 * 
	 */
	public void removeFromDynamicsWorld(DynamicsWorld dynamicsWorld)
	{
		for (NBStaticRigidBody nbbco : staticRigidBodys)
		{
			dynamicsWorld.removeRigidBody(nbbco.getRigidBody());
		}

		for (NBSimpleKinematicRigidBody nbbco : simpleKinematicRigidBodys)
		{
			dynamicsWorld.removeRigidBody(nbbco.getRigidBody());
		}
		isInDynamicWorld = false;
	}

	public String toString()
	{
		return "NifBullet, file: " + fileName + " in class of " + this.getClass();
	}

}
