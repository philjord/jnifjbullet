package nifbullet.stat;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import nif.NifFile;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.niobject.NiObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nifbullet.BulletNifModelClassifier;
import nifbullet.PartedBulletNifModel;
import utils.source.MeshSource;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.sun.j3d.utils.geometry.GeometryInfo;

public class NBStaticModel implements PartedBulletNifModel
{
	protected ArrayList<NBStaticRigidBody> nifBulletbhkCollisionObjects = new ArrayList<NBStaticRigidBody>();

	private String fileName = "";

	protected NBStaticRigidBody rootStaticBody;

	private boolean isInDynamicWorld = false;

	public NBStaticModel(String filename, MeshSource meshSource, Transform3D rootTrans)
	{
		this.fileName = filename;
		addPart(filename, meshSource, this, rootTrans);
	}

	public NBStaticModel(GeometryInfo gi, Transform3D rootTrans)
	{
		if (gi != null)
		{
			NBStaticRigidBody nbbco = new NBStaticRigidBody(gi, rootTrans, this);
			nifBulletbhkCollisionObjects.add(nbbco);
		}
	}

	public void destroy()
	{
		if (isInDynamicWorld)
		{
			new Throwable("destroy called whilst in dynamic world");
		}

		for (NBStaticRigidBody nbbco : nifBulletbhkCollisionObjects)
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
		for (NBStaticRigidBody nbbco : nifBulletbhkCollisionObjects)
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
		for (NBStaticRigidBody nbbco : nifBulletbhkCollisionObjects)
		{
			dynamicsWorld.removeRigidBody(nbbco.getRigidBody());
		}
		isInDynamicWorld = false;
	}

	@Override
	public void addPart(String partFileName, MeshSource meshSource, Object pointer, Transform3D rootTrans)
	{
		if (BulletNifModelClassifier.isStaticModel(partFileName, meshSource))
		{
			NifFile nifFile = NifToJ3d.loadNiObjects(partFileName, meshSource);

			for (NiObject niObject : nifFile.blocks.getNiObjects())
			{
				if (niObject instanceof bhkCollisionObject)
				{
					bhkCollisionObject bhkCollisionObject = (bhkCollisionObject) niObject;
					bhkRigidBody bhkRigidBody = (bhkRigidBody) nifFile.blocks.get(bhkCollisionObject.body);
					int layer = bhkRigidBody.layer.layer;
					if (layer == OblivionLayer.OL_STATIC || layer == OblivionLayer.OL_UNIDENTIFIED || layer == OblivionLayer.OL_STAIRS
							|| layer == OblivionLayer.OL_TERRAIN || layer == OblivionLayer.OL_TRANSPARENT)
					{
						float sf = (float) rootTrans.getScale();
						rootTrans.setScale(1.0f);
						NBStaticRigidBody nbbco = new NBStaticRigidBody(bhkCollisionObject, nifFile.blocks, rootTrans, this, sf);
						nifBulletbhkCollisionObjects.add(nbbco);

						if (rootStaticBody != null)
						{
							rootStaticBody = nbbco;
						}
					}
					else if (layer == OblivionLayer.OL_LINE_OF_SIGHT)
					{
						//skipped for now
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
			new Throwable("bad idea bad bad, bad file too " + this + "  " + fileName).printStackTrace();
		}

	}

	@Override
	public void setPartTransform(Object pointer, Transform3D t)
	{
		rootStaticBody.setPartTransform(pointer, t);
	}

	@Override
	public Object getPartPointer(CollisionShape collisionShape)
	{
		return rootStaticBody.getPartPointer(collisionShape);
	}

	@Override
	public void removePart(Object pointer)
	{
		rootStaticBody.removePart(pointer);
	}

	public String toString()
	{
		return "NifBullet, file: " + fileName + " class;" + this.getClass().getSimpleName();
	}

}
