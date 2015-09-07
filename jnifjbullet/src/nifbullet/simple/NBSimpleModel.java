package nifbullet.simple;

import java.util.ArrayList;
import java.util.HashMap;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;

import nif.NifJ3dHavokRoot;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.j3d.J3dNiAVObject;
import nif.j3d.NiToJ3dData;
import nif.j3d.animation.J3dNiControllerManager;
import nif.niobject.NiObject;
import nif.niobject.RootCollisionNode;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nifbullet.BulletNifModelClassifier;
import nifbullet.NBRigidBody;
import nifbullet.PartedBulletNifModel;
import utils.source.MeshSource;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.sun.j3d.utils.geometry.GeometryInfo;

//I need to continue merging but check kinematics work and ship parts too
// I notice stuff by spring field elementary are dynamic but not updating the visuals?

public class NBSimpleModel extends BranchGroup implements PartedBulletNifModel
{
	private ArrayList<NBRigidBody> nbRigidBodys = new ArrayList<NBRigidBody>();

	private HashMap<Object, ArrayList<NBRigidBody>> pointerToParts = new HashMap<Object, ArrayList<NBRigidBody>>();

	private HashMap<CollisionShape, Object> partsToPointer = new HashMap<CollisionShape, Object>();

	private String fileName = "";

	private DynamicsWorld dynamicsWorld = null;

	private NifJ3dHavokRoot nifJ3dRoot;//TODO: what if multiples? currently just last set

	public NBSimpleModel(String fileName, MeshSource meshSource, Transform3D rootTrans)
	{
		this.fileName = fileName;
		this.setName(this.getClass().getSimpleName() + ":" + fileName);
		setCapability(BranchGroup.ALLOW_DETACH);
		addPart(fileName, meshSource, this, rootTrans);
	}

	/**
	 * For land use only! needs to be cleaned up
	 * no parts to be added	 
	 * @param gi
	 * @param rootTrans
	 */
	public NBSimpleModel(GeometryInfo gi, Transform3D rootTrans)
	{
		this.fileName = "LAND";
		this.setName(this.getClass().getSimpleName() + ":" + fileName);
		setCapability(BranchGroup.ALLOW_DETACH);
		if (gi != null)
		{
			NBStaticRigidBody nbbco = new NBStaticRigidBody(gi, rootTrans, this);
			add(nbbco);
		}
	}

	public void destroy()
	{
		if (dynamicsWorld != null)
		{
			new Throwable("destroy called whilst in dynamic world");
		}

		for (NBRigidBody rb : nbRigidBodys)
		{
			rb.destroy();
		}
		nbRigidBodys.clear();

	}

	protected void add(NBRigidBody rb)
	{
		nbRigidBodys.add(rb);
	}

	public void addToDynamicsWorld(DynamicsWorld _dynamicsWorld)
	{
		this.dynamicsWorld = _dynamicsWorld;
		for (NBRigidBody rb : nbRigidBodys)
		{
			if (rb.getRigidBody() != null)
				dynamicsWorld.addRigidBody(rb.getRigidBody());
		}
	}

	/** basically a set enabled false
	 * 
	 */
	public void removeFromDynamicsWorld()
	{
		for (NBRigidBody rb : nbRigidBodys)
		{
			if (rb.getRigidBody() != null)
				dynamicsWorld.removeRigidBody(rb.getRigidBody());
		}

		this.dynamicsWorld = null;
	}

	@Override
	public void addPart(String partFileName, MeshSource meshSource, Object pointer, Transform3D rootTrans)
	{
		if (partFileName != null && partFileName.length() > 0)
		{
			//NOTE! we don't actually add parts per se, they are all static so we just create tehm
			if (BulletNifModelClassifier.isStaticModel(partFileName, meshSource)
					|| BulletNifModelClassifier.isKinematicModel(partFileName, meshSource))
			{
				nifJ3dRoot = NifToJ3d.loadHavok(partFileName, meshSource);
				J3dNiAVObject j3dNiNodeRoot = nifJ3dRoot.getHavokRoot();
				NiToJ3dData niToJ3dData = nifJ3dRoot.getNiToJ3dData();

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
							NBStaticRigidBody nbbco = new NBStaticRigidBody(bhkCollisionObject, niToJ3dData.getNiObjects(), rootTrans,
									this, sf);

							updatePointers(pointer, nbbco);
						}
						else if (layer == OblivionLayer.OL_ANIM_STATIC)
						{
							float sf = (float) rootTrans.getScale();
							rootTrans.setScale(1.0f);
							NBKinematicRigidBody kb = new NBKinematicRigidBody(this, j3dNiNodeRoot, bhkCollisionObject, niToJ3dData,
									rootTrans, this, sf);

							updatePointers(pointer, kb);
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
					else if (niObject instanceof RootCollisionNode)
					{						 
						RootCollisionNode rootCollisionNode = (RootCollisionNode) niObject;
						float sf = (float) rootTrans.getScale();
						rootTrans.setScale(1.0f);
						NBStaticRigidBody nbbco = new NBStaticRigidBody(rootCollisionNode, niToJ3dData.getNiObjects(), rootTrans,
								this, sf);

						updatePointers(pointer, nbbco);						
					}
				}

				// but must be placed so camera dist works too
				j3dNiNodeRoot.getTransformGroup().setTransform(rootTrans);

				// needed for animations to occur			
				addChild(j3dNiNodeRoot);

			}
			else
			{
				new Throwable("addPart not simple or kinematic " + this + "  " + partFileName).printStackTrace();
			}
		}

	}

	private void updatePointers(Object pointer, NBRigidBody rb)
	{
		nbRigidBodys.add(rb);
		if (pointerToParts.get(pointer) == null)
		{
			pointerToParts.put(pointer, new ArrayList<NBRigidBody>());
		}

		pointerToParts.get(pointer).add(rb);
		partsToPointer.put(rb.getColShape(), pointer);

	}

	@Override
	public void setPartTransform(Object pointer, Transform3D t)
	{
		ArrayList<NBRigidBody> rbs = pointerToParts.get(pointer);
		for (NBRigidBody rb : rbs)
		{
			rb.updateRootTransform(t);
		}
	}

	@Override
	public Object getPartPointer(CollisionShape collisionShape)
	{
		return partsToPointer.get(collisionShape);
	}

	@Override
	public void removePart(Object pointer)
	{
		ArrayList<NBRigidBody> rbs = pointerToParts.remove(pointer);
		for (NBRigidBody rb : rbs)
		{
			partsToPointer.remove(rb.getColShape());

			dynamicsWorld.removeRigidBody(rb.getRigidBody());

			nbRigidBodys.remove(rb);
		}

	}

	public J3dNiControllerManager getJ3dNiControllerManager()
	{
		return nifJ3dRoot.getHavokRoot().getJ3dNiControllerManager();
	}

	public String getFileName()
	{
		return fileName;
	}

	public String toString()
	{
		return "NifBullet, file: " + getFileName() + " class;" + this.getClass().getSimpleName();
	}

}
