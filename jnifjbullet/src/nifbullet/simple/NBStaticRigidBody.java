package nifbullet.simple;

import org.jogamp.java3d.Transform3D;
import org.jogamp.java3d.utils.geometry.GeometryInfo;
import org.jogamp.vecmath.Quat4f;
import org.jogamp.vecmath.Vector3f;

import nif.NiObjectList;
import nif.enums.OblivionLayer;
import nif.j3d.J3dNiAVObject;
import nif.niobject.NiAVObject;
import nif.niobject.RootCollisionNode;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkNPCollisionObject;
import nif.niobject.bhk.bhkPhysicsSystem;
import nif.niobject.bhk.bhkRigidBody;
import nif.niobject.bhk.bhkRigidBodyT;
import nif.niobject.bhk.bhkShape;
import nif.niobject.hkx.hknpBodyCinfo;
import nif.niobject.hkx.hknpMaterial;
import nif.niobject.hkx.hknpPhysicsSystemData;
import nif.niobject.hkx.hknpShape;
import nif.niobject.hkx.reader.HKXContents;
import nifbullet.BulletNifModel;
import nifbullet.NBRigidBody;
import nifbullet.convert.BhkCollisionToNifBullet;
import nifbullet.convert.BhkShapeToCollisionShape;
import nifbullet.convert.RootCollisionNodeToCollisionShape;
import nifbullet.convert.hkxShapeToCollisionShape;
import nifbullet.util.NifBulletUtil;
import utils.convert.ConvertFromHavok;
import utils.convert.ConvertFromNif;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.dom.HeightfieldTerrainShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.Transform;

public class NBStaticRigidBody extends NBRigidBody
{
	private NiAVObject parentNiObject;

	private NiObjectList niObjectList;

	private Transform3D worldTransformCalc = new Transform3D();

	/** 
	 * 
	 * @param bhkCollisionObject
	 * @param blocks
	 * @param rootTrans
	 * @param parentModel
	 * @param fixedScaleFactor
	 */
	public NBStaticRigidBody(bhkCollisionObject bhkCollisionObject, NiObjectList blocks, Transform3D rootTrans, BulletNifModel parentModel)
	{
		super(parentModel);

		this.niObjectList = blocks;

		parentNiObject = (NiAVObject) blocks.get(bhkCollisionObject.target);
		bhkRigidBody bhkRigidBody = (bhkRigidBody) blocks.get(bhkCollisionObject.body);
		setBhkRigidBody(bhkRigidBody);

		int layer = bhkRigidBody.layer.layer;
		if (layer == OblivionLayer.OL_STATIC || layer == OblivionLayer.OL_UNIDENTIFIED || layer == OblivionLayer.OL_STAIRS
				|| layer == OblivionLayer.OL_TERRAIN || layer == OblivionLayer.OL_TRANSPARENT || layer == OblivionLayer.OL_TREES)
		{
			if (bhkRigidBody.mass == 0)
			{
				bhkShape bhkShape = (bhkShape) blocks.get(bhkRigidBody.shape);
				//updateTransfrom MUST be called first, it sets scale
				Transform worldTransform = calcWorldTransform(rootTrans);
				colShape = BhkShapeToCollisionShape.processBhkShape(bhkShape, blocks, scale);
				setRigidBody(NifBulletUtil.createStaticRigidBody(bhkRigidBody, colShape, this));
				getRigidBody().setWorldTransform(worldTransform);
			}
			else
			{
				new Throwable("bhkRigidBody.mass != 0 " + this).printStackTrace();
			}

		}
		else if (layer == OblivionLayer.OL_LINE_OF_SIGHT)
		{
			//skipped for now
		}
		else
		{
			new Throwable(
					"Why is a non OL_STATIC, OL_LINE_OF_SIGHT, OL_UNIDENTIFID, OL_STAIRS, OL_TERRAIN, OL_TRANSPARENT, OL_TREES handed to  me? "
							+ layer + " " + this).printStackTrace();
		}
	}
	
	public NBStaticRigidBody(bhkNPCollisionObject bhkNPCollisionObject, bhkPhysicsSystem bhkPhysicsSystem, NiObjectList blocks, Transform3D rootTrans, BulletNifModel parentModel)
	{
		super(parentModel);
		
		this.niObjectList = blocks;
		parentNiObject = (NiAVObject) blocks.get(bhkNPCollisionObject.target);
		
		HKXContents contents = bhkPhysicsSystem.hkxContents;
		hknpPhysicsSystemData hknpPhysicsSystemData = (hknpPhysicsSystemData)contents.getContentCollection().iterator().next();
		hknpBodyCinfo[] bodyCinfos = hknpPhysicsSystemData.bodyCinfos;
		hknpMaterial[] materials = hknpPhysicsSystemData.materials;
		for(int  i= 0 ; i < bodyCinfos.length; i++) {
			hknpBodyCinfo hknpBodyCinfo = bodyCinfos[i];
			hknpMaterial hknpMaterial = materials[i];

			long shapeId = hknpBodyCinfo.shape;
			if(shapeId > 0) {
				hknpShape hknpShape = (hknpShape)contents.get(shapeId);
								
				
				//a shape is dymanic if for the asme index there is a motionProperty
				 
				//setBhkRigidBody(bhkRigidBody);
		
				//int layer = hknpShape.layer.layer;
				//if (layer == OblivionLayer.OL_STATIC || layer == OblivionLayer.OL_UNIDENTIFIED || layer == OblivionLayer.OL_STAIRS
				//		|| layer == OblivionLayer.OL_TERRAIN || layer == OblivionLayer.OL_TRANSPARENT || layer == OblivionLayer.OL_TREES)
				//{
				//	if (bhkRigidBody.mass == 0)
				//	{
						 
						//updateTransfrom MUST be called first, it sets scale
						Transform worldTransform = calcWorldTransform(rootTrans);
						colShape = hkxShapeToCollisionShape.processBhkShape(hknpShape, contents, scale);
						setRigidBody(NifBulletUtil.createStaticRigidBody(hknpMaterial, colShape, this));
						getRigidBody().setWorldTransform(worldTransform);
				//	}
				//	else
				//	{
				//		new Throwable("bhkRigidBody.mass != 0 " + this).printStackTrace();
				//	}
		
				//}
			}
		}
	}
	/**
	 * Special cut down version for morrowind
	 * @param  
	 * @param rootTrans
	 * @param parentModel
	 */
	public NBStaticRigidBody(RootCollisionNode rootCollisionNode, NiObjectList blocks, Transform3D rootTrans, BulletNifModel parentModel)
	{
		super(parentModel);
		Transform worldTransform = calcWorldTransform(rootTrans);
		colShape = RootCollisionNodeToCollisionShape.processRootCollisionNode(rootCollisionNode, blocks, 1f);
		RigidBody rigidBody = new RigidBody(new RigidBodyConstructionInfo(0, null, colShape));
		rigidBody.setCollisionFlags(CollisionFlags.STATIC_OBJECT);
		setRigidBody(rigidBody);
		rigidBody.setWorldTransform(worldTransform);
	}

	/**
	 * 
	 * Special cut down version for J3dLAND  
	 * @param  
	 * @param rootTrans
	 * @param parentModel
	 */
	public NBStaticRigidBody(GeometryInfo gi, Transform3D rootTrans, BulletNifModel parentModel)
	{
		super(parentModel);
		Transform worldTransform = calcWorldTransform(rootTrans);
		colShape = BhkCollisionToNifBullet.makeFromGeometryInfo(gi);
		RigidBody rigidBody = new RigidBody(new RigidBodyConstructionInfo(0, null, colShape));
		rigidBody.setCollisionFlags(CollisionFlags.STATIC_OBJECT);
		setRigidBody(rigidBody);
		rigidBody.setWorldTransform(worldTransform);
	}

	/**
	 * 
	 * Special cut down version for J3dLAND  
	 * @param  
	 * @param rootTrans
	 * @param parentModel
	 */
	public NBStaticRigidBody(float[][] heights, Transform3D rootTrans, NBSimpleModel parentModel, float terrainSquareSize)
	{
		super(parentModel);
		HeightfieldTerrainShape hfts = BhkCollisionToNifBullet.makeHeightfieldTerrainShape(heights, terrainSquareSize);
		// Interesting note, height field moves everything down by ((max-min)/2)+min height in order
		// to centralize the field in it's AABB, so we must place it back up by that amount
		//http://www.bulletphysics.com/Bullet/BulletFull/classbtHeightfieldTerrainShape.html
		Vector3f trans = new Vector3f();
		rootTrans.get(trans);
		//trans.y should be 0 now
		trans.y = ((hfts.getMaxHeight() - hfts.getMinHeight()) / 2) + hfts.getMinHeight();
		rootTrans.setTranslation(trans);
		Transform worldTransform = calcWorldTransform(rootTrans);

		colShape = hfts;
		RigidBody rigidBody = new RigidBody(new RigidBodyConstructionInfo(0, null, colShape));
		rigidBody.setCollisionFlags(CollisionFlags.STATIC_OBJECT);
		setRigidBody(rigidBody);
		rigidBody.setWorldTransform(worldTransform);
	}

	/**
	 * NOTICE this just uses the unchanging static niObject fixed rotr's
	 * The kinematic equivilent use altering j3dNiAVObjects
	 * @param rootTrans
	 * @return
	 */
	private Transform calcWorldTransform(Transform3D rootTrans)
	{
		// add the root trans in
		worldTransformCalc.set(rootTrans);

		NiAVObject parent = parentNiObject;
		// WAIT J3dNiAVObject goes from parent down wards this business goes 
		// root then parent up to null then bhkRigidbody!
		mulFromRootDown(parent);

		bhkRigidBody rb = getBhkRigidBody();
		//land and morrwind can be null
		if (rb != null && rb instanceof bhkRigidBodyT)
		{
			temp.setRotation(ConvertFromHavok.toJ3d(rb.rotation));
			temp.setTranslation(ConvertFromHavok.toJ3d(rb.translation, 1f, niObjectList.nifVer));
			worldTransformCalc.mul(temp);
		}

		Transform worldTransform = NifBulletUtil.newIdentityTransform();
		worldTransformCalc.get(worldTransform.origin);
		worldTransformCalc.get(worldTransform.basis);

		//NOTICE there is no scaling able to be done via
		// bullet transforms, so scaling got sent to the 
		// BhkShapeToCollisionShape call and is in the model now
		// so we just record it here
		this.scale = (float) worldTransformCalc.getScale();

		return worldTransform;
	}

	// deburner
	private Transform3D temp = new Transform3D();

	private void mulFromRootDown(NiAVObject parent)
	{
		if (parent != null)
		{
			//note go up first then come back down and do multiplys
			mulFromRootDown(parent.parent);

			if (!J3dNiAVObject.ignoreTopTransformRot(parent))
			{
				temp.setRotation(ConvertFromNif.toJ3d(parent.rotation));
			}
			else
			{
				temp.setRotation(new Quat4f(0, 0, 0, 1));
			}
			temp.setTranslation(ConvertFromNif.toJ3d(parent.translation));
			temp.setScale(parent.scale);

			worldTransformCalc.mul(temp);
		}
	}

	@Override
	public void updateRootTransform(Transform3D rootTrans)
	{
		throw new UnsupportedOperationException();

	}

}
