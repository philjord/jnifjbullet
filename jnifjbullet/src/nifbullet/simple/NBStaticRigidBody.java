package nifbullet.simple;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;

import nif.NiObjectList;
import nif.enums.OblivionLayer;
import nif.niobject.NiAVObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nif.niobject.bhk.bhkRigidBodyT;
import nif.niobject.bhk.bhkShape;
import nif.niobject.bs.BSFadeNode;
import nifbullet.BulletNifModel;
import nifbullet.NBRigidBody;
import nifbullet.convert.BhkCollisionToNifBullet;
import nifbullet.convert.BhkShapeToCollisionShape;
import nifbullet.util.NifBulletUtil;
import utils.convert.ConvertFromHavok;
import utils.convert.ConvertFromNif;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.Transform;
import com.sun.j3d.utils.geometry.GeometryInfo;

public class NBStaticRigidBody extends NBRigidBody
{
	private NiAVObject parentNiObject;

	/** send fixed scaling via the float not transform
	 * 
	 * @param bhkCollisionObject
	 * @param blocks
	 * @param rootTrans
	 * @param parentModel
	 * @param fixedScaleFactor
	 */
	public NBStaticRigidBody(bhkCollisionObject bhkCollisionObject, NiObjectList blocks, Transform3D rootTrans, BulletNifModel parentModel,
			float fixedScaleFactor)
	{
		super(parentModel, fixedScaleFactor);
		parentNiObject = (NiAVObject) blocks.get(bhkCollisionObject.target);
		bhkRigidBody bhkRigidBody = (bhkRigidBody) blocks.get(bhkCollisionObject.body);
		setBhkRigidBody(bhkRigidBody);

		int layer = bhkRigidBody.layer.layer;
		if (layer == OblivionLayer.OL_STATIC || layer == OblivionLayer.OL_UNIDENTIFIED || layer == OblivionLayer.OL_STAIRS
				|| layer == OblivionLayer.OL_TERRAIN || layer == OblivionLayer.OL_TRANSPARENT)
		{
			if (bhkRigidBody.mass == 0)
			{
				bhkShape bhkShape = (bhkShape) blocks.get(bhkRigidBody.shape);
				colShape = BhkShapeToCollisionShape.processBhkShape(bhkShape, blocks, fixedScaleFactor);
				setRigidBody(NifBulletUtil.createStaticRigidBody(bhkRigidBody, colShape, this));
				updateRootTransform(rootTrans);
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
			new Throwable("Why is a non OL_STATIC, OL_LINE_OF_SIGHT, OL_UNIDENTIFID, OL_STAIRS, OL_TERRAIN, OL_TRANSPARENT handed to  me? "
					+ layer + " " + this).printStackTrace();
		}
	}

	/**
	 * Special cut down verison for J3dLAND
	 * @param physicsTriStripArray
	 * @param rootTrans
	 * @param parentModel
	 */
	public NBStaticRigidBody(GeometryInfo gi, Transform3D rootTrans, BulletNifModel parentModel)
	{
		super(parentModel, 1.0f);
		colShape = BhkCollisionToNifBullet.makeFromGeometryInfo(gi);
		RigidBody rigidBody = new RigidBody(new RigidBodyConstructionInfo(0, null, colShape));
		rigidBody.setCollisionFlags(CollisionFlags.STATIC_OBJECT);
		setRigidBody(rigidBody);
		updateRootTransform(rootTrans);
	}

	public void updateRootTransform(Transform3D rootTrans)
	{
		getRigidBody().setWorldTransform(calcWorldTransform(rootTrans));
	}

	//deburners
	private Transform3D worldTransformCalc = new Transform3D();

	private Transform worldTransform = NifBulletUtil.newIdentityTransform();

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

		Transform3D temp = new Transform3D();
		NiAVObject parent = parentNiObject;
		while (parent != null)
		{
			if (!(parent instanceof BSFadeNode))//BSFadeNodes are skipped
			{
				temp.setRotation(ConvertFromNif.toJ3d(parent.rotation));
			}
			else
			{
				temp.setRotation(new Quat4f(0, 0, 0, 1));
			}
			temp.setTranslation(ConvertFromNif.toJ3d(parent.translation));
			worldTransformCalc.mul(temp);

			parent = parent.parent;
		}

		if (getBhkRigidBody() instanceof bhkRigidBodyT)
		{
			temp.setRotation(ConvertFromHavok.toJ3d(getBhkRigidBody().rotation));
			temp.setTranslation(ConvertFromHavok.toJ3d(getBhkRigidBody().translation, fixedScaleFactor));
			worldTransformCalc.mul(temp);
		}

		worldTransformCalc.get(worldTransform.origin);
		worldTransformCalc.get(worldTransform.basis);
		return worldTransform;
	}

}
