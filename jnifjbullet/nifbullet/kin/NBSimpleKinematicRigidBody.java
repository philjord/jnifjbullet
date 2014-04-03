package nifbullet.kin;

import javax.media.j3d.Group;
import javax.media.j3d.Transform3D;

import nif.enums.OblivionLayer;
import nif.j3d.J3dNiAVObject;
import nif.j3d.NiToJ3dData;
import nif.niobject.NiAVObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nif.niobject.bhk.bhkRigidBodyT;
import nif.niobject.bhk.bhkShape;
import nifbullet.BulletNifModel;
import nifbullet.NBRigidBody;
import nifbullet.convert.BhkShapeToCollisionShape;
import nifbullet.util.NifBulletUtil;
import tools3d.utils.Utils3D;
import utils.convert.ConvertFromHavok;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.linearmath.Transform;

public class NBSimpleKinematicRigidBody extends NBRigidBody
{
	private J3dNiAVObject j3dNiNodeRoot;

	private Transform3D rootTrans;

	private J3dNiAVObject parentJ3dNiAVObject;

	public NBSimpleKinematicRigidBody(Group behaviourRootGroup, J3dNiAVObject j3dNiNodeRoot, bhkCollisionObject bhkCollisionObject,
			NiToJ3dData niToJ3dData, Transform3D rootTrans, BulletNifModel parentModel, float fixedScaleFactor)
	{
		super(parentModel, fixedScaleFactor);

		this.j3dNiNodeRoot = j3dNiNodeRoot;
		this.rootTrans = rootTrans;

		bhkRigidBody bhkRigidBody = (bhkRigidBody) niToJ3dData.get(bhkCollisionObject.body);
		setBhkRigidBody(bhkRigidBody);
		parentJ3dNiAVObject = niToJ3dData.get((NiAVObject) niToJ3dData.get(bhkCollisionObject.target));

		int layer = bhkRigidBody.layer.layer;

		if (layer == OblivionLayer.OL_ANIM_STATIC)
		{
			if (bhkRigidBody.mass == 0)
			{
				bhkShape bhkShape = (bhkShape) niToJ3dData.get(bhkRigidBody.shape);
				CollisionShape colShape = BhkShapeToCollisionShape.processBhkShape(bhkShape, niToJ3dData.getNiObjects(), fixedScaleFactor);

				setRigidBody(NifBulletUtil.createRigidBody(bhkRigidBody, colShape, calcWorldTransform(), this));

				SceneGraphTransformChangeBehavior behave = new SceneGraphTransformChangeBehavior(parentJ3dNiAVObject, this, niToJ3dData);

				behaviourRootGroup.addChild(behave);
				behave.setSchedulingBounds(Utils3D.defaultBounds);
				behave.setEnable(true);
			}
			else
			{
				new Throwable("bhkRigidBody.mass != 0 " + this).printStackTrace();
			}
		}
		else
		{
			new Throwable("Why is a non OL_ANIM_STATIC handed to  me? " + layer + " " + this).printStackTrace();
		}
	}

	public void updateRootTransform(Transform3D newRootTrans)
	{
		rootTrans.set(newRootTrans);
		updateInternalWorldTransform();
	}

	/**
	 * Notice that no new rootTransform is handed in, this method updates the internals 
	 * based on animations, it is not a method to update rootTransform
	 */
	public void updateInternalWorldTransform()
	{
		System.out.println("updatey called? should be called heaps?");
		getRigidBody().getMotionState().setWorldTransform(calcWorldTransform());
		getRigidBody().activate(true);
	}

	//deburners
	private Transform3D worldTransformCalc = new Transform3D();

	private Transform worldTransform = NifBulletUtil.newIdentityTransform();

	/** 
	 * go up to the root of the model adding in all transforms then add o the root transform (models location in world)	
	 * NOTICE this use the altering j3dNiAVObjects
	 * Note the fixed rotrs taht static equivilent uses
	 * @param rootTrans
	 * @return
	 */

	private Transform calcWorldTransform()
	{
		// add the root trans in
		worldTransformCalc.set(rootTrans);
		if (parentJ3dNiAVObject != null)
		{

			// get teh transform up to the root
			Transform3D temp = new Transform3D();
			parentJ3dNiAVObject.getTreeTransform(temp, j3dNiNodeRoot);
			worldTransformCalc.mul(temp);

			// add in the rigid body trans
			if (getBhkRigidBody() instanceof bhkRigidBodyT)
			{
				temp.set(ConvertFromHavok.toJ3d(getBhkRigidBody().rotation),
						ConvertFromHavok.toJ3d(getBhkRigidBody().translation, fixedScaleFactor), 1f);
				worldTransformCalc.mul(temp);
			}
		}

		worldTransformCalc.get(worldTransform.origin);
		worldTransformCalc.get(worldTransform.basis);
		return worldTransform;

	}

}
