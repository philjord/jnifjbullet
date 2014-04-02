package nifbullet.dyn;

import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nif.NiObjectList;
import nif.enums.OblivionLayer;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nif.niobject.bhk.bhkRigidBodyT;
import nif.niobject.bhk.bhkShape;
import nifbullet.BulletNifModel;
import nifbullet.NBRigidBody;
import nifbullet.convert.BhkShapeToCollisionShape;
import nifbullet.util.NifBulletUtil;
import utils.convert.ConvertFromHavok;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.linearmath.Transform;

public class NBDynamicRigidBody extends NBRigidBody
{
	// transform binding point, send rigid body trans updates out to the parent transform group and nifbullet
	private NBDynRBTransformListener rigidBodyTransformListener;

	// root shape to allow multi parts to be added as required
	private CompoundShape colShape = new CompoundShape();

	private HashMap<Object, CollisionShape> partsToPointer = new HashMap<Object, CollisionShape>();

	/**
	 * As one of these simple objects controls the transform of one simple model we need the model now to update it
	 * @param nifBullet
	 * @param bhkCollisionObject
	 * @param niToJ3dData
	 */
	public NBDynamicRigidBody(NifBulletTransformListener nbtl, bhkCollisionObject bhkCollisionObject, NiObjectList niToJ3dData,
			BulletNifModel parentModel, float fixedScaleFactor, float forcedMass)
	{
		super(parentModel, fixedScaleFactor);

		bhkRigidBody bhkRigidBody = (bhkRigidBody) niToJ3dData.get(bhkCollisionObject.body);
		setBhkRigidBody(bhkRigidBody);

		int layer = bhkRigidBody.layer.layer;

		if (forcedMass != 0 || layer == OblivionLayer.OL_CLUTTER || layer == OblivionLayer.OL_PROPS)
		{
			bhkRigidBody.mass = forcedMass != 0 ? forcedMass : bhkRigidBody.mass;
			if (bhkRigidBody.mass != 0)
			{
				Transform3D colTrans = new Transform3D();

				if (bhkRigidBody instanceof bhkRigidBodyT)
				{
					colTrans = new Transform3D(ConvertFromHavok.toJ3d(bhkRigidBody.rotation), ConvertFromHavok.toJ3d(
							bhkRigidBody.translation, fixedScaleFactor), 1.0f);
				}
				else
				{
					colTrans.setIdentity();
				}

				bhkShape bhkShape = (bhkShape) niToJ3dData.get(bhkRigidBody.shape);

				colShape.addChildShape(NifBulletUtil.createTrans(colTrans),
						BhkShapeToCollisionShape.processBhkShape(bhkShape, niToJ3dData, true, fixedScaleFactor));
				setRigidBody(NifBulletUtil.createRigidBody(bhkRigidBody, colShape, NifBulletUtil.newIdentityTransform(), this));

				//note we don't set initial transform or velocities during construction
				rigidBodyTransformListener = new NBDynRBTransformListener(nbtl, getRigidBody(), NifBulletUtil.newIdentityTransform());
				getRigidBody().setMotionState(rigidBodyTransformListener);
			}
			else
			{
				new Throwable("bhkRigidBody.mass == 0 " + this).printStackTrace();
			}
		}
		else
		{
			new Throwable("Why is a non OL_CLUTTER or OL_PROPS handed to  me? " + layer + " " + this).printStackTrace();
		}
	}

	public void forceUpdate(Transform3D trans)
	{
		Transform worldTransform = NifBulletUtil.createTrans(trans);
		rigidBodyTransformListener.setWorldTransform(worldTransform);
		// note I MUST also call this, even though I set it in the motion state above, odd.
		getRigidBody().setWorldTransform(worldTransform);
		getRigidBody().activate(true);
	}

	public void forceUpdate(Transform3D trans, Vector3f linearVelocity, Vector3f rotationalVelocity)
	{
		forceUpdate(trans);
		forceUpdate(linearVelocity, rotationalVelocity);
	}

	public void forceUpdate(Vector3f linearVelocity, Vector3f rotationalVelocity)
	{
		getRigidBody().setLinearVelocity(linearVelocity);
		getRigidBody().setAngularVelocity(rotationalVelocity);
		getRigidBody().activate(true);
	}

	public void applyRelCentralForce(Vector3f linearForce)
	{
		//tranlate force to local coords
		Transform wt = new Transform();
		getRigidBody().getWorldTransform(wt);
		Quat4f currRotation = new Quat4f();
		wt.getRotation(currRotation);
		Transform3D t = new Transform3D();
		t.set(currRotation);
		t.transform(linearForce);

		getRigidBody().applyCentralForce(linearForce);
		getRigidBody().activate(true);

	}

	public void applyRelTorque(Vector3f rotationalForce)
	{
		//tranlate force to local coords
		Transform wt = new Transform();
		getRigidBody().getWorldTransform(wt);
		Quat4f currRotation = new Quat4f();
		wt.getRotation(currRotation);
		Transform3D t = new Transform3D();
		t.set(currRotation);
		t.transform(rotationalForce);

		getRigidBody().applyTorque(rotationalForce);
		getRigidBody().activate(true);
	}

	/**
	 * The pointer can be reused? so one pointer many parts? NO!
	 * @param bhkCollisionObject
	 * @param blocks
	 * @param pointer
	 */
	public void addPart(bhkCollisionObject bhkCollisionObject, NiObjectList blocks, Object pointer)
	{
		if (partsToPointer.get(pointer) != null)
		{
			throw new RuntimeException("multiple pointer part mapping!");
		}
		
		bhkRigidBody bhkRigidBody = (bhkRigidBody) blocks.get(bhkCollisionObject.body);

		Transform3D colTrans = new Transform3D();
		if (bhkRigidBody instanceof bhkRigidBodyT)
		{
			colTrans = new Transform3D(ConvertFromHavok.toJ3d(bhkRigidBody.rotation), ConvertFromHavok.toJ3d(bhkRigidBody.translation,
					fixedScaleFactor), 1.0f);
		}
		else
		{
			colTrans.setIdentity();
		}

		bhkShape bhkShape = (bhkShape) blocks.get(bhkRigidBody.shape);
		CollisionShape partColShape = BhkShapeToCollisionShape.processBhkShape(bhkShape, blocks, true, fixedScaleFactor);

		colShape.addChildShape(NifBulletUtil.createTrans(colTrans), partColShape);

		partsToPointer.put(pointer, partColShape);
	}

	public void setPartTransform(Object pointer, Transform3D t)
	{
		CollisionShape cs = partsToPointer.get(pointer);
		colShape.updateChildTransform(colShape.getChildShapeIndex(cs), NifBulletUtil.createTrans(t));
	}
	//TODO: remove part

	//TODO: get part somehow for mouse over and intersept?

}
