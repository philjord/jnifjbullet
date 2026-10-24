package nifbullet.simple;

import org.jogamp.java3d.Group;
import org.jogamp.java3d.Transform3D;
import org.jogamp.java3d.TransformGroup;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

import nif.NifVer;
import nif.enums.OblivionLayer;
import nif.j3d.J3dNiAVObject;
import nif.j3d.NiToJ3dData;
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
import nifbullet.convert.BhkShapeToCollisionShape;
import nifbullet.convert.RootCollisionNodeToCollisionShape;
import nifbullet.convert.hkxShapeToCollisionShape;
import nifbullet.util.NifBulletUtil;
import tools3d.utils.Utils3D;
import utils.convert.ConvertFromHavok;

/**
 * Kinematic Rigid Bodies are fixed shapes that can be moved about in the world, but are not
 * driven by physics dynamic, so a door is kinematic, an apple is dynamic
 * Of course once you get going a door with a tapestry has both features
 *  
 * StaticRigidBody is rigid (table)
 * KinematicRigidBody is a rigid that moves itself (Door, Player) 
 * DynamicRigidBody is a rigid that rolls about (apple)
 */

public class NBKinematicRigidBody extends NBRigidBody {
	private J3dNiAVObject	j3dNiNodeRoot;

	private NifVer			nifVer;

	private Transform3D		rootTrans;

	private J3dNiAVObject	parentJ3dNiAVObject;

	private TransformGroup	simpleTG;

	public NBKinematicRigidBody(Group behaviourRootGroup, J3dNiAVObject j3dNiNodeRoot,
								bhkCollisionObject bhkCollisionObject, NiToJ3dData niToJ3dData, Transform3D rootTrans,
								BulletNifModel parentModel, float fixedScaleFactor) {
		super(parentModel);

		this.scale = fixedScaleFactor;
		this.j3dNiNodeRoot = j3dNiNodeRoot;
		this.nifVer = niToJ3dData.nifVer;
		this.rootTrans = rootTrans;

		bhkRigidBody bhkRigidBody = (bhkRigidBody)niToJ3dData.get(bhkCollisionObject.body);
		setBhkRigidBody(bhkRigidBody);
		parentJ3dNiAVObject = niToJ3dData.get((NiAVObject)niToJ3dData.get(bhkCollisionObject.target));

		int layer = bhkRigidBody.layer.layer;

		if (layer == OblivionLayer.OL_ANIM_STATIC) {
			if (bhkRigidBody.mass == 0) {
				bhkShape bhkShape = (bhkShape)niToJ3dData.get(bhkRigidBody.shape);
				colShape = BhkShapeToCollisionShape.processBhkShape(bhkShape, niToJ3dData.getNiObjects(),
						fixedScaleFactor);

				setRigidBody(NifBulletUtil.createRigidBody(bhkRigidBody, colShape, calcWorldTransform(), this));

				SceneGraphTransformChangeBehavior behave = new SceneGraphTransformChangeBehavior(parentJ3dNiAVObject,
						this, niToJ3dData);

				behaviourRootGroup.addChild(behave);
				behave.setSchedulingBounds(Utils3D.defaultBounds);
				behave.setEnable(true);
			} else {
				new Throwable("bhkRigidBody.mass != 0 " + this).printStackTrace();
			}
		} else {
			new Throwable("Why is a non OL_ANIM_STATIC handed to  me? " + layer + " " + this).printStackTrace();
		}
	}

	public NBKinematicRigidBody(Group behaviourRootGroup, J3dNiAVObject j3dNiNodeRoot,
								bhkNPCollisionObject bhkNPCollisionObject, NiToJ3dData niToJ3dData,
								bhkPhysicsSystem bhkPhysicsSystem, int bodyId, Transform3D rootTrans,
								BulletNifModel parentModel, float fixedScaleFactor) {
		super(parentModel);

		// null NifVer hence Skyrim scale
		this.nifVer = null;

		this.scale = fixedScaleFactor;
		this.j3dNiNodeRoot = j3dNiNodeRoot;
		this.rootTrans = rootTrans;

		HKXContents contents = bhkPhysicsSystem.hkxContents;
		if (contents != null) {
			hknpPhysicsSystemData hknpPhysicsSystemData = (hknpPhysicsSystemData)contents.getContentCollection()
					.iterator().next();
			hknpBodyCinfo[] bodyCinfos = hknpPhysicsSystemData.bodyCinfos;
			hknpMaterial[] materials = hknpPhysicsSystemData.materials;

			hknpBodyCinfo hknpBodyCinfo = bodyCinfos[bodyId];
			hknpMaterial hknpMaterial = materials[bodyId];

			long shapeId = hknpBodyCinfo.shape;
			if (shapeId > 0) {
				parentJ3dNiAVObject = niToJ3dData.get((NiAVObject)niToJ3dData.get(bhkNPCollisionObject.target));

				hknpShape hknpShape = (hknpShape)contents.get(shapeId);
				colShape = hkxShapeToCollisionShape.processBhkShape(hknpShape, contents, niToJ3dData.nifVer,
						fixedScaleFactor);
				setRigidBody(NifBulletUtil.createRigidBody(hknpBodyCinfo, null, hknpMaterial, colShape,
						calcWorldTransform(), this));

				SceneGraphTransformChangeBehavior behave = new SceneGraphTransformChangeBehavior(parentJ3dNiAVObject,
						this, niToJ3dData);

				behaviourRootGroup.addChild(behave);
				behave.setSchedulingBounds(Utils3D.defaultBounds);
				behave.setEnable(true);
			} else {
				new Throwable("shapeId <= 0" + this).printStackTrace();
			}

		}
	}

	// for tes3 doors
	public NBKinematicRigidBody(Group behaviourRootGroup, TransformGroup simpleTG, RootCollisionNode rootCollisionNode,
								NiToJ3dData niToJ3dData, Transform3D rootTrans, BulletNifModel parentModel,
								float fixedScaleFactor) {
		super(parentModel);

		this.scale = fixedScaleFactor;
		this.rootTrans = rootTrans;
		this.simpleTG = simpleTG;

		colShape = RootCollisionNodeToCollisionShape.processRootCollisionNode(rootCollisionNode,
				niToJ3dData.getNiObjects(), 1f);
		DefaultMotionState myMotionState = new DefaultMotionState(calcWorldTransform());
		RigidBody rigidBody = new RigidBody(new RigidBodyConstructionInfo(0, myMotionState, colShape));
		rigidBody.setCollisionFlags(CollisionFlags.KINEMATIC_OBJECT);
		setRigidBody(rigidBody);

		SceneGraphTransformChangeBehavior behave = new SceneGraphTransformChangeBehavior(simpleTG, this, niToJ3dData);

		behaviourRootGroup.addChild(behave);
		behave.setSchedulingBounds(Utils3D.defaultBounds);
		behave.setEnable(true);

	}

	@Override
	public void updateRootTransform(Transform3D newRootTrans) {
		rootTrans.set(newRootTrans);
		updateInternalWorldTransform();
	}

	/**
	 * Notice that no new rootTransform is handed in, this method updates the internals 
	 * based on animations, it is not a method to update rootTransform
	 */
	public void updateInternalWorldTransform() {
		getRigidBody().getMotionState().setWorldTransform(calcWorldTransform());
		getRigidBody().activate(true);
	}

	//deburners
	private Transform3D	worldTransformCalc	= new Transform3D();

	private Transform	worldTransform		= NifBulletUtil.newIdentityTransform();

	/** 
	 * go up to the root of the model adding in all transforms then add o the root transform (models location in world)	
	 * NOTICE this use the altering j3dNiAVObjects
	 * Note the fixed rotrs that static equivilent uses
	 * @param rootTrans
	 * @return
	 */

	private Transform calcWorldTransform() {
		// add the root trans in
		worldTransformCalc.set(rootTrans);

		if (parentJ3dNiAVObject != null) {
			// get teh transform up to the root
			Transform3D temp = new Transform3D();
			parentJ3dNiAVObject.getTreeTransform(temp, j3dNiNodeRoot);
			worldTransformCalc.mul(temp);

			// add in the rigid body trans
			if (getBhkRigidBody() instanceof bhkRigidBodyT) {
				temp.set(ConvertFromHavok.toJ3d(getBhkRigidBody().rotation),
						ConvertFromHavok.toJ3d(getBhkRigidBody().translation, scale, nifVer), 1f);
				worldTransformCalc.mul(temp);
			}
		} else if (simpleTG != null) {
			Transform3D temp = new Transform3D();
			simpleTG.getTransform(temp);
			worldTransformCalc.mul(temp);
		}

		worldTransformCalc.get(worldTransform.origin);
		worldTransformCalc.get(worldTransform.basis);
		return worldTransform;

	}

}
