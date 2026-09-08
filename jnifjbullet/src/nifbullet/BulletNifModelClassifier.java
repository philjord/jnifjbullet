package nifbullet;

import org.jogamp.java3d.Transform3D;

import nif.NifFile;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.j3d.NiToJ3dData;
import nif.niobject.NiBone;
import nif.niobject.NiNode;
import nif.niobject.NiObject;
import nif.niobject.NiSkinInstance;
import nif.niobject.RootCollisionNode;
import nif.niobject.bhk.bhkConstraint;
import nif.niobject.bhk.bhkPhysicsSystem;
import nif.niobject.bhk.bhkRigidBody;
import nif.niobject.bs.BSTreeNode;
import nif.niobject.controller.NiMultiTargetTransformController;
import nif.niobject.controller.NiTimeController;
import nif.niobject.controller.NiTransformController;
import nif.niobject.hkx.hknpBodyCinfo;
import nif.niobject.hkx.hknpConstraintCinfo;
import nif.niobject.hkx.hknpMotionCinfo;
import nif.niobject.hkx.hknpPhysicsSystemData;
import nif.niobject.hkx.reader.HKXContents;
import nifbullet.dyn.NBSimpleDynamicModel;
import nifbullet.simple.NBSimpleModel;
import utils.source.MeshSource;

public class BulletNifModelClassifier {

	//There are the follow classes of dynamics to consider

	// A/ single rigidbody with mass, collisonobject parent = root object (e.g. clutter) -Simple Dynamic

	// B/ single or multiple rigid bodies, 0 mass, collision objects go up through ninode to root,
	// no controllers bullet - Static

	// C/ single or multiple rigid bodies,  0 mass, collision objects go up through ninode to root,
	// controllers exist - door, crane etc called kinematic by bullet -Kinematic

	// D/ characters  

	// E/ multiple dynamic rigid bodies with constraints - armourgo.nif clothes on the ground or ragdolls -Complex Dynamic

	// F/ multiple dynamic rigid bodies with static connecting to the environment - plants, chain dolls -Complex Dynamic

	// G/ multiple dynamic bodies without contraints, (crazy ragdoll death of storm actronach?) so one of them represents the RECO location?
	// possibly the nonaccum ninodes are part of what dictates the "pelvis" point is -Complex Dynamic

	/*
	 * So we see that 
	 * A need an instreco update from the one body - MISC - layer or OL_CLUTTER or OL_PROP
	 * B needs no listeners and cannot ever have root changed (must be removed added) - (this is the bulk of things) layer of OL_STATIC
	 * C needs the bodies updated from the ninode animation updates
	 * D needs special treatment jbullet character demo KinematicCharacterController
	 * E needs the main rigid body to update the instreco, but also needs the other ninodes updated from the rigidbodies
	 * F needs only the nionodes updated from the bodies, the stats can be ignored like statics, no instreco update
	 * G is a complex dynamic
	 * E,F,G all require parts of the visual model (J3dNiNode likes) to listen to parts of teh physics, which is a much 
	 * finer grain than the current InstRECO level we have now (though E and G stil need InstRECO level updates) and will need the
	 * visual and havok(+bullet) model built together and will only be for client side work 
	 * 
	 * C example C:\game media\Fallout\meshes\dungeons\enclave\rooms\ecvdoorsm01.nif
	 * 
	 * E I see that in the case of clothes  C:\game media\Fallout\meshes\armor\1950stylecasual01\m\go.nif they use bones but do
	 * not have a body target of the top level ninode, in fact they use a skin instance. For now I skip skininstance files and anything with Bone01 type names needs to be skipped too?
	 *  
	 * I can't think what a comlex dynamic would be? multiple transform controllers is still simple
	 * 	 
	 * 
	 * Static bodies will never be listened to for updates, obviously
	 */

	// BULLET note in nif files there is a many bhkCollisionObject each has one rigidbody child,
	// each ridgidbody child has constraints to other rigid bodies in the tree (with a single bhkCollisionObject parent)
	// see chaindollarena01 in chedenhal for interesting havok

	// bhkWorldObject has a layer value that tells you what it is see NifBulletbhkRigidBody.OL_STATIC etc

	// 

	private NifFile		nifFile;
	private NiToJ3dData	niToJ3dData;
	private PhysicsInfo	physicsInfo;

	/** 
	 * For Debug use only you should really have the nifFile in hand 
	 * @param filename
	 * @param meshSource
	 */
	public BulletNifModelClassifier(String filename, MeshSource meshSource) {
		this(NifToJ3d.loadNiObjects(filename, meshSource));
	}

	public BulletNifModelClassifier(NifFile nifFile) {
		if (nifFile != null) {
			this.nifFile = nifFile;
			niToJ3dData = new NiToJ3dData(nifFile.blocks);
			this.physicsInfo = getPhysicsInfo(niToJ3dData);
		}
	}

	//no rigids at all

	public boolean isNotPhysics() {
		boolean ret = false;
		if (nifFile != null) {
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode) {
				ret = physicsInfo.rigidBodyCount == 0;
			}
		}
		return ret;
	}

	//no massed rigids, at least 1 non massed
	//OL_STATIC layer and L_STAIRS, OL_TERRAIN, OL_LINE_OF_SIGHT, plus more
	//no constraints
	//no bones or skins

	//TODO: need to handle switch nodes
	//F:\game media\skyrim\meshes\landscape\trees\treepineforest03.nif
	//F:\game media\Oblivion\meshes\Plants\FloraPrimrosePurple.NIF

	public boolean isStaticModel() {
		boolean ret = false;
		if (nifFile != null) {
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode) {
				ret = physicsInfo.massedRigidBodyCount == 0 && //
						physicsInfo.nonMassedRigidBodyCount > 0 && //
						isOnlyAllowedLayers(niToJ3dData,
								new int[] {OblivionLayer.OL_STATIC, OblivionLayer.OL_LINE_OF_SIGHT,
									OblivionLayer.OL_UNIDENTIFIED, OblivionLayer.OL_STAIRS, OblivionLayer.OL_TERRAIN,
									OblivionLayer.OL_TRANSPARENT, OblivionLayer.OL_TREES})
						&& //
						physicsInfo.constraintCount == 0;// 

				//getSkinAndBoneCount(niToJ3dData) == 0; // trees can be skinned but have simple phys

				//	System.out.println("getMassedRigidBodyCount " + getMassedRigidBodyCount(niToJ3dData));
				//	System.out.println("getNonMassedRigidBodyCount " + getNonMassedRigidBodyCount(niToJ3dData));
				//	System.out.println("isOnlyAllowedLayers "
				//			+ isOnlyAllowedLayers(niToJ3dData, new int[]
				//			{ OblivionLayer.OL_STATIC, OblivionLayer.OL_LINE_OF_SIGHT, OblivionLayer.OL_UNIDENTIFIED, OblivionLayer.OL_STAIRS,
				//					OblivionLayer.OL_TERRAIN, OblivionLayer.OL_TRANSPARENT, OblivionLayer.OL_TREES }));
				//	System.out.println("getConstraintCount " + getConstraintCount(niToJ3dData));
				//	System.out.println("getSkinAndBoneCount " + getSkinAndBoneCount(niToJ3dData));

			}
		}
		return ret;
	}

	//no massed rigids, at least 1 non massed rigid
	//layers allowed OL_ANIM_STATIC, OL_STATIC
	//must have 1 OL_ANIM_STATIC
	//allowed to have transform controllers (of the OL_ANIM_STATIC ninodes), if none then a dud model probably
	//no constraints
	//no bones or skins
	//E.G.   C:\game media\Fallout\meshes\clutter\briefcasedetonator
	//or C:\game media\Fallout\meshes\dungeons\office\doors

	public boolean isKinematicModel() {
		boolean ret = false;
		if (nifFile != null) {
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode) {
				ret = physicsInfo.massedRigidBodyCount == 0 && //
						physicsInfo.nonMassedRigidBodyCount > 0 && //
						physicsInfo.rigidBodyCount - (getLayerCount(niToJ3dData, OblivionLayer.OL_STATIC) + //                                                                               
														getLayerCount(niToJ3dData, OblivionLayer.OL_ANIM_STATIC)) == 0
						&& // 
						getLayerCount(niToJ3dData, OblivionLayer.OL_ANIM_STATIC) > 0 && // 
						physicsInfo.transformControllerCount >= 0 && //
						physicsInfo.constraintCount == 0;

			}
		}
		return ret;
	}

	// 1 only massed rigid (and is directly off root?), has 0 non massed (unless a forced mass used)
	// 1 total OL_PROP or OL_CLUTTER layer  (I think clutter can be picked and props can't?)(unless a forced mass used)
	// no transform controllers
	// no constraints
	// no bones or skins

	public boolean isSimpleDynamicModel(float forcedMass) {
		boolean ret = false;
		if (nifFile != null) {
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode) {
				//or it has a forced mass which flips this to a dynamic from any layer type
				ret = (physicsInfo.massedRigidBodyCount == 1 && //
						physicsInfo.nonMassedRigidBodyCount == 0 && //
						getLayerCount(niToJ3dData, OblivionLayer.OL_PROPS) + getLayerCount(niToJ3dData,
								OblivionLayer.OL_CLUTTER)//
						// apprently some static have mass in skyrim see Clutter\Silver\SilverCandleStick01.nif
																	+ getLayerCount(niToJ3dData,
																			OblivionLayer.OL_STATIC) == 1 //
						|| forcedMass != 0) && //
						physicsInfo.transformControllerCount == 0 && //
						physicsInfo.constraintCount == 0 && //
						physicsInfo.skinAndBoneCount == 0;

				//TODO: check for one rigid collision off the root
			}
		}
		return ret;
	}

	// dynamics and static, but at least one dynamic
	// not sure what sort of layers to allow, statics need to be static 
	// constraints can exist (storm atronach has none in death for example)
	// bones allowed for visual rendering
	// any of the go.nif files under armor, trainign dummy, ragdolls storm atronach

	public boolean isComplexDynamic() {
		boolean ret = false;
		if (nifFile != null) {
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode) {
				ret = physicsInfo.massedRigidBodyCount > 0 && //
						physicsInfo.nonMassedRigidBodyCount == 0 && //
						physicsInfo.transformControllerCount == 0 && //
						physicsInfo.constraintCount > 0;
				/*ret = getMassedRigidBodyCount(niToJ3dData) > 0 && //
						getNonMassedRigidBodyCount(niToJ3dData) == 0 && //
						getTransformControllerCount(niToJ3dData) == 0 && //
						getConstraintCount(niToJ3dData) > 0;*/
			}
		}
		return ret;
	}

	private static class PhysicsInfo {
		public int	rigidBodyCount				= 0;
		public int	massedRigidBodyCount		= 0;
		public int	nonMassedRigidBodyCount		= 0;
		public int	transformControllerCount	= 0;
		public int	constraintCount				= 0;
		public int	skinAndBoneCount			= 0;
	}

	private static PhysicsInfo getPhysicsInfo(NiToJ3dData niToJ3dData) {
		PhysicsInfo physicsInfo = new PhysicsInfo();
		for (NiObject niObject : niToJ3dData.getNiObjects()) {
			if (niObject instanceof bhkRigidBody) {
				bhkRigidBody bhkRigidBody = (bhkRigidBody)niObject;
				physicsInfo.rigidBodyCount++;
				physicsInfo.massedRigidBodyCount += (bhkRigidBody.mass > 0 ? 1 : 0);
				physicsInfo.nonMassedRigidBodyCount += (bhkRigidBody.mass == 0 ? 1 : 0); // note count of 0 mass				
			} else if (niObject instanceof RootCollisionNode) {
				RootCollisionNode rootCollisionNode = (RootCollisionNode)niObject;
				physicsInfo.rigidBodyCount += rootCollisionNode.numChildren;
				physicsInfo.nonMassedRigidBodyCount += rootCollisionNode.numChildren;// all children are non massed rigids
			} else if (niObject instanceof bhkPhysicsSystem) {
				bhkPhysicsSystem bhkPhysicsSystem = (bhkPhysicsSystem)niObject;

				HKXContents contents = bhkPhysicsSystem.hkxContents;
				// the first one had better be a system
				hknpPhysicsSystemData hknpPhysicsSystemData = (hknpPhysicsSystemData)contents.getContentCollection()
						.iterator().next();
				hknpBodyCinfo[] bodyCinfos = hknpPhysicsSystemData.bodyCinfos;
				hknpMotionCinfo[] motionCinfos = hknpPhysicsSystemData.motionCinfos;

				physicsInfo.rigidBodyCount = hknpPhysicsSystemData.bodyCinfos.length;

				for (int i = 0; i < bodyCinfos.length; i++) {
					if (motionCinfos != null	&& bodyCinfos[i].motionId < motionCinfos.length
						&& motionCinfos[bodyCinfos[i].motionId].inverseMass != 0) {
						physicsInfo.massedRigidBodyCount++;
					} else {
						physicsInfo.nonMassedRigidBodyCount++;
					}
				}

				hknpConstraintCinfo[] hknpConstraintCinfos = hknpPhysicsSystemData.constraintCinfos;
				// if we have motion for all parts then we are not nonmassed, but massed
				if (hknpConstraintCinfos != null)
					physicsInfo.constraintCount += hknpConstraintCinfos.length;

			} else if (niObject instanceof NiTransformController
						|| niObject instanceof NiMultiTargetTransformController) {
				//TODO: check for dud entries, no controller or no interpolator, is teh below correct?
				// what about other position controllers and what about extra targets in multi?
				NiObject target = niToJ3dData.get(((NiTimeController)niObject).target);
				if (target != null) {
					physicsInfo.transformControllerCount++;
				}
			} else if (niObject instanceof bhkConstraint) {
				//TODO: check for dud entries, at least one side of constraint must attach to a rigid body
				physicsInfo.constraintCount++;
			} else if (niObject instanceof NiSkinInstance || niObject instanceof NiBone) {
				physicsInfo.skinAndBoneCount++;
			}

		}
		return physicsInfo;
	}

	private static int getLayerCount(NiToJ3dData niToJ3dData, int layer) {
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects()) {
			if (niObject instanceof bhkRigidBody) {
				bhkRigidBody bhkRigidBody = (bhkRigidBody)niObject;

				if (bhkRigidBody.layer.layer == layer) {
					ret++;
				}
			} else if (niObject instanceof bhkPhysicsSystem) {
				// a differnet system entirely, it's all a bit rubbish, 
				//try to return 1 in this case only to ht the dynamic test
				if (layer == OblivionLayer.OL_PROPS)
					return 1;

			}
		}
		return ret;
	}

	private static boolean isOnlyAllowedLayers(NiToJ3dData niToJ3dData, int[] allowedLayers) {
		//return true if a RootCollisionNode exists (morrowind system, no layers all statics)
		//FO4 + has a physic system that's not about layers
		for (NiObject niObject : niToJ3dData.getNiObjects()) {
			if (niObject instanceof RootCollisionNode) {
				return true;
			} else if (niObject instanceof bhkPhysicsSystem) {
				return true;
			}
		}

		int countOfAllowed = 0;
		for (int l : allowedLayers) {
			countOfAllowed += getLayerCount(niToJ3dData, l);
		}
		PhysicsInfo physicsInfo = getPhysicsInfo(niToJ3dData);
		return (countOfAllowed - physicsInfo.rigidBodyCount) == 0;
	}

	public static void testNif(String filename, MeshSource meshSource) {
		BulletNifModelClassifier bulletNifModelClassifier = new BulletNifModelClassifier(filename, meshSource);
		int categoryCount = 0;

		// Note no Else's because we want to catch double classification mistakes
		if (bulletNifModelClassifier.isNotPhysics()) {
			categoryCount++;
			System.out.println("isNotPhysics");
		}

		if (bulletNifModelClassifier.isStaticModel()) {
			categoryCount++;
			System.out.println("isStaticModel");
		}

		if (bulletNifModelClassifier.isKinematicModel()) {
			categoryCount++;
			System.out.println("isKinematicModel");
		}

		if (bulletNifModelClassifier.isSimpleDynamicModel(0)) {
			categoryCount++;
			System.out.println("isSimpleDynamicModel");
		}

		if (bulletNifModelClassifier.isComplexDynamic()) {
			categoryCount++;
			System.out.println("isComplexDynamic");
		}

		if (categoryCount != 1) {
			//TODO: E:\game media\Oblivion\meshes\architecture\arena\arenacolumn01.nif gives count0
			System.err.println("Bad category count for file!!" + categoryCount);
		}
		return;

	}

	public static BulletNifModel createNifBullet(String filename, MeshSource meshSource, float forcedMass) {

		BulletNifModelClassifier bulletNifModelClassifier = new BulletNifModelClassifier(filename, meshSource);
		if (bulletNifModelClassifier.isNotPhysics()) {
			System.out.println("is not physics");
			return null;
		} else if (bulletNifModelClassifier.isStaticModel() || bulletNifModelClassifier.isKinematicModel()) {
			return new NBSimpleModel(filename, meshSource, new Transform3D());
		} else if (bulletNifModelClassifier.isSimpleDynamicModel(forcedMass)) {
			return new NBSimpleDynamicModel(filename, meshSource, forcedMass);
		} else if (bulletNifModelClassifier.isComplexDynamic()) {
			System.out.println("createNifBullet isComplexDynamic not yet! " + filename);
			return null;
		}

		System.err.println("Bad category for file isNotPhysics didn't catch it!!" + filename);
		return null;

	}
}
