package nifbullet;

import java.util.Iterator;

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
import nif.niobject.hkx.hkBaseObject;
import nif.niobject.hkx.hknpBodyCinfo;
import nif.niobject.hkx.hknpMotionCinfo;
import nif.niobject.hkx.hknpPhysicsSystemData;
import nif.niobject.hkx.reader.HKXContents;
import nifbullet.dyn.NBSimpleDynamicModel;
import nifbullet.simple.NBSimpleModel;
import utils.source.MeshSource;

public abstract class BulletNifModelClassifier
{

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

	public static void testNif(String filename, MeshSource meshSource)
	{
		int categoryCount = 0;

		// Note no Else's because we want to catch double classification mistakes
		if (isNotPhysics(filename, meshSource))
		{
			categoryCount++;
			System.out.println("isNotPhysics");
		}

		if (isStaticModel(filename, meshSource))
		{
			categoryCount++;
			System.out.println("isStaticModel");
		}

		if (isKinematicModel(filename, meshSource))
		{
			categoryCount++;
			System.out.println("isKinematicModel");
		}

		if (isSimpleDynamicModel(filename, meshSource, 0))
		{
			categoryCount++;
			System.out.println("isSimpleDynamicModel");
		}

		if (isComplexDynamic(filename, meshSource))
		{
			categoryCount++;
			System.out.println("isComplexDynamic");
		}

		if (categoryCount != 1)
		{
			//TODO: E:\game media\Oblivion\meshes\architecture\arena\arenacolumn01.nif gives count0
			System.err.println("Bad category count for file!!" + categoryCount);
		}
		return;

	}

	//no rigids at all
	public static boolean isNotPhysics(String filename, MeshSource meshSource)
	{
		NifFile nifFile = NifToJ3d.loadNiObjects(filename, meshSource);

		boolean ret = false;
		if (nifFile != null)
		{
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode)
			{
				NiToJ3dData niToJ3dData = new NiToJ3dData(nifFile.blocks);

				ret = getRigidBodyCount(niToJ3dData) == 0;
			}
		}
		return ret;
	}

	//no massed rigids, at least 1 non massed
	//OL_STATIC layer and L_STAIRS, OL_TERRAIN, OL_LINE_OF_SIGHT, plus more
	//no constraints
	//no bones or skins
	public static boolean isStaticModel(String filename, MeshSource meshSource)
	{
		//TODO: need to handle switch nodes
		//F:\game media\skyrim\meshes\landscape\trees\treepineforest03.nif
		//F:\game media\Oblivion\meshes\Plants\FloraPrimrosePurple.NIF

		NifFile nifFile = NifToJ3d.loadNiObjects(filename, meshSource);
		return isStaticModel(nifFile);
	}

	public static boolean isStaticModel(NifFile nifFile)
	{
		boolean ret = false;
		if (nifFile != null)
		{
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode)
			{
				NiToJ3dData niToJ3dData = new NiToJ3dData(nifFile.blocks);

				ret = getMassedRigidBodyCount(niToJ3dData) == 0 && //
						getNonMassedRigidBodyCount(niToJ3dData) > 0 && //
						isOnlyAllowedLayers(niToJ3dData, new int[]
						{ OblivionLayer.OL_STATIC, OblivionLayer.OL_LINE_OF_SIGHT, OblivionLayer.OL_UNIDENTIFIED, OblivionLayer.OL_STAIRS,
								OblivionLayer.OL_TERRAIN, OblivionLayer.OL_TRANSPARENT, OblivionLayer.OL_TREES }) && //
						getConstraintCount(niToJ3dData) == 0;// 
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
	public static boolean isKinematicModel(String filename, MeshSource meshSource)
	{
		NifFile nifFile = NifToJ3d.loadNiObjects(filename, meshSource);
		return isKinematicModel(nifFile);
	}

	public static boolean isKinematicModel(NifFile nifFile)
	{
		boolean ret = false;
		if (nifFile != null)
		{
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode)
			{
				NiToJ3dData niToJ3dData = new NiToJ3dData(nifFile.blocks);

				ret = getMassedRigidBodyCount(niToJ3dData) == 0 && //
						getNonMassedRigidBodyCount(niToJ3dData) > 0 && //
						getRigidBodyCount(niToJ3dData) - (getLayerCount(niToJ3dData, OblivionLayer.OL_STATIC) + //                                                                               
								getLayerCount(niToJ3dData, OblivionLayer.OL_ANIM_STATIC)) == 0 && // 
						getLayerCount(niToJ3dData, OblivionLayer.OL_ANIM_STATIC) > 0 && // 
						getTransformControllerCount(niToJ3dData) >= 0 && //
						getConstraintCount(niToJ3dData) == 0;
				//getSkinAndBoneCount(niToJ3dData) == 0;
			}
		}
		return ret;
	}

	// 1 only massed rigid (and is directly off root?), has 0 non massed (unless a forced mass used)
	// 1 total OL_PROP or OL_CLUTTER layer  (I think clutter can be picked and props can't?)(unless a forced mass used)
	// no transform controllers
	// no constraints
	// no bones or skins
	public static boolean isSimpleDynamicModel(String filename, MeshSource meshSource, float forcedMass)
	{
		NifFile nifFile = NifToJ3d.loadNiObjects(filename, meshSource);
		return isSimpleDynamicModel(nifFile, forcedMass);
	}

	public static boolean isSimpleDynamicModel(NifFile nifFile, float forcedMass)
	{
		boolean ret = false;
		if (nifFile != null)
		{
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode)
			{
				NiToJ3dData niToJ3dData = new NiToJ3dData(nifFile.blocks);
				//or it has a forced mass which flips this to a dynamic from any layer type

				ret = (getMassedRigidBodyCount(niToJ3dData) == 1 && //
						getNonMassedRigidBodyCount(niToJ3dData) == 0 && //
						getLayerCount(niToJ3dData, OblivionLayer.OL_PROPS) + getLayerCount(niToJ3dData, OblivionLayer.OL_CLUTTER) == 1 //
				|| forcedMass != 0) && //
						getTransformControllerCount(niToJ3dData) == 0 && //
						getConstraintCount(niToJ3dData) == 0 && //
						getSkinAndBoneCount(niToJ3dData) == 0;

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
	public static boolean isComplexDynamic(String filename, MeshSource meshSource)
	{
		NifFile nifFile = NifToJ3d.loadNiObjects(filename, meshSource);
		return isStaticModel(nifFile);
	}

	public static boolean isComplexDynamic(NifFile nifFile)
	{
		boolean ret = false;
		if (nifFile != null)
		{
			if (nifFile.blocks.root() instanceof NiNode || nifFile.blocks.root() instanceof BSTreeNode)
			{
				NiToJ3dData niToJ3dData = new NiToJ3dData(nifFile.blocks);

				ret = getMassedRigidBodyCount(niToJ3dData) > 0 && //
						getNonMassedRigidBodyCount(niToJ3dData) == 0 && //
						getTransformControllerCount(niToJ3dData) == 0 && //
						getConstraintCount(niToJ3dData) > 0;
			}
		}
		return ret;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	private static int getRigidBodyCount(NiToJ3dData niToJ3dData)
	{
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof bhkRigidBody)
			{
				ret++;
			}
			else if (niObject instanceof RootCollisionNode)
			{
				RootCollisionNode rootCollisionNode = (RootCollisionNode) niObject;
				ret += rootCollisionNode.numChildren;
			}
			//TODO: this breaks things?
			/*else if (niObject instanceof bhkPhysicsSystem)
			{
				bhkPhysicsSystem bhkPhysicsSystem = (bhkPhysicsSystem) niObject;
				
				HKXContents contents = bhkPhysicsSystem.hkxContents;
				// the first one had better be a system
				hknpPhysicsSystemData hknpPhysicsSystemData = (hknpPhysicsSystemData)contents.getContentCollection().iterator().next();
				return hknpPhysicsSystemData.bodyCinfos.length;
			}*/
		}
		return ret;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	private static int getMassedRigidBodyCount(NiToJ3dData niToJ3dData)
	{
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof bhkRigidBody)
			{
				bhkRigidBody bhkRigidBody = (bhkRigidBody) niObject;
				ret += (bhkRigidBody.mass > 0 ? 1 : 0);
			}
			/*else if (niObject instanceof bhkPhysicsSystem)
			{
				bhkPhysicsSystem bhkPhysicsSystem = (bhkPhysicsSystem) niObject;
				
				HKXContents contents = bhkPhysicsSystem.hkxContents;
				// the first one had better be a system
				hknpPhysicsSystemData hknpPhysicsSystemData = (hknpPhysicsSystemData)contents.getContentCollection().iterator().next();
				hknpBodyCinfo[] bodyCinfos = hknpPhysicsSystemData.bodyCinfos;
				hknpMotionCinfo[] motionCinfos = hknpPhysicsSystemData.motionCinfos;
				
				// if we have motion for all parts then we are not nonmassed, but massed
				if(motionCinfos != null && (bodyCinfos.length == motionCinfos.length) )
					ret = 1;// in case its a single mesha nd the rampy things			
			}*/
		}
		return ret;
	}

	private static int getNonMassedRigidBodyCount(NiToJ3dData niToJ3dData)
	{
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof bhkRigidBody)
			{
				bhkRigidBody bhkRigidBody = (bhkRigidBody) niObject;
				ret += (bhkRigidBody.mass == 0 ? 1 : 0); // note count of 0 mass
			}
			else if (niObject instanceof RootCollisionNode)
			{
				RootCollisionNode rootCollisionNode = (RootCollisionNode) niObject;
				ret += rootCollisionNode.numChildren;// all children are non massed rigids
			}
			else if (niObject instanceof bhkPhysicsSystem)
			{
				bhkPhysicsSystem bhkPhysicsSystem = (bhkPhysicsSystem) niObject;
				
				HKXContents contents = bhkPhysicsSystem.hkxContents;
				if(contents != null) {
					Iterator<hkBaseObject> iter = contents.getContentCollection().iterator();
					if(iter.hasNext()) {
						// the first one had better be a system
						hknpPhysicsSystemData hknpPhysicsSystemData = (hknpPhysicsSystemData)iter.next();
						hknpBodyCinfo[] bodyCinfos = hknpPhysicsSystemData.bodyCinfos;
						hknpMotionCinfo[] motionCinfos = hknpPhysicsSystemData.motionCinfos;
						
						// if we have motion for all parts then we are not nonmassed, but massed
						if(motionCinfos == null || (bodyCinfos.length != motionCinfos.length) )
							ret += bodyCinfos.length;	
					}
				}
			}
					
		}
		return ret;
	}

	private static int getTransformControllerCount(NiToJ3dData niToJ3dData)
	{
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof NiTransformController || niObject instanceof NiMultiTargetTransformController)
			{
				//TODO: check for dud entries, no controller or no interpolator, is teh below correct?
				// what about other position controllers and what about extra targets in multi?
				NiObject target = niToJ3dData.get(((NiTimeController) niObject).target);
				if (target != null)
				{
					ret++;
				}
			}
		}
		return ret;
	}

	private static int getConstraintCount(NiToJ3dData niToJ3dData)
	{
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof bhkConstraint)
			{
				//TODO: check for dud entries, at least one side of constraint must attach to a rigid body
				ret++;
			}
		}
		return ret;
	}

	private static int getSkinAndBoneCount(NiToJ3dData niToJ3dData)
	{
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof NiSkinInstance || niObject instanceof NiBone)
			{
				ret++;
			}
		}
		return ret;
	}

	private static int getLayerCount(NiToJ3dData niToJ3dData, int layer)
	{
		int ret = 0;
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof bhkRigidBody)
			{
				bhkRigidBody bhkRigidBody = (bhkRigidBody) niObject;

				if (bhkRigidBody.layer.layer == layer)
				{
					ret++;
				}
			}
		}
		return ret;
	}

	private static boolean isOnlyAllowedLayers(NiToJ3dData niToJ3dData, int[] allowedLayers)
	{
		//return true if a RootCollisionNode exists (morrowind system, no layers all statics)
		for (NiObject niObject : niToJ3dData.getNiObjects())
		{
			if (niObject instanceof RootCollisionNode)
			{
				return true;
			}
		}

		int countOfAllowed = 0;
		for (int l : allowedLayers)
		{
			countOfAllowed += getLayerCount(niToJ3dData, l);
		}
		return (countOfAllowed - getRigidBodyCount(niToJ3dData)) == 0;
	}

	public static BulletNifModel createNifBullet(String filename, MeshSource meshSource, float forcedMass)
	{

		if (isNotPhysics(filename, meshSource))
		{
			System.out.println("is not physics");
			return null;
		}
		else if (isStaticModel(filename, meshSource) || isKinematicModel(filename, meshSource))
		{
			return new NBSimpleModel(filename, meshSource, new Transform3D());
		}
		else if (isSimpleDynamicModel(filename, meshSource, forcedMass))
		{
			return new NBSimpleDynamicModel(filename, meshSource, forcedMass);
		}
		else if (isComplexDynamic(filename, meshSource))
		{
			System.out.println("createNifBullet isComplexDynamic not yet! " + filename);
			return null;
		}

		System.err.println("Bad category for file isNotPhysics didn't catch it!!" + filename);
		return null;

	}
}
