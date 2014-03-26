package nifbullet.dyn;

import nif.NifFile;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.niobject.NiNode;
import nif.niobject.NiObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nifbullet.BulletNifModel;
import nifbullet.BulletNifModelClassifier;
import utils.source.MeshSource;

/**
 * //my ship needs to have a root rigid body that is compound so I can add all apras to it!
*probably a custom thing, note that the apar models probabyl should be rigid? 
*No in fact they should have transformable and static parts? Because a gun added on has a turret base that doesn't swivel
*dynamic rigid bodies need to be simple singles or
*compound, where coumpounfd have child col shapes that can kinematically animate
* much like a kinematic RB, but each child is just a col shape and can have more col shapes
* then I need ot intersect with them to find which is pointed at
* So I really need to be able to "add" file name to this model (or a compound version of it)
* Then when intersections happen find out which one it was (maybe a user object pointer)
* Then be able to change the transform for each sub collision object

* arrayList of static bodies, therefore they never move and other things a jointed from them
* arrayList of dynamic children - rigid bodies with constraints
* and array list of transformable children bodies that are in a non jointed constraint with parent
* 
* A The static plus dynamic is training dummy style
* B dynamic plus jointed dynamic is the clothes fall to floor model (one is master and hence holds "location")
* C one dynamic animated collision shapes 
* 
* Can A have B? yes A is B with a static attached
* Can A have C? yes dual col shapes can exist for static and dynamics
* 
* 
* So we derive this model:
* simple model has one dyn RB which is it's root
* complex modle has multiple dyn RB the first is it's root and there are constraints
* 	The root object is updated by AREF updates, and it reports it's own updates back out
* 	So a complex with more than 1 static RB will have an issue if a force transform comes in 
* 	because it needs to update all statics (based off the start positions) but the dyns should 
* 	be constrained to follow, though big moves might be worth updating all parts by, depends a bit
* 
* Then the dyn RB has 2 types, 
* Simple has a single nif model and is simple
* Multipart is a compound at the top  and can have nifs added to the root and allows user pointers
* It also allows a transform update based on user object.
* Currently it does not have a user ref update call back (as it is not animated or dynamical at the
* compound shape level
* For now all dynRB will be multipart, but statics will not
*  
* 
* /**
 * All complex dynamic objects will be called ragdoll.
Ragdolls basically simply have constraints (or not for skeletons that fall apart) 
Ragdolls can be connected to bones (and hence visuals will look skinish and lovely) 
or ninodes (and visuals will be storm actronach or chain doll or chandelier) physics doesn’t care.
 
Ragdolls can be anchored or un anchored, chandelier or chain doll anchored, character with foot 
caught in bear trap anchored. Basically physics has a constraint attached to a static (or even kinematic) 
results are just whatever is determined by physics. 

Unanchored (and in fact anchored because it’s no cost) have a ninode (or nibone) that is considered the “root”;
updates to this root’s position need to be sent back into model exactly like a simple dynamic  

Note a complex dynamic that is unanchored and has one ninode and no constraints is the current simple dynamic.
So at some point these will merge as 1 model called dynamic. The anchoring to beartraps and kinematics would be
 across 2 instrecos and so would be “above and outside” the nifbulletmodel system.
 
 Each rigid body in a ragdoll needs to be joined via it's NBDynRigid
 Body to it's particular J3dNinode. That J3dNinode is either rigid (ninode based) or bone (ninode with the bone marker (or nibone?)
  Picking however wants the rigidbody to point to the instreco, so I think the user object pointer should point to the NRigidBody 
  and that should hold a model ref, and that should hold an instref
 
 Note for the unconstrained ragdolls (storm atronach) I still need all transforms to come from the root node so the rendering 
 bounds system works, I can't have every ninode trmsformed in teh world coords and a root node at 0,0,0

 * @author philip
 *
 
 * @param fileName
 * @param meshSource
 * @param forcedMass
 */

public class NBComplexDynamicModel extends NBDynamicModel implements BulletNifModel
{

	public NBComplexDynamicModel(String fileName, MeshSource meshSource)
	{
		this(fileName, meshSource, 0);

	}

	public NBComplexDynamicModel(String fileName, MeshSource meshSource, float forcedMass)
	{

		super(fileName);

		if (BulletNifModelClassifier.isSimpleDynamicModel(fileName, meshSource, forcedMass))
		{
			NifFile nifFile = NifToJ3d.loadNiObjects(fileName, meshSource);

			if (nifFile != null)
			{
				if (nifFile.blocks.root() instanceof NiNode)
				{
					for (NiObject niObject : nifFile.blocks.getNiObjects())
					{
						if (niObject instanceof bhkCollisionObject)
						{
							bhkCollisionObject bhkCollisionObject = (bhkCollisionObject) niObject;
							bhkRigidBody bhkRigidBody = (bhkRigidBody) nifFile.blocks.get(bhkCollisionObject.body);
							int layer = bhkRigidBody.layerCopy.layer;
							if (forcedMass != 0 || layer == OblivionLayer.OL_CLUTTER || layer == OblivionLayer.OL_PROPS)
							{
								bhkRigidBody.mass = forcedMass != 0 ? forcedMass : bhkRigidBody.mass;
								if (bhkRigidBody.mass != 0)
								{
									//NOTE for now the first is considered the root 
									if (rootDynamicBody != null)
									{
										rootDynamicBody = new NBDynamicRigidBody(new NifBulletTransformListenerDelegate(),
												bhkCollisionObject, nifFile.blocks, this, 1.0f, forcedMass);
									}
								}
								else
								{
									new Throwable("bhkRigidBody.mass == 0 " + this).printStackTrace();
								}
							}
							else
							{
								new Throwable("what is this layer being given to me for? " + layer + " " + this).printStackTrace();
							}
						}
					}
				}
			}
		}
		else
		{
			new Exception("NifBulletClasser.isSimpleDynamic = false " + fileName).printStackTrace();
		}

	}

}
