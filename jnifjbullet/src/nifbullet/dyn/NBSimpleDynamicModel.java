package nifbullet.dyn;

import java.util.Iterator;

import nif.NifFile;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.niobject.NiNode;
import nif.niobject.NiObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkNPCollisionObject;
import nif.niobject.bhk.bhkPhysicsSystem;
import nif.niobject.bhk.bhkRigidBody;
import nif.niobject.hkx.hkBaseObject;
import nif.niobject.hkx.hknpBodyCinfo;
import nif.niobject.hkx.hknpMaterial;
import nif.niobject.hkx.hknpMotionCinfo;
import nif.niobject.hkx.hknpPhysicsSystemData;
import nif.niobject.hkx.reader.HKXContents;
import nifbullet.BulletNifModel;
import nifbullet.BulletNifModelClassifier;
import utils.source.MeshSource;

/**
* @param fileName
* @param meshSource
* @param forcedMass
*/

public class NBSimpleDynamicModel extends NBDynamicModel implements BulletNifModel {

	public NBSimpleDynamicModel(String fileName, MeshSource meshSource, float forcedMass) {
		super(fileName);
		NifFile nifFile = NifToJ3d.loadNiObjects(fileName, meshSource);
		if (nifFile != null) {
			BulletNifModelClassifier bulletNifModelClassifier = new BulletNifModelClassifier(nifFile);
			if (bulletNifModelClassifier.isSimpleDynamicModel(forcedMass)) {

				if (nifFile.blocks.root() instanceof NiNode) {
					for (NiObject niObject : nifFile.blocks.getNiObjects()) {
						//check bhkNPCollisionObject first as it's a sub class of bhkCollisionObject
						if (niObject instanceof bhkNPCollisionObject) {

							bhkNPCollisionObject bhkNPCollisionObject = (bhkNPCollisionObject)niObject;

							// notice variable body has been seconded here
							bhkPhysicsSystem bhkPhysicsSystem = (bhkPhysicsSystem)nifFile.blocks
									.get(bhkNPCollisionObject.body);

							HKXContents contents = bhkPhysicsSystem.hkxContents;
							if (contents != null) {
								Iterator<hkBaseObject> iter = contents.getContentCollection().iterator();
								if (iter.hasNext()) {
									// the first one had better be a system
									hknpPhysicsSystemData hknpPhysicsSystemData = (hknpPhysicsSystemData)iter.next();
									hknpBodyCinfo[] bodyCinfos = hknpPhysicsSystemData.bodyCinfos;
									hknpMotionCinfo[] motionCinfos = hknpPhysicsSystemData.motionCinfos;
									//hknpConstraintCinfo[] hknpConstraintCinfos = hknpPhysicsSystemData.constraintCinfos;
									hknpMaterial[] materials = hknpPhysicsSystemData.materials;

									hknpBodyCinfo hknpBodyCinfo = bodyCinfos[bhkNPCollisionObject.BodyID];
									hknpMotionCinfo hknpMotionCinfo = motionCinfos[hknpBodyCinfo.motionId];
									hknpMaterial hknpMaterial = materials[hknpBodyCinfo.materialId];

									//or inverseMass?
									if (hknpMotionCinfo.massFactor != 0) {
										if (rootDynamicBody != null) {
											new Throwable("Multiple rigid bodies found in a simple dunamic model !!!! "
															+ fileName).printStackTrace();
										} else {

											rootDynamicBody = new NBDynamicRigidBody(
													new NifBulletTransformListenerDelegate(), hknpBodyCinfo,
													hknpMotionCinfo, hknpMaterial, contents, nifFile.blocks.nifVer,
													this, 1.0f, forcedMass);

										}
									} else {
										new Throwable("bhkRigidBody.mass == 0 " + this).printStackTrace();
									}
								}
							}

						} else if (niObject instanceof bhkCollisionObject) {

							//TODO: check for collision being off the root node, otherwise we should be a complex dynamic

							bhkCollisionObject bhkCollisionObject = (bhkCollisionObject)niObject;
							bhkRigidBody bhkRigidBody = (bhkRigidBody)nifFile.blocks.get(bhkCollisionObject.body);
							int layer = bhkRigidBody.layerCopy.layer;
							if (forcedMass != 0 || layer == OblivionLayer.OL_CLUTTER || layer == OblivionLayer.OL_PROPS
							// apprently some static have mass in skyrim see Clutter\Silver\SilverCandleStick01.nif
								|| (layer == OblivionLayer.OL_STATIC && bhkRigidBody.mass > 0) //
							) {
								bhkRigidBody.mass = forcedMass != 0 ? forcedMass : bhkRigidBody.mass;
								if (bhkRigidBody.mass != 0) {
									if (rootDynamicBody != null) {
										new Throwable("Multiple rigid bodies found in a simple dunamic model !!!! "
														+ fileName).printStackTrace();
									} else {
										rootDynamicBody = new NBDynamicRigidBody(
												new NifBulletTransformListenerDelegate(), bhkCollisionObject,
												nifFile.blocks, this, 1.0f, forcedMass);
									}
								} else {
									new Throwable("bhkRigidBody.mass == 0 " + this).printStackTrace();
								}
							} else {
								new Throwable("what is this layer being given to me for? " + layer + " " + this)
										.printStackTrace();
							}
						}
					}
				}
			}
		} else {
			new Exception("NifBulletClasser.isSimpleDynamic = false " + fileName).printStackTrace();
		}

	}

}
