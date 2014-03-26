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
* @param fileName
* @param meshSource
* @param forcedMass
*/

public class NBSimpleDynamicModel extends NBDynamicModel implements BulletNifModel
{

	public NBSimpleDynamicModel(String fileName, MeshSource meshSource, float forcedMass)
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
							//TODO: check for collision being off the root node, otherwise we should be a complex dynamic

							bhkCollisionObject bhkCollisionObject = (bhkCollisionObject) niObject;
							bhkRigidBody bhkRigidBody = (bhkRigidBody) nifFile.blocks.get(bhkCollisionObject.body);
							int layer = bhkRigidBody.layerCopy.layer;
							if (forcedMass != 0 || layer == OblivionLayer.OL_CLUTTER || layer == OblivionLayer.OL_PROPS)
							{
								bhkRigidBody.mass = forcedMass != 0 ? forcedMass : bhkRigidBody.mass;
								if (bhkRigidBody.mass != 0)
								{
									if (rootDynamicBody != null)
									{
										new Throwable("Multiple rigid bodies found in a simple dunamic model !!!! " + fileName)
												.printStackTrace();
									}
									else
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
