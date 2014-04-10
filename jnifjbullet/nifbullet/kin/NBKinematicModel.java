package nifbullet.kin;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;

import nif.NifJ3dHavokRoot;
import nif.NifToJ3d;
import nif.enums.OblivionLayer;
import nif.j3d.J3dNiAVObject;
import nif.j3d.NiToJ3dData;
import nif.j3d.animation.J3dNiControllerManager;
import nif.niobject.NiObject;
import nif.niobject.bhk.bhkCollisionObject;
import nif.niobject.bhk.bhkRigidBody;
import nifbullet.BulletNifModelClassifier;
import nifbullet.stat.NBStaticOrKinematicModel;
import nifbullet.stat.NBStaticRigidBody;
import utils.source.MeshSource;

public class NBKinematicModel extends NBStaticOrKinematicModel
{
	private NifJ3dHavokRoot nifJ3dRoot;

	public NBKinematicModel(String fileName, MeshSource meshSource, Transform3D rootTrans)
	{
		super(fileName);
		this.setName("NBKinematicModel:" + fileName);

		setCapability(BranchGroup.ALLOW_DETACH);

		if (BulletNifModelClassifier.isKinematicModel(fileName, meshSource))
		{
			nifJ3dRoot = NifToJ3d.loadHavok(fileName, meshSource);
			J3dNiAVObject j3dNiNodeRoot = nifJ3dRoot.getHavokRoot();
			NiToJ3dData niToJ3dData = nifJ3dRoot.getNiToJ3dData();

			for (NiObject niObject : niToJ3dData.getNiObjects())
			{
				if (niObject instanceof bhkCollisionObject)
				{
					bhkCollisionObject bhkCollisionObject = (bhkCollisionObject) niObject;
					bhkRigidBody bhkRigidBody = (bhkRigidBody) niToJ3dData.get(bhkCollisionObject.body);
					int layer = bhkRigidBody.layer.layer;
					if (layer == OblivionLayer.OL_STATIC || layer == OblivionLayer.OL_LINE_OF_SIGHT
							|| layer == OblivionLayer.OL_UNIDENTIFIED || layer == OblivionLayer.OL_STAIRS
							|| layer == OblivionLayer.OL_TERRAIN || layer == OblivionLayer.OL_TRANSPARENT)
					{
						float sf = (float) rootTrans.getScale();
						rootTrans.setScale(1.0f);
						NBStaticRigidBody sb = new NBStaticRigidBody(bhkCollisionObject, niToJ3dData.getNiObjects(), rootTrans, this, sf);
						add(sb);
					}
					else if (layer == OblivionLayer.OL_ANIM_STATIC)
					{
						float sf = (float) rootTrans.getScale();
						rootTrans.setScale(1.0f);
						NBKinematicRigidBody kb = new NBKinematicRigidBody(this, j3dNiNodeRoot, bhkCollisionObject, niToJ3dData, rootTrans,
								this, sf);
						add(kb);
					}
					else
					{
						// skipped 
						new Throwable("what is this layer being given to me for? " + layer + " " + this).printStackTrace();
					}
				}
			}

			// but must be placed so camera dist works too
			j3dNiNodeRoot.getTransformGroup().setTransform(rootTrans);
			//needed for animations to occur			
			addChild(j3dNiNodeRoot);
		}
		else
		{
			new Throwable("NifBulletClasser.isSimpleKinematic = false  " + fileName).printStackTrace();
		}
	}

	public J3dNiControllerManager getJ3dNiControllerManager()
	{
		return nifJ3dRoot.getHavokRoot().getJ3dNiControllerManager();
	}

	public String toString()
	{
		return "NifBullet, file: " + getFileName() + " in class of " + this.getClass();
	}

}
