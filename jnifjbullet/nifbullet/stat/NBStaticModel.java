package nifbullet.stat;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;

import utils.source.MeshSource;

import com.sun.j3d.utils.geometry.GeometryInfo;

/**
 * TODO: it looks like static and kinematic are basically the same thing, with a bit of an optional animation treee attachment part.
 * In thos case where the layer doesn't suggest kinmatic just ignore the animation half
 * possibly kinematic should sub class static?
 * Note that 
 * @author philip
 *
 */
public class NBStaticModel extends NBStaticOrKinematicModel
{
	/**
	 * 
	 * fileName can be null for statics that are designed only to have parts
	 */
	public NBStaticModel(String filename, MeshSource meshSource, Transform3D rootTrans)
	{
		super(filename);

		setCapability(BranchGroup.ALLOW_DETACH);

		if (filename != null && filename.length() > 0)
		{
			addPart(filename, meshSource, this, rootTrans);
		}
	}

	/**
	 * For land use only! needs to be cleaned up
	 * no parts to be added	 
	 * @param gi
	 * @param rootTrans
	 */
	public NBStaticModel(GeometryInfo gi, Transform3D rootTrans)
	{
		super("LAND");
		if (gi != null)
		{
			NBStaticRigidBody nbbco = new NBStaticRigidBody(gi, rootTrans, this);
			add(nbbco);
		}
	}

	public String toString()
	{
		return "NifBullet, file: " + getFileName() + " class;" + this.getClass().getSimpleName();
	}

}
