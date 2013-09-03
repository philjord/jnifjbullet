package nifbullet.dyn;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3f;

/**
 * Interface for nifbullet to update a listener
 * @author philip
 *
 */
public interface NifBulletTransformListener
{
	public void transformChanged(Transform3D trans, Vector3f linearVelocity, Vector3f rotationalVelocity);
}
