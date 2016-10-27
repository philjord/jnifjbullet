package nifbullet.dyn;

import org.jogamp.java3d.Transform3D;
import org.jogamp.vecmath.Vector3f;

/**
 * Interface for nifbullet to update a listener
 * @author philip
 *
 */
public interface NifBulletTransformListener
{
	public void transformChanged(Transform3D trans, Vector3f linearVelocity, Vector3f rotationalVelocity);
}
