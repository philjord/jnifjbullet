package nifbullet.dyn;

import javax.vecmath.Vector3f;

import org.jogamp.java3d.Transform3D;

/**
 * Interface for nifbullet to update a listener
 * @author philip
 *
 */
public interface NifBulletTransformListener
{
	public void transformChanged(Transform3D trans, Vector3f linearVelocity, Vector3f rotationalVelocity);
}
