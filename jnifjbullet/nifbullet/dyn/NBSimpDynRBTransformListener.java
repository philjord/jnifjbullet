package nifbullet.dyn;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

/**
 * Used by a rigidbody to update a transfromgroup, 
 * question mark over when this is needed given that I've exclude dynamic with constraint and skin types? 
 * Note a nifbullet is optionally also allowed to listen
 * @author philip
 *
 */
public class NBSimpDynRBTransformListener extends DefaultMotionState
{
	private RigidBody rigidBody;

	private NifBulletTransformListener nifBulletTransformListener;

	public NBSimpDynRBTransformListener(NifBulletTransformListener nifBulletTransformListener, RigidBody rigidBody, Transform initialTransform)
	{
		super(initialTransform);
		this.nifBulletTransformListener = nifBulletTransformListener;
		this.rigidBody = rigidBody;
	}

	@Override
	public Transform getWorldTransform(Transform outTrans)
	{
		return super.getWorldTransform(outTrans);
	}

	//deburners
	private Vector3f angularVel = new Vector3f();

	private Vector3f linearVel = new Vector3f();

	private Transform3D temp = new Transform3D();

	@Override
	public void setWorldTransform(Transform worldTrans)
	{
		// jitter checks removed, because JBullet should do this for me
		//	if (!graphicsWorldTrans.origin.epsilonEquals(worldTrans.origin, 0.0001f)
		//			|| !graphicsWorldTrans.basis.epsilonEquals(worldTrans.basis, 0.0001f))
		
		 
		super.setWorldTransform(worldTrans);

		rigidBody.getAngularVelocity(angularVel);
		rigidBody.getLinearVelocity(linearVel);

		temp.set(graphicsWorldTrans.basis);
		temp.invert();
		temp.transform(angularVel);
		temp.transform(linearVel);

		
		temp.set(graphicsWorldTrans.basis, graphicsWorldTrans.origin, 1f);
				
		// Note world coords is correct at the nifbullet listener level
		nifBulletTransformListener.transformChanged(temp, linearVel, angularVel);
	}

}
