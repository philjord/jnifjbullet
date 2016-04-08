package nifbullet;

import javax.media.j3d.Transform3D;

import nif.niobject.bhk.bhkRigidBody;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.RigidBody;

public abstract class NBRigidBody
{
	private BulletNifModel parentModel;

	private RigidBody rigidBody;

	private bhkRigidBody bhkRigidBody;

	protected float scale = 1f;

	// root shape to allow multi parts to be added as required
	protected CollisionShape colShape;

	public NBRigidBody(BulletNifModel parentModel)
	{
		this.parentModel = parentModel;
	}

	public BulletNifModel getParentModel()
	{
		return parentModel;
	}

	public void destroy()
	{
		if (rigidBody != null)
			rigidBody.destroy();
	}

	public RigidBody getRigidBody()
	{
		return rigidBody;
	}

	protected bhkRigidBody getBhkRigidBody()
	{
		return bhkRigidBody;
	}

	protected void setBhkRigidBody(bhkRigidBody bhkRigidBody)
	{
		this.bhkRigidBody = bhkRigidBody;
	}

	protected void setRigidBody(RigidBody rigidBody)
	{
		this.rigidBody = rigidBody;
		rigidBody.setUserPointer(this);
	}

	public CollisionShape getColShape()
	{
		return colShape;
	}

	public abstract void updateRootTransform(Transform3D rootTrans);

}
