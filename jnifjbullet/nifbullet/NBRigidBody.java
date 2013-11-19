package nifbullet;

import nif.niobject.bhk.bhkRigidBody;

import com.bulletphysics.dynamics.RigidBody;

public class NBRigidBody
{
	private BulletNifModel parentModel;

	private RigidBody rigidBody;

	private bhkRigidBody bhkRigidBody;

	protected float fixedScaleFactor = 1.0f;

	public NBRigidBody(BulletNifModel parentModel, float fixedScaleFactor)
	{
		this.parentModel = parentModel;
		this.fixedScaleFactor = fixedScaleFactor;
	}

	public BulletNifModel getParentModel()
	{
		return parentModel;
	}

	public void destroy()
	{
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
}
