package nifbullet;

import com.bulletphysics.dynamics.DynamicsWorld;

public interface BulletNifModel
{
	public void destroy();

	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld);

	public void removeFromDynamicsWorld();

}
