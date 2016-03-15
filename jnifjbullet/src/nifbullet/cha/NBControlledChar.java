package nifbullet.cha;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nifbullet.util.NifBulletUtil;

import com.bulletphysics.collision.broadphase.CollisionFilterGroups;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.character.KinematicCharacterController;
import com.bulletphysics.dynamics.character.KinematicCharacterController.CharacterPositionListener;
import com.bulletphysics.linearmath.Transform;

public class NBControlledChar implements NifBulletChar
{
	private KinematicCharacterController character;

	private PairCachingGhostObject ghostObject;

	private float stepHeight = 0.4f;

	private DynamicsWorld dynamicsWorld = null;

	public NBControlledChar(Transform3D baseTrans)
	{
		this(baseTrans, 0.9f, 0.5f);

	}

	public NBControlledChar(Transform3D baseTrans, float characterHeight, float characterRadius)
	{
		Transform tr = NifBulletUtil.createTrans(baseTrans);
		tr.basis.setIdentity();// no rotation		

		// capsule shape height is height+(2*radius) so we take the radius out
		ConvexShape capsule = new CapsuleShape(characterRadius, characterHeight - (characterRadius * 2));

		ghostObject = new PairCachingGhostObject();

		ghostObject.setWorldTransform(tr);
		ghostObject.setCollisionShape(capsule);
		ghostObject.setCollisionFlags(CollisionFlags.CHARACTER_OBJECT);

		character = new KinematicCharacterController(ghostObject, capsule, stepHeight);
	}

	public KinematicCharacterController getCharacterController()
	{
		return character;
	}

	public void setCharacterPositionListener(CharacterPositionListener listener)
	{
		character.addCharacterPositionListener(listener);
	}

	public void setTransform(Quat4f q, Vector3f v)
	{
		Transform3D t = new Transform3D(q, v, 1f);
		setTransform(t);
	}

	public void setTransform(Transform3D trans)
	{
		//Transform worldTransform = NifBulletUtil.createTrans(trans);
		//TODO: removed for now allowing phys to move by nav inputs
		//character.warp(worldTransform.origin);
		//I think character flags above has this covered ghostObject.activate(true);
	}

	public void destroy()
	{
		if (dynamicsWorld != null)
		{
			new Throwable("destroy called whilst in dynamic world");
		}

	}

	@Override
	public void addToDynamicsWorld(DynamicsWorld dynamicsWorld1)
	{
		this.dynamicsWorld = dynamicsWorld1;
		dynamicsWorld.addCollisionObject(ghostObject, CollisionFilterGroups.CHARACTER_FILTER,
				(short) (CollisionFilterGroups.STATIC_FILTER | CollisionFilterGroups.DEFAULT_FILTER));
		dynamicsWorld.addAction(character);

	}

	@Override
	public void removeFromDynamicsWorld()
	{
		// check for double remove or no add yet
		if (dynamicsWorld != null)
		{
			dynamicsWorld.removeCollisionObject(ghostObject);
			dynamicsWorld.removeAction(character);
			dynamicsWorld = null;
		}
	}

}
