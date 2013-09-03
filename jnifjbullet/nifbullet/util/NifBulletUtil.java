package nifbullet.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3f;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nif.niobject.bhk.bhkRigidBody;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

public abstract class NifBulletUtil
{
	/** for statics only, no motion state created
	 * 
	 * @param bhkRigidBody
	 * @param colShape
	 * @param startTransform
	 * @return
	 */
	public static RigidBody createStaticRigidBody(bhkRigidBody bhkRigidBody, CollisionShape colShape, Object userPointer)
	{
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(0, null, colShape);
		rbInfo.friction = bhkRigidBody.friction;
		rbInfo.restitution = bhkRigidBody.restitution;
		RigidBody rigidBody = new RigidBody(rbInfo);
		rigidBody.setUserPointer(userPointer);
		rigidBody.setCollisionFlags(CollisionFlags.STATIC_OBJECT);
		return rigidBody;
	}

	/**
	 * DO NOT hand static into this call
	 * Note, in space damping needs to be 0! just make sure the damn models are correct!
	 * Collision type will be set to KINEMATIC if mass == 0 
	 * @param bhkRigidBody
	 * @param colShape
	 * @return
	 */
	public static RigidBody createRigidBody(bhkRigidBody bhkRigidBody, CollisionShape colShape, Transform startTransform, Object userPointer)
	{
		float mass = bhkRigidBody.mass;

		Vector3f localInertia = new Vector3f(0, 0, 0);

		// rigidbody is dynamic if and only if mass is non zero
		boolean isDynamic = (mass != 0f);

		if (isDynamic)
		{
			//TODO: BULLET bhkRigidBody.inertia might be better?
			colShape.calculateLocalInertia(mass, localInertia);
		}

		// note I need to set motion states for kinematics to work
		DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

		//TODO: BULLET am I always getting good values from nif, compare with defaults
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
		rbInfo.linearDamping = bhkRigidBody.linearDamping;
		rbInfo.angularDamping = bhkRigidBody.angularDamping;

		rbInfo.friction = bhkRigidBody.friction;
		rbInfo.restitution = bhkRigidBody.restitution;
		rbInfo.linearSleepingThreshold = 0.2f;
		rbInfo.angularSleepingThreshold = 0.25f;

		//TODO: apparently I should remove this when bullet is improved?
		rbInfo.additionalDamping = true;
		rbInfo.additionalDampingFactor = 0.005f;
		rbInfo.additionalLinearDampingThresholdSqr = 0.01f;
		rbInfo.additionalAngularDampingThresholdSqr = 0.01f;
		rbInfo.additionalAngularDampingFactor = 0.01f;

		RigidBody rigidBody = new RigidBody(rbInfo);
		rigidBody.setUserPointer(userPointer);

		if (mass == 0f)
		{
			// set it as kinematic
			rigidBody.setCollisionFlags(CollisionFlags.KINEMATIC_OBJECT);
		}

		return rigidBody;
	}

	public static ByteBuffer convertToByteBuffer(float[] gVertices)
	{
		ByteBuffer buf = ByteBuffer.allocateDirect(gVertices.length * 4).order(ByteOrder.nativeOrder());
		for (int i = 0; i < gVertices.length; i++)
		{
			buf.putFloat(gVertices[i]);
		}
		buf.flip();
		return buf;
	}

	public static ByteBuffer convertToIndexBuffer(int[] gIndices)
	{
		ByteBuffer buf = ByteBuffer.allocateDirect(gIndices.length * 4).order(ByteOrder.nativeOrder());
		for (int i = 0; i < gIndices.length; i++)
		{
			buf.putInt(gIndices[i]);
		}
		buf.flip();
		return buf;
	}

	public static int[] remove2dTriangles(int[] coordIndices)
	{
		int tri2dcount = 0;
		for (int i = 0; i < coordIndices.length; i += 3)
		{
			if (coordIndices[i + 0] == coordIndices[i + 1] || coordIndices[i + 0] == coordIndices[i + 2]
					|| coordIndices[i + 1] == coordIndices[i + 2])
			{
				tri2dcount++;
			}
		}

		int[] ret = new int[coordIndices.length - tri2dcount];
		int idx = 0;
		for (int i = 0; i < coordIndices.length; i += 3)
		{
			if (!(coordIndices[i + 0] == coordIndices[i + 1] || coordIndices[i + 0] == coordIndices[i + 2] || coordIndices[i + 1] == coordIndices[i + 2]))
			{
				ret[idx + 0] = coordIndices[i + 0];
				ret[idx + 1] = coordIndices[i + 1];
				ret[idx + 2] = coordIndices[i + 2];
				idx += 3;
			}
		}
		return ret;
	}

	/*public static void j3dToOdeFloatArray(float[] inArray)
	{
		// make (y = -z) and (z = y) to go from java3d to ode format
		float tempY = 0;
		for (int i = 0; i < inArray.length; i += 3)
		{
			// grab y
			tempY = inArray[i + 1];
			// skip x [i+0]
			inArray[i + 0] = inArray[i + 0];
			inArray[i + 1] = -inArray[i + 2];
			inArray[i + 2] = tempY;
		}

	}*/

	public static boolean isCCW(Vector3f v1, Vector3f v2, Vector3f v3, Vector3f normal)
	{
		Vector3f a = new Vector3f(v1);
		a.sub(v2);
		Vector3f b = new Vector3f(v2);
		b.sub(v3);
		Vector3f cross = new Vector3f();
		cross.cross(a, b);
		cross.normalize();
		normal.normalize();
		// now see how close it is to the handed in normal
		cross.sub(normal);
		return cross.length() < 0.5;
	}

	public static boolean isCCWAroundZ(Vector3f v1, Vector3f v2, Vector3f v3)
	{
		return (v2.x - v1.x) * (v3.y - v1.y) - (v2.y - v1.y) * (v3.x - v1.x) > 0;
	}

	public static boolean isCCWAroundZ(Point3f v1, Point3f v2, Point3f v3)
	{
		return (v2.x - v1.x) * (v3.y - v1.y) - (v2.y - v1.y) * (v3.x - v1.x) > 0;
	}

	public static void reverseTriIdxWinding(int[] indices)
	{
		for (int i = 0; i < indices.length; i += 3)
		{
			int temp = indices[i + 1];

			indices[i + 0] = indices[i + 0];
			indices[i + 1] = indices[i + 2];
			indices[i + 2] = temp;
		}
	}

	/**
	 * I convienence method because the default construtor usesthe "busted" emtpy matrix constructor for rotation
	 * @return an identity Matrix3f
	 */
	public static Matrix3f newIdentityMatrix3f()
	{
		Matrix3f ret = new Matrix3f();
		ret.setIdentity();
		return ret;
	}

	/**
	 * I convienence method because the default construtor usesthe "busted" emtpy matrix constructor for rotation
	 * @return an identity transform
	 */
	public static Transform newIdentityTransform()
	{
		Transform ret = new Transform();
		ret.setIdentity();
		return ret;
	}

	public static Transform createTrans(Vector3f v)
	{
		Transform ret = new Transform();
		ret.transform(v);
		ret.origin.set(v);
		return ret;
	}

	public static Transform createTrans(Quat4f q, Vector3f v)
	{
		Matrix3f m = new Matrix3f();
		m.set(q);
		Transform ret = new Transform();
		ret.origin.set(v);
		ret.basis.set(m);
		return ret;
	}

	public static Transform createTrans(Transform3D t3d)
	{
		Transform ret = new Transform();
		t3d.get(ret.origin);
		t3d.get(ret.basis);
		return ret;
	}

	public static Transform3D createTrans(Transform in)
	{
		Transform3D t3d = new Transform3D();
		t3d.setTranslation(in.origin);
		t3d.setRotationScale(in.basis);
		return t3d;
	}

	/**
	 * Transformed in place note
	 * @param in
	 * @return
	 */
	public static Point3f[] transformPoints(Point3f[] in, Transform t)
	{
		Vector3f v = new Vector3f();
		for (int i = 0; i < in.length; i++)
		{
			v.set(in[i]);
			t.transform(v);
			in[i].set(v);
		}
		return in;
	}

	public static float[] makePrimitive(Point3f[] in)
	{
		float[] out = new float[in.length * 3];
		for (int i = 0; i < in.length; i++)
		{
			out[i * 3 + 0] = in[i].x;
			out[i * 3 + 1] = in[i].y;
			out[i * 3 + 2] = in[i].z;
		}
		return out;
	}

	public static float[] makePrimitive(Vector3f[] in)
	{
		float[] out = new float[in.length * 3];
		for (int i = 0; i < in.length; i++)
		{
			out[i * 3 + 0] = in[i].x;
			out[i * 3 + 1] = in[i].y;
			out[i * 3 + 2] = in[i].z;
		}
		return out;
	}

}
