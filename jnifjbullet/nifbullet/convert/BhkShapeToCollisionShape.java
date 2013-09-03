package nifbullet.convert;

import java.util.WeakHashMap;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nif.NiObjectList;
import nif.niobject.NiTriStripsData;
import nif.niobject.bhk.bhkBoxShape;
import nif.niobject.bhk.bhkCapsuleShape;
import nif.niobject.bhk.bhkCompressedMeshShape;
import nif.niobject.bhk.bhkCompressedMeshShapeData;
import nif.niobject.bhk.bhkConvexListShape;
import nif.niobject.bhk.bhkConvexVerticesShape;
import nif.niobject.bhk.bhkListShape;
import nif.niobject.bhk.bhkMoppBvTreeShape;
import nif.niobject.bhk.bhkMultiSphereShape;
import nif.niobject.bhk.bhkNiTriStripsShape;
import nif.niobject.bhk.bhkPackedNiTriStripsShape;
import nif.niobject.bhk.bhkShape;
import nif.niobject.bhk.bhkSphereShape;
import nif.niobject.bhk.bhkTransformShape;
import nif.niobject.bhk.hkPackedNiTriStripsData;
import nifbullet.util.NifBulletUtil;
import utils.convert.ConvertFromHavok;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.linearmath.Transform;

public abstract class BhkShapeToCollisionShape
{
	/**
	 * Convinience for non dynamic shapes
	 * @param bhkShape
	 * @param niToJ3dData
	 * @return
	 */

	public static CollisionShape processBhkShape(bhkShape bhkShape, NiObjectList niToJ3dData)
	{
		return processBhkShape(bhkShape, niToJ3dData, false);
	}

	private static WeakHashMap<bhkShape, CollisionShape> preloadedShapes = new WeakHashMap<bhkShape, CollisionShape>();

	public static CollisionShape processBhkShape(bhkShape bhkShape, NiObjectList niToJ3dData, boolean isDynamic)
	{
		CollisionShape ret = null;
		ret = preloadedShapes.get(bhkShape);
		if (ret != null)
			return ret;

		if (bhkShape instanceof bhkListShape)
		{
			bhkListShape bhkListShape = (bhkListShape) bhkShape;
			CompoundShape cs = new CompoundShape();
			for (int i = 0; i < bhkListShape.numSubShapes; i++)
			{
				// odd that listshape does not transform children
				CollisionShape s = processBhkShape((bhkShape) niToJ3dData.get(bhkListShape.subShapes[i]), niToJ3dData, isDynamic);
				if (s != null)
				{
					Transform idTransform = NifBulletUtil.newIdentityTransform();
					cs.addChildShape(idTransform, s);
					cs.recalculateLocalAabb();
				}
			}
			ret = cs;
		}
		else if (bhkShape instanceof bhkConvexListShape)
		{
			bhkConvexListShape bhkConvexListShape = (bhkConvexListShape) bhkShape;
			CompoundShape cs = new CompoundShape();
			for (int i = 0; i < bhkConvexListShape.numSubShapes; i++)
			{
				// odd that listshape does not transform children
				CollisionShape s = processBhkShape((bhkShape) niToJ3dData.get(bhkConvexListShape.subShapes[i]), niToJ3dData, isDynamic);
				if (s != null)
				{
					Transform idTransform = NifBulletUtil.newIdentityTransform();
					cs.addChildShape(idTransform, s);
					cs.recalculateLocalAabb();
				}
			}
			ret = cs;
		}
		else if (bhkShape instanceof bhkNiTriStripsShape)
		{
			bhkNiTriStripsShape bhkNiTriStripsShape = (bhkNiTriStripsShape) bhkShape;
			CompoundShape cs = new CompoundShape();
			for (int i = 0; i < bhkNiTriStripsShape.numStripsData; i++)
			{
				NiTriStripsData niTriStripsData = (NiTriStripsData) niToJ3dData.get(bhkNiTriStripsShape.stripsData[i]);
				Transform idTransform = NifBulletUtil.newIdentityTransform();
				cs.addChildShape(idTransform, BhkCollisionToNifBullet.processNiTriStripsData(niTriStripsData, isDynamic));
				cs.recalculateLocalAabb();
			}
			ret = cs;
		}
		else if (bhkShape instanceof bhkMoppBvTreeShape)
		{
			bhkMoppBvTreeShape bhkMoppBvTreeShape = (bhkMoppBvTreeShape) bhkShape;
			if (bhkMoppBvTreeShape.shape.ref != -1)
			{
				bhkShape bhkShape2 = (bhkShape) niToJ3dData.get(bhkMoppBvTreeShape.shape);
				ret = processBhkShape(bhkShape2, niToJ3dData, isDynamic);
			}
			else
			{
				System.out.println("bhkMoppBvTreeShape.shape.ref == -1");
				ret = null;
			}
		}
		else
		{
			ret = createCollisionShape(bhkShape, niToJ3dData, isDynamic);
		}

		if (ret != null)
			preloadedShapes.put(bhkShape, ret);
		return ret;
	}

	private static CollisionShape createCollisionShape(bhkShape bhkShape, NiObjectList niToJ3dData, boolean isDynamic)
	{
		if (bhkShape instanceof bhkPackedNiTriStripsShape)
		{
			bhkPackedNiTriStripsShape bhkPackedNiTriStripsShape = (bhkPackedNiTriStripsShape) bhkShape;

			if (bhkPackedNiTriStripsShape.data.ref != -1)
			{
				hkPackedNiTriStripsData hkPackedNiTriStripsData = (hkPackedNiTriStripsData) niToJ3dData.get(bhkPackedNiTriStripsShape.data);
				return BhkCollisionToNifBullet.hkPackedNiTriStripsData(hkPackedNiTriStripsData, isDynamic);
			}
			System.out.println("bhkPackedNiTriStripsShape.data.ref == -1");
			return null;
		}
		else if (bhkShape instanceof bhkCompressedMeshShape)
		{
			bhkCompressedMeshShape bhkCompressedMeshShape = (bhkCompressedMeshShape) bhkShape;

			if (bhkCompressedMeshShape.data.ref != -1)
			{
				bhkCompressedMeshShapeData bhkCompressedMeshShapeData = (bhkCompressedMeshShapeData) niToJ3dData
						.get(bhkCompressedMeshShape.data);

				return BhkCollisionToNifBullet.bhkCompressedMeshShape(bhkCompressedMeshShapeData, isDynamic);
			}
			System.out.println("bhkCompressedMeshShape.data.ref == -1");
			return null;
		}
		else if (bhkShape instanceof hkPackedNiTriStripsData)
		{
			return BhkCollisionToNifBullet.hkPackedNiTriStripsData((hkPackedNiTriStripsData) bhkShape, isDynamic);
		}
		else if (bhkShape instanceof bhkBoxShape)
		{
			return BhkCollisionToNifBullet.bhkBoxShape((bhkBoxShape) bhkShape);
		}
		else if (bhkShape instanceof bhkCapsuleShape)
		{
			return BhkCollisionToNifBullet.bhkCapsuleShape((bhkCapsuleShape) bhkShape);
		}
		else if (bhkShape instanceof bhkSphereShape)
		{
			return BhkCollisionToNifBullet.bhkSphereShape((bhkSphereShape) bhkShape);
		}
		else if (bhkShape instanceof bhkConvexVerticesShape)
		{
			return BhkCollisionToNifBullet.bhkConvexVerticesShape((bhkConvexVerticesShape) bhkShape);
		}
		else if (bhkShape instanceof bhkMultiSphereShape)
		{
			return BhkCollisionToNifBullet.bhkMultiSphereShape((bhkMultiSphereShape) bhkShape);
		}
		else if (bhkShape instanceof bhkTransformShape)
		{
			return bhkTransformShape((bhkTransformShape) bhkShape, niToJ3dData, isDynamic);
		}

		else
		{
			System.out.println("NifHavokToj3d - unknown bhkShape " + bhkShape);
			return null;
		}

	}

	private static CollisionShape bhkTransformShape(bhkTransformShape data, NiObjectList niToJ3dData, boolean isDynamic)
	{
		if (data.shape.ref != -1)
		{
			bhkShape bhkShape = (bhkShape) niToJ3dData.get(data.shape);
			CollisionShape shape = processBhkShape(bhkShape, niToJ3dData, isDynamic);
			if (shape != null)
			{
				Quat4f q = ConvertFromHavok.toJ3dQ4f(data.transform);
				Vector3f v = ConvertFromHavok.toJ3dV3f(data.transform);

				CompoundShape cs = new CompoundShape();
				Transform t = NifBulletUtil.createTrans(q, v);

				cs.addChildShape(t, shape);
				cs.recalculateLocalAabb();
				return cs;
			}
			else
			{
				System.out.println("shape == null " + bhkShape);
				return null;
			}
		}
		else
		{
			System.out.println("data.shape.ref == -1");
			return null;
		}

	}
}
