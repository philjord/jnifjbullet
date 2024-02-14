package nifbullet.convert;

import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;

import org.jogamp.java3d.Transform3D;
import org.jogamp.vecmath.Matrix4f;

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
	 * @param scale 
	 * @return
	 */

	public static CollisionShape processBhkShape(bhkShape bhkShape, NiObjectList niToJ3dData, float scale)
	{
		return processBhkShape(bhkShape, niToJ3dData, false, scale);
	}

	//Any shape can be scaled so preloading is hard, basically we do it for scale=1.0 ONLY
	public static boolean CACHE_WEAK = true;
	private static Map<bhkShape, CollisionShape> preloadedScale1Shapes = Collections
			.synchronizedMap(new WeakHashMap<bhkShape, CollisionShape>());

	public static CollisionShape processBhkShape(bhkShape bhkShape, NiObjectList niToJ3dData, boolean isDynamic, float scale)
	{
		CollisionShape ret = null;
		if (scale == 1.0f)
		{

			ret = preloadedScale1Shapes.get(bhkShape);
			if (ret != null)
				return ret;
		}

		if (bhkShape instanceof bhkListShape)
		{
			bhkListShape bhkListShape = (bhkListShape) bhkShape;
			CompoundShape cs = new CompoundShape();
			for (int i = 0; i < bhkListShape.numSubShapes; i++)
			{
				// odd that listshape does not transform children
				CollisionShape s = processBhkShape((bhkShape) niToJ3dData.get(bhkListShape.subShapes[i]), niToJ3dData, isDynamic, scale);
				if (s != null)
				{
					Transform idTransform = NifBulletUtil.newIdentityTransform();
					cs.addChildShape(idTransform, s);
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
				CollisionShape s = processBhkShape((bhkShape) niToJ3dData.get(bhkConvexListShape.subShapes[i]), niToJ3dData, isDynamic,
						scale);
				if (s != null)
				{
					Transform idTransform = NifBulletUtil.newIdentityTransform();
					cs.addChildShape(idTransform, s);
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
				cs.addChildShape(idTransform, BhkCollisionToNifBullet.processNiTriStripsData(niTriStripsData, isDynamic, scale));
			}
			ret = cs;
		}
		else if (bhkShape instanceof bhkMoppBvTreeShape)
		{
			bhkMoppBvTreeShape bhkMoppBvTreeShape = (bhkMoppBvTreeShape) bhkShape;
			if (bhkMoppBvTreeShape.shape.ref != -1)
			{
				bhkShape bhkShape2 = (bhkShape) niToJ3dData.get(bhkMoppBvTreeShape.shape);
				ret = processBhkShape(bhkShape2, niToJ3dData, isDynamic, scale);
			}
			else
			{
				System.out.println("bhkMoppBvTreeShape.shape.ref == -1");
				ret = null;
			}
		}
		else
		{
			ret = createCollisionShape(bhkShape, niToJ3dData, isDynamic, scale);
		}

		if (scale == 1.0f)
		{
			if (ret != null)
				if(CACHE_WEAK)
				preloadedScale1Shapes.put(bhkShape, ret);
		}
		return ret;
	}

	private static CollisionShape createCollisionShape(bhkShape bhkShape, NiObjectList niToJ3dData, boolean isDynamic, float scale)
	{
		if (bhkShape instanceof bhkPackedNiTriStripsShape)
		{
			bhkPackedNiTriStripsShape bhkPackedNiTriStripsShape = (bhkPackedNiTriStripsShape) bhkShape;

			if (bhkPackedNiTriStripsShape.data.ref != -1)
			{
				hkPackedNiTriStripsData hkPackedNiTriStripsData = (hkPackedNiTriStripsData) niToJ3dData.get(bhkPackedNiTriStripsShape.data);
				return BhkCollisionToNifBullet.hkPackedNiTriStripsData(hkPackedNiTriStripsData, isDynamic, scale,
						bhkPackedNiTriStripsShape.scale.x, bhkPackedNiTriStripsShape.scale.y, bhkPackedNiTriStripsShape.scale.z,
						niToJ3dData.nifVer);
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

				return BhkCollisionToNifBullet.bhkCompressedMeshShape(bhkCompressedMeshShapeData, isDynamic, scale, niToJ3dData.nifVer);
			}
			System.out.println("bhkCompressedMeshShape.data.ref == -1");
			return null;
		}
		else if (bhkShape instanceof hkPackedNiTriStripsData)
		{
			return BhkCollisionToNifBullet.hkPackedNiTriStripsData((hkPackedNiTriStripsData) bhkShape, isDynamic, scale, 1, 1, 1,
					niToJ3dData.nifVer);
		}
		else if (bhkShape instanceof bhkBoxShape)
		{
			return BhkCollisionToNifBullet.bhkBoxShape((bhkBoxShape) bhkShape, scale, niToJ3dData.nifVer);
		}
		else if (bhkShape instanceof bhkCapsuleShape)
		{
			return BhkCollisionToNifBullet.bhkCapsuleShape((bhkCapsuleShape) bhkShape, scale, niToJ3dData.nifVer);
		}
		else if (bhkShape instanceof bhkSphereShape)
		{
			return BhkCollisionToNifBullet.bhkSphereShape((bhkSphereShape) bhkShape, scale, niToJ3dData.nifVer);
		}
		else if (bhkShape instanceof bhkConvexVerticesShape)
		{
			return BhkCollisionToNifBullet.bhkConvexVerticesShape((bhkConvexVerticesShape) bhkShape, scale, niToJ3dData.nifVer);
		}
		else if (bhkShape instanceof bhkMultiSphereShape)
		{
			return BhkCollisionToNifBullet.bhkMultiSphereShape((bhkMultiSphereShape) bhkShape, scale, niToJ3dData.nifVer);
		}
		else if (bhkShape instanceof bhkTransformShape)
		{
			return bhkTransformShape((bhkTransformShape) bhkShape, niToJ3dData, isDynamic, scale);
		}

		else
		{
			System.out.println("NifHavokToj3d - unknown bhkShape " + bhkShape);
			return null;
		}

	}

	private static CollisionShape bhkTransformShape(bhkTransformShape data, NiObjectList niToJ3dData, boolean isDynamic, float scale)
	{
		if (data.shape.ref != -1)
		{
			bhkShape bhkShape = (bhkShape) niToJ3dData.get(data.shape);
			CollisionShape shape = processBhkShape(bhkShape, niToJ3dData, isDynamic, scale);
			if (shape != null)
			{
				Transform3D t3d = new Transform3D();
				Matrix4f m = ConvertFromHavok.toJ3dM4(data.transform, niToJ3dData.nifVer);
				t3d.set(m);

				CompoundShape cs = new CompoundShape();
				Transform t = NifBulletUtil.createTrans(t3d);

				cs.addChildShape(t, shape);
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
