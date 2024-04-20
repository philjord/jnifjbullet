package nifbullet.convert;

import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;

import com.bulletphysics.collision.shapes.CollisionShape;

import nif.NifVer;
import nif.niobject.hkx.hknpCapsuleShape;
import nif.niobject.hkx.hknpCompressedMeshShape;
import nif.niobject.hkx.hknpCompressedMeshShapeData;
import nif.niobject.hkx.hknpConvexPolytopeShape;
import nif.niobject.hkx.hknpDynamicCompoundShape;
import nif.niobject.hkx.hknpScaledConvexShape;
import nif.niobject.hkx.hknpShape;
import nif.niobject.hkx.hknpSphereShape;
import nif.niobject.hkx.hknpStaticCompoundShape;
import nif.niobject.hkx.reader.HKXContents;

public abstract class hkxShapeToCollisionShape {
	/**
	 * Convenience for non dynamic shapes
	 * @param bhkShape
	 * @param niToJ3dData
	 * @param scale
	 * @return
	 */

	public static NifVer nifVer = new NifVer("FO4", NifVer.VER_20_2_0_7, 12, 130);

	public static CollisionShape processBhkShape(hknpShape hknpShape, HKXContents contents, float scale) {
		return processBhkShape(hknpShape, contents, false, scale);
	}

	//Any shape can be scaled so preloading is hard, basically we do it for scale=1.0 ONLY
	public static boolean							CACHE_WEAK				= true;
	private static Map<hknpShape, CollisionShape>	preloadedScale1Shapes	= Collections
			.synchronizedMap(new WeakHashMap<hknpShape, CollisionShape>());

	public static CollisionShape processBhkShape(	hknpShape hknpShape, HKXContents contents, boolean isDynamic,
													float scale) {
		CollisionShape ret = null;
		if (scale == 1.0f) {
			ret = preloadedScale1Shapes.get(hknpShape);
			if (ret != null)
				return ret;
		}

		ret = createCollisionShape(hknpShape, contents, isDynamic, scale);

		if (scale == 1.0f) {
			if (ret != null)
				if (CACHE_WEAK)
					preloadedScale1Shapes.put(hknpShape, ret);
		}
		return ret;
	}

	private static CollisionShape createCollisionShape(	hknpShape hknpShape, HKXContents contents, boolean isDynamic,
														float scale) {
		
		if (hknpShape instanceof hknpSphereShape) {
			return hkxCollisionToNifBullet.hknpSphereShape((hknpSphereShape)hknpShape, scale, nifVer);
		} else if (hknpShape instanceof hknpCapsuleShape) {
			return hkxCollisionToNifBullet.hknpCapsuleShape((hknpCapsuleShape)hknpShape, scale, nifVer);
		} else	if (hknpShape instanceof hknpDynamicCompoundShape) {
			System.out.println("hknpDynamicCompoundShape!");
			//TODO:
			//bhkTransformShape((bhkTransformShape) bhkShape, group, niToJ3dData);
			return null;
		} else	if (hknpShape instanceof hknpStaticCompoundShape) {
			System.out.println("hknpStaticCompoundShape!");
			//TODO:
			//bhkTransformShape((bhkTransformShape) bhkShape, group, niToJ3dData);
			return null;
		} else if (hknpShape instanceof hknpScaledConvexShape) {
			System.out.println("hknpScaledConvexShape!");
			//TODO:
			//hknpScaledConvexShape((hknpScaledConvexShape) bhkShape, group, niToJ3dData);
			return null;
		} else if (hknpShape instanceof hknpCompressedMeshShape) {
			hknpCompressedMeshShape hknpCompressedMeshShape = (hknpCompressedMeshShape)hknpShape;

			if (hknpCompressedMeshShape.data > 0) {
				hknpCompressedMeshShapeData hknpCompressedMeshShapeData = (hknpCompressedMeshShapeData)contents
						.get(hknpCompressedMeshShape.data);

				return hkxCollisionToNifBullet.hknpCompressedMeshShape(hknpCompressedMeshShapeData, isDynamic, scale,
						nifVer);
			}
			System.out.println("hknpCompressedMeshShape.data == -1");
			return null;
		} else if (hknpShape instanceof hknpConvexPolytopeShape) {
			return hkxCollisionToNifBullet.hknpConvexPolytopeShape((hknpConvexPolytopeShape)hknpShape, scale, nifVer);
		}  else {
			System.out.println("NifHavokToj3d - unknown bhkShape " + hknpShape);
			return null;
		}

	}

}
