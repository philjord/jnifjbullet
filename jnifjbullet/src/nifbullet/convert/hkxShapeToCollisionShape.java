package nifbullet.convert;

import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;

import org.jogamp.java3d.Transform3D;
import org.jogamp.vecmath.Matrix4f;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.linearmath.Transform;

import nif.NifVer;
import nif.niobject.hkx.hknpCapsuleShape;
import nif.niobject.hkx.hknpCompoundShape;
import nif.niobject.hkx.hknpCompressedMeshShape;
import nif.niobject.hkx.hknpCompressedMeshShapeData;
import nif.niobject.hkx.hknpConvexPolytopeShape;
import nif.niobject.hkx.hknpConvexShape;
import nif.niobject.hkx.hknpDynamicCompoundShape;
import nif.niobject.hkx.hknpScaledConvexShape;
import nif.niobject.hkx.hknpShape;
import nif.niobject.hkx.hknpShapeInstance;
import nif.niobject.hkx.hknpSphereShape;
import nif.niobject.hkx.hknpStaticCompoundShape;
import nif.niobject.hkx.reader.HKXContents;
import nifbullet.util.NifBulletUtil;
import utils.convert.ConvertFromHavok;

public abstract class hkxShapeToCollisionShape {
	/**
	 * Convenience for non dynamic shapes
	 * @param bhkShape
	 * @param niToJ3dData
	 * @param scale
	 * @return
	 */

	public static CollisionShape processBhkShape(	hknpShape hknpShape, HKXContents contents, NifVer nifVer,
													float scale) {
		return processBhkShape(hknpShape, contents, nifVer, false, scale);
	}

	//Any shape can be scaled so preloading is hard, basically we do it for scale=1.0 ONLY
	public static boolean							CACHE_WEAK				= true;
	private static Map<hknpShape, CollisionShape>	preloadedScale1Shapes	= Collections
			.synchronizedMap(new WeakHashMap<hknpShape, CollisionShape>());

	public static CollisionShape processBhkShape(	hknpShape hknpShape, HKXContents contents, NifVer nifVer,
													boolean isDynamic, float scale) {
		CollisionShape ret = null;
		if (scale == 1.0f) {
			ret = preloadedScale1Shapes.get(hknpShape);
			if (ret != null)
				return ret;
		}

		ret = createCollisionShape(hknpShape, contents, nifVer, isDynamic, scale);

		if (scale == 1.0f) {
			if (ret != null)
				if (CACHE_WEAK)
					preloadedScale1Shapes.put(hknpShape, ret);
		}
		return ret;
	}

	private static CollisionShape createCollisionShape(	hknpShape hknpShape, HKXContents contents, NifVer nifVer,
														boolean isDynamic, float scale) {

		if (hknpShape instanceof hknpSphereShape) {//BEFORE hknpConvexShape
			return hkxCollisionToNifBullet.hknpSphereShape((hknpSphereShape)hknpShape, scale, nifVer);
		} else if (hknpShape instanceof hknpCapsuleShape) {//BEFORE hknpConvexPolytopeShape
			return hkxCollisionToNifBullet.hknpCapsuleShape((hknpCapsuleShape)hknpShape, scale, nifVer);
		} else if (hknpShape instanceof hknpDynamicCompoundShape) {
			if (!isDynamic) {
				// this seems to make no odds
				System.out.println("createCollisionShape hknpDynamicCompoundShape! isDynamic=false in file " + nifVer.fileName);				
			}
			return hknpCompoundShape((hknpDynamicCompoundShape)hknpShape, contents, nifVer, isDynamic, scale);
		} else if (hknpShape instanceof hknpStaticCompoundShape) {
			if (isDynamic)
				System.out.println("createCollisionShape hknpStaticCompoundShape! isDynamic=" + isDynamic);
			return hknpCompoundShape((hknpStaticCompoundShape)hknpShape, contents, nifVer, isDynamic, scale);
		} else if (hknpShape instanceof hknpScaledConvexShape) {
			hknpScaledConvexShape((hknpScaledConvexShape)hknpShape, contents, nifVer, isDynamic, scale);
			return null;
		} else if (hknpShape instanceof hknpCompressedMeshShape) {
			hknpCompressedMeshShape hknpCompressedMeshShape = (hknpCompressedMeshShape)hknpShape;

			if (hknpCompressedMeshShape.data > 0) {
				hknpCompressedMeshShapeData hknpCompressedMeshShapeData = (hknpCompressedMeshShapeData)contents
						.get(hknpCompressedMeshShape.data);

				return hkxCollisionToNifBullet.hknpCompressedMeshShape(hknpCompressedMeshShapeData, isDynamic, scale,
						nifVer);
			} else {
				System.out.println("hknpCompressedMeshShape.data == -1");
				return null;
			}
		} else if (hknpShape instanceof hknpConvexPolytopeShape) {//BEFORE hknpConvexShape
			return hkxCollisionToNifBullet.hknpConvexPolytopeShape((hknpConvexPolytopeShape)hknpShape, scale, nifVer);
		} else if (hknpShape instanceof hknpConvexShape) {//AFTER hknpConvexPolytopeShape, hknpCapsuleShape, hknpSphereShape
			return hkxCollisionToNifBullet.hknpConvexShape((hknpConvexShape)hknpShape, scale, nifVer);
		} else if (hknpShape instanceof hknpCompoundShape) {
			return hknpCompoundShape((hknpCompoundShape)hknpShape, contents, nifVer, isDynamic, scale);
		} else {
			System.out.println("hkxShapeToCollisionShape.createCollisionShape - unknown bhkShape " + hknpShape);
			return null;
		}
	}

	private static CollisionShape hknpScaledConvexShape(hknpScaledConvexShape data, HKXContents contents, NifVer nifVer,
														boolean isDynamic, float scale) {
		CompoundShape cs = new CompoundShape();

		long shapeId = data.coreShape;
		if (shapeId > 0) {
			hknpShape hknpShape = (hknpShape)contents.get(shapeId);
			// there is no scaling in Bullet transforms so the scale is passed into the model create
			// along with the REFER scale value
			CollisionShape shape = createCollisionShape(hknpShape, contents, nifVer, isDynamic, data.scale.x * scale);
			if (shape != null) {
				cs.addChildShape(new Transform(), shape);
			} else {
				System.out.println("shape == null " + hknpShape + " " + nifVer.fileName);
				return null;
			}
		}

		return cs;
	}

	private static CollisionShape hknpCompoundShape(hknpCompoundShape data, HKXContents contents, NifVer nifVer,
													boolean isDynamic, float scale) {
		CompoundShape cs = new CompoundShape();
		for (int i = 0; i < data.instances.elements.length; i++) {
			hknpShapeInstance s = data.instances.elements[i];
			long shapeId = s.shape;
			if (shapeId > 0) {
				hknpShape hknpShape = (hknpShape)contents.get(shapeId);
				// there is no scaling in Bullet transforms so the scale is passed into the model create
				// along with the REFER scale value
				CollisionShape shape = createCollisionShape(hknpShape, contents, nifVer, isDynamic, s.scale.x* scale);
				if (shape != null) {
					Transform3D t3d = new Transform3D();

					Matrix4f m = ConvertFromHavok.toJ3dM4(s.transform, nifVer);
					t3d.set(m);					
					 
					Transform t = NifBulletUtil.createTrans(t3d);

					cs.addChildShape(t, shape);
				} else {
					System.out.println("shape == null " + hknpShape + " " + nifVer.fileName);
					return null;
				}
			}
		}

		return cs;
	}

}
