package nifbullet.convert;

import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;

import org.jogamp.java3d.Transform3D;
import org.jogamp.vecmath.Matrix4f;
import org.jogamp.vecmath.Vector3d;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.linearmath.Transform;

import nif.NifVer;
import nif.niobject.hkx.hknpCapsuleShape;
import nif.niobject.hkx.hknpCompoundShape;
import nif.niobject.hkx.hknpCompressedMeshShape;
import nif.niobject.hkx.hknpCompressedMeshShapeData;
import nif.niobject.hkx.hknpConvexPolytopeShape;
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

	private static CollisionShape createCollisionShape(hknpShape hknpShape, HKXContents contents, boolean isDynamic,
														float scale) {
		
		if (hknpShape instanceof hknpSphereShape) {
			return hkxCollisionToNifBullet.hknpSphereShape((hknpSphereShape)hknpShape, scale, nifVer);
		} else if (hknpShape instanceof hknpCapsuleShape) {
			return hkxCollisionToNifBullet.hknpCapsuleShape((hknpCapsuleShape)hknpShape, scale, nifVer);
		} else	if (hknpShape instanceof hknpDynamicCompoundShape) {
			if(!isDynamic)
				System.out.println("createCollisionShape hknpDynamicCompoundShape! isDynamic=" + isDynamic);
			hknpCompoundShape((hknpDynamicCompoundShape)hknpShape, contents, isDynamic, scale);
			return null;
		} else	if (hknpShape instanceof hknpStaticCompoundShape) {			
			if(isDynamic)
				System.out.println("createCollisionShape hknpStaticCompoundShape! isDynamic=" + isDynamic);
			hknpCompoundShape((hknpStaticCompoundShape)hknpShape, contents, isDynamic, scale);
			return null;
		} else if (hknpShape instanceof hknpScaledConvexShape) {
			hknpScaledConvexShape((hknpScaledConvexShape) hknpShape, contents, isDynamic, scale);
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
		} else if (hknpShape instanceof hknpConvexPolytopeShape) {
			return hkxCollisionToNifBullet.hknpConvexPolytopeShape((hknpConvexPolytopeShape)hknpShape, scale, nifVer);
		}  else {
			System.out.println("NifHavokToj3d - unknown bhkShape " + hknpShape);
			return null;
		}
	}
	
	private static CollisionShape hknpScaledConvexShape(hknpScaledConvexShape data, HKXContents contents, boolean isDynamic, float scale)
	{
		CompoundShape cs = new CompoundShape();
 
		long shapeId = data.coreShape;
		if(shapeId > 0) {
			hknpShape hknpShape = (hknpShape)contents.get(shapeId);
			CollisionShape shape = createCollisionShape(hknpShape, contents, isDynamic, scale);
			if (shape != null) {
				Transform3D t3d = new Transform3D();

				t3d.set(ConvertFromHavok.toJ3d(data.position, nifVer));
				
				//Note no ConvertFromHavok as these are just straight multipliers 
				t3d.setScale(new Vector3d(data.scale.x, data.scale.z, data.scale.y));
 
				Transform t = NifBulletUtil.createTrans(t3d);
				
				cs.addChildShape(t, shape);
			} else {
				System.out.println("shape == null " + hknpShape);
				return null;
			}
		}
		
		return cs;
	}
	
	private static CollisionShape hknpCompoundShape(hknpCompoundShape data, HKXContents contents, boolean isDynamic, float scale)
	{
		CompoundShape cs = new CompoundShape();
		for(int i = 0 ; i < data.instances.elements.length;i++) {
			hknpShapeInstance s = data.instances.elements[i];
			long shapeId = s.shape;
			if(shapeId > 0) {
				hknpShape hknpShape = (hknpShape)contents.get(shapeId);
				CollisionShape shape = createCollisionShape(hknpShape, contents, isDynamic, scale);
				if (shape != null) {
 					Transform3D t3d = new Transform3D();
	
					Matrix4f m = ConvertFromHavok.toJ3dM4(s.transform, nifVer);
					t3d.set(m);
					
					//Note no ConvertFromHavok as these are just straight multipliers 
					t3d.setScale(new Vector3d(s.scale.x, s.scale.z, s.scale.y));
	 
					Transform t = NifBulletUtil.createTrans(t3d);
					
					cs.addChildShape(t, shape);
				} else {
					System.out.println("shape == null " + hknpShape);
					return null;
				}
			}			
		}
		
		return cs;
	}

}
