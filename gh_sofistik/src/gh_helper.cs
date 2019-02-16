using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;

namespace gh_sofistik
{
   // extends linq queries
   static class LinqExtension
   {
      public static T GetItemOrLast<T>(this IList<T> list, int index)
      {
         if (list.Count == 0)
            return default(T);
         else if (index >= list.Count)
            return list[list.Count - 1];
         else
            return list[index];
      }

      public static int GetItemOrCountUp(this IList<int> list, int index)
      {
         if (index >= list.Count)
         {
            if (list[list.Count - 1] > 0)
               return list[list.Count - 1] + (index - list.Count + 1);
         }
         else if (list.Count > 0)
         {
            return list[index];
         }
         return 0;
      }
   }

   // extends the IGH interface
   static class IGH_Extension
   {
      // extends the GetData method
      public static T GetData<T>(this IGH_DataAccess da, int index)
      {
         var item = default(T);

         da.GetData(index, ref item);

         return item;
      }

      // extension reads data and returns type
      public static List<T> GetDataList<T>(this IGH_DataAccess da, int index)
      {
         var list = new List<T>();

         da.GetDataList(index, list);

         return list;
      }

      // extension reads data and returns type
      public static GH_Structure<T> GetDataTree<T>(this IGH_DataAccess da, int index) where T : IGH_Goo
      {
         var geometry = new GH_Structure<T>();

         da.GetDataTree(index, out geometry);

         return geometry;
      }
   }

   static class Util
   {
      public static bool CastCurveTo<Q>(Rhino.Geometry.Curve curve, out Q target)
      {
         if(curve != null)
         {
            // cast to GH_Curve (Caution: this loses all structural information)
            if (typeof(Q).IsAssignableFrom(typeof(GH_Curve)))
            {
               var gc = new GH_Curve(curve);
               target = (Q)(object)gc;
               return true;
            }
            // cast to GH_Line (Caution: this loses all structural information)
            else if (typeof(Q).IsAssignableFrom(typeof(GH_Line)))
            {
               if (curve is Rhino.Geometry.LineCurve)
               {
                  var gl = new GH_Line((curve as Rhino.Geometry.LineCurve).Line);
                  target = (Q)(object)gl;
                  return true;
               }
            }
            // cast to GH_Arc (Caution: this loses all structural information)
            else if (typeof(Q).IsAssignableFrom(typeof(GH_Arc)))
            {
               if (curve is Rhino.Geometry.ArcCurve)
               {
                  var ga = new GH_Arc((curve as Rhino.Geometry.ArcCurve).Arc);
                  target = (Q)(object)ga;
                  return true;
               }
            }
            else
            {
               throw new Exception("Unable to cast to type: " + typeof(Q).ToString());
            }
         }

         target = default(Q);
         return false;
      }

      public static bool CastBrepTo<Q>(Rhino.Geometry.Brep brep, out Q target)
      {
         if(brep != null)
         {
            if(typeof(Q).IsAssignableFrom(typeof(GH_Brep)))
            {
               var gb = new GH_Brep(brep);
               target = (Q)(object)gb;
               return true;
            }
         }

         target = default(Q);
         return false;
      }
   }

   static class DrawUtil
   {
      private static double scaleFactor = 1.0;

      public static double ScaleFactor
      {
         get
         {
            return scaleFactor;
         }
         set
         {
            scaleFactor = value < 0 ? 0 : value;
         }
      }      

      public static double DensityFactor { get; set; } = 1.0;
      
      public static System.Drawing.Color DrawColStrc { get; set; } = System.Drawing.Color.Red;

      public static System.Drawing.Color DrawColForces { get; set; } = System.Drawing.Color.IndianRed;
      

      private static Transform GetGlobalTransformArea(Point3d areaPoint, BrepFace bf, Vector3d refLX)
      {
         double u;
         double v;
         bf.ClosestPoint(areaPoint, out u, out v);
         Vector3d srfcNormal = bf.NormalAt(u, v);
         Transform tf;         
         if (refLX.Equals(Vector3d.Zero) || !(srfcNormal.IsParallelTo(refLX, 0.0001)==0))   // if DirectionLocalX is zero, or surfaceNormal is (anti)parallel to DirectionLocalX, use standard surface u,v coordinates
         {  
            Plane oPlane = new Plane();
            bf.FrameAt(u, v, out oPlane);
            tf = Rhino.Geometry.Transform.ChangeBasis(Vector3d.Negate(oPlane.XAxis), Vector3d.Negate(oPlane.YAxis), oPlane.ZAxis, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis);            
         } else   // construct transform with DirectionLocalX from ReferencePoint
         {
            Vector3d lx = new Vector3d(refLX);
            Vector3d lz = srfcNormal;
            lx.Unitize();
            lz.Unitize();
            Vector3d ly = Vector3d.CrossProduct(lz, lx);
            ly.Unitize();
            lx = Vector3d.CrossProduct(ly, lz);
            lx.Unitize();            
            tf = Rhino.Geometry.Transform.ChangeBasis(lx, ly, lz, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis);
         }         
         return tf;
      }

      public static Transform GetGlobalTransformLine(Vector3d tangent, Vector3d refLZ)
      {
         //setup changeBasis transform from local tangent space to global world space
         Vector3d lx = new Vector3d(tangent);
         Vector3d lz = new Vector3d(refLZ);
         if (!(lx.IsParallelTo(lz, 0.0001) == 0))   //tangent (anti)parallel to DirectionLocalZ, use global Z
         {
            lz = new Vector3d(Vector3d.Negate(Vector3d.ZAxis));
            if (!(lx.IsParallelTo(lz, 0.0001) == 0))   //tangent (anti)parallel to gloabl Z (==DirectionLocalZ), use global X
            {
               lz = new Vector3d(Vector3d.XAxis);
            }
         }
         lx.Unitize();
         lz.Unitize();
         Vector3d ly = Vector3d.CrossProduct(lz, lx);
         ly.Unitize();
         lz = Vector3d.CrossProduct(lx, ly);
         lz.Unitize();         
         return Rhino.Geometry.Transform.ChangeBasis(lx, ly, lz, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis);
      }

      public static Transform GetGlobalTransformPoint(Vector3d refLX, Vector3d refLZ)
      {
         // similar procedure as above
         // lz master. if both local directions are given from referencePoint, then DirectionLocalX gets adjusted to form an orthogonal right handed coordinate frame
         Vector3d lx = new Vector3d(refLX);
         Vector3d lz = new Vector3d(refLZ);
         if (!(lx.IsParallelTo(lz, 0.0001) == 0))
         {            
            lx = new Vector3d(Vector3d.Negate(Vector3d.ZAxis));           
            if (!(lx.IsParallelTo(lz, 0.0001) == 0)) lx = new Vector3d(Vector3d.XAxis);
         }         
         lx.Unitize();
         lz.Unitize();
         Vector3d ly = Vector3d.CrossProduct(lz, lx);
         ly.Unitize();
         lx = Vector3d.CrossProduct(ly, lz);
         lx.Unitize();
         return Rhino.Geometry.Transform.ChangeBasis(lx, ly, lz, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis);
      }

      public static BoundingBox GetClippingBox(BoundingBox bBox, double forceLength, double momentLength)
      {  
         bBox.Inflate(Math.Max(forceLength, momentLength*0.05+momentLength/2) * ScaleFactor);
         return bBox;
      }

      public static void ParseFixString(string s, bool [,,] bits)
      {
         String lowS=s.Trim().ToLower();
         lowS = lowS.Replace("f", "pxpypzmxmymz").Replace("pp", "pxpypz").Replace("mm", "mxmymz");
         lowS = ".." +lowS;
         char[]ca=lowS.ToCharArray();
         for (int i = 1; i < ca.Length; i++)
         {
            if (ca[i].Equals('x')) checkPrevChar(bits, ca[i - 1], ca[i - 2], 0);
            if (ca[i].Equals('y')) checkPrevChar(bits, ca[i - 1], ca[i - 2], 1);
            if (ca[i].Equals('z')) checkPrevChar(bits, ca[i - 1], ca[i - 2], 2);
         }         
      }

      private static void checkPrevChar(bool [,,] bits, char pc, char pc2, int d)
      {
         if (pc.Equals('p'))
         {
            if (pc2.Equals('l')) bits[0, d, 1] = true;
            else bits[0, d, 0] = true;
         }

         else if (pc.Equals('m'))
         {
            if (pc2.Equals('l')) bits[1, d, 1] = true;
            else bits[1, d, 0] = true;
         }
      }
            
      public static List<Vector3d> getFixedAxis(Transform tLocalGlobal, bool[,,] bits, int flagPosAxis)
      {
         if (flagPosAxis < 0) flagPosAxis = 0;
         if (flagPosAxis > 1) flagPosAxis = 1;
         List<Vector3d> fixedList = new List<Vector3d>();
         Vector3d[] va = new Vector3d[3];
         va[0] = Vector3d.XAxis;
         va[1] = Vector3d.YAxis;
         va[2] = Vector3d.ZAxis;
         for (int i = 0; i < 3; i++)
         {
            if (bits[flagPosAxis, i, 0] || bits[flagPosAxis, i, 1])
            {
               Vector3d toAdd = new Vector3d(va[i]);
               if (bits[flagPosAxis, i, 1])
                  toAdd.Transform(tLocalGlobal);

               bool alreadyInside = false;
               foreach (Vector3d v in fixedList)
               {
                  if (!(v.IsParallelTo(toAdd, 0.0001) == 0))
                  {
                     alreadyInside = true;
                     break;
                  }
               }
               if (!alreadyInside)
                  fixedList.Add(toAdd);
            }
         }
         return fixedList;
      }

      private static Transform getSupportOrientation(Transform tLocalGlobal, bool[,,] bits, List<Vector3d> fixedList, int flagPosAxis)
      {
         if (flagPosAxis < 0) flagPosAxis = 0;
         if (flagPosAxis > 1) flagPosAxis = 1;

         Transform tDrawGlobal = Rhino.Geometry.Transform.Identity;

         if (fixedList.Count > 0)
         {
            Transform tDrawLocal = Rhino.Geometry.Transform.Identity;

            if (fixedList.Count == 1 || fixedList.Count == 3)
            {

               if (!(bits[flagPosAxis, 2, 0] || bits[flagPosAxis, 2, 1]))
               {
                  tDrawLocal = Rhino.Geometry.Transform.ChangeBasis(Vector3d.ZAxis, Vector3d.XAxis, Vector3d.YAxis, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis);
                  if (!(bits[flagPosAxis, 1, 0] || bits[flagPosAxis, 1, 1]))
                  {
                     tDrawLocal = Rhino.Geometry.Transform.ChangeBasis(Vector3d.YAxis, Vector3d.ZAxis, Vector3d.XAxis, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis);
                     if (bits[flagPosAxis, 0, 1]) tDrawGlobal = tLocalGlobal * tDrawLocal;
                     else tDrawGlobal = tDrawLocal;
                  }
                  else
                  {
                     if (bits[flagPosAxis, 1, 1]) tDrawGlobal = tLocalGlobal * tDrawLocal;
                     else tDrawGlobal = tDrawLocal;
                  }
               }
               else if (bits[flagPosAxis, 2, 1])
               {
                  tDrawGlobal = tLocalGlobal;
               }
            }

            if (fixedList.Count == 2)
            {
               Vector3d freeAxis = Vector3d.CrossProduct(fixedList[0], fixedList[1]);
               tLocalGlobal = DrawUtil.GetGlobalTransformPoint(fixedList[0], freeAxis);
               tDrawLocal = Rhino.Geometry.Transform.ChangeBasis(Vector3d.ZAxis, Vector3d.XAxis, Vector3d.YAxis, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis);
               tDrawGlobal = tLocalGlobal * tDrawLocal;
            }
         }
         return tDrawGlobal;
      }

      public static void DrawCurve(Rhino.Display.DisplayPipeline pipeline, Curve crv, Vector3d frc, Vector3d mmnt, bool uhl, Vector3d localDir, BrepFace bf)
      {
         List<Point3d> fList = new List<Point3d>(); // lists for drawing dotted lines which connect force,moment arrows
         List<Point3d> mList = new List<Point3d>();
         double lineInc = Math.Round(crv.GetLength() * DensityFactor);
         lineInc = lineInc > 1 ? lineInc : 1;
         lineInc = 1 / lineInc;
         Vector3d tempForce = frc;
         Vector3d tempMoment = mmnt;
         //iterate over curve segments, determined by lineInc
         for (double i = 0; Math.Round(i, 4) <= 1; i += lineInc)
         {
            if (uhl) //transform needed
            {
               Transform tf;
               if (bf is null) tf = GetGlobalTransformLine(crv.TangentAt(i * crv.GetLength()), localDir); //setup transform for single curve, no brepFace
               else tf = GetGlobalTransformArea(crv.PointAtNormalizedLength(i), bf, localDir); //setup transform for edge of brepFace
               //transform if necessary
               if (!frc.Equals(Vector3d.Zero))
               {
                  tempForce = new Vector3d(frc);
                  tempForce.Transform(tf);
               }
               if (!mmnt.Equals(Vector3d.Zero))
               {
                  tempMoment = new Vector3d(mmnt);
                  tempMoment.Transform(tf);
               }
            }
            //draw and add endPoints to list for dottetLines
            if (!frc.Equals(Vector3d.Zero)) fList.Add(DrawUtil.DrawForceArrow(pipeline, crv.PointAtNormalizedLength(i), tempForce));
            if (!mmnt.Equals(Vector3d.Zero)) mList.Add(DrawUtil.DrawMomentArrow(pipeline, crv.PointAtNormalizedLength(i), tempMoment));
         }

         //draw dotted Lines
         IEnumerator<Point3d> fi = fList.GetEnumerator();
         fi.MoveNext();
         Point3d prevF = fi.Current;
         while (fi.MoveNext())
         {
            pipeline.DrawDottedLine(new Line(prevF, fi.Current), DrawUtil.DrawColForces);
            prevF = fi.Current;
         }
         IEnumerator<Point3d> mi = mList.GetEnumerator();
         mi.MoveNext();
         Point3d prevM = mi.Current;
         while (mi.MoveNext())
         {
            pipeline.DrawDottedLine(new Line(prevM, mi.Current), DrawUtil.DrawColForces);
            prevM = mi.Current;
         }
      }

      public static Point3d DrawForceArrow(Rhino.Display.DisplayPipeline pipeline, Point3d location, Vector3d direction)
      {
         // setup line from 'location' to '(location+direction*scaleFactor)' and draw arrow
         Point3d res = new Point3d(location.X - ScaleFactor * direction.X, location.Y - ScaleFactor * direction.Y, location.Z - ScaleFactor * direction.Z);
         Line arrowLine = new Line(res, location);
         pipeline.DrawArrow(arrowLine, DrawColForces, 0.0, 0.2);
         return res;
      }

      public static Point3d DrawMomentArrow(Rhino.Display.DisplayPipeline pipeline, Point3d location, Vector3d direction)
      {
         // setup arc at 'location' with normal 'direction' and radius direction.length*scaleFactor/2
         // draw arc and arrow at end of arc
         Vector3d invDir = new Vector3d(Vector3d.Negate(direction));
         Arc mArc = new Arc(new Plane(location, invDir), ScaleFactor * direction.Length / 2, 2 * Math.PI * 0.75);
         Point3d arrowPoint = mArc.PointAt(0);
         Vector3d arrowDir = mArc.TangentAt(0);
         arrowDir.Reverse();
         pipeline.DrawArc(mArc, DrawColForces);
         pipeline.DrawArrowHead(arrowPoint, arrowDir, DrawColForces, 0.0, ScaleFactor * direction.Length * 0.2);
         return arrowPoint;
      }

      public static void DrawSupport(Rhino.Display.DisplayPipeline pipeline, Point3d location, Transform tLocalGlobal, bool[,,] bits, bool shaded)
      {
         List<Vector3d> fixedP = getFixedAxis(tLocalGlobal, bits, 0);
         List<Vector3d> fixedM = getFixedAxis(tLocalGlobal, bits, 1);

         if (fixedM.Count == 3 && fixedP.Count > 0)
         {
            Transform tP = getSupportOrientation(tLocalGlobal, bits, fixedP, 0);
            if (fixedP.Count == 3)
               drawCube(pipeline, location, tP, shaded, true);
            else
            {
               drawPyramid(pipeline, location, tP, shaded);
               drawFreeDegreeLine(pipeline, location, tP, 3 - fixedP.Count);
               drawCube(pipeline, location, tP, shaded, false);
            }
         }
         else
         {
            if (fixedP.Count > 0)
            {
               Transform tP = getSupportOrientation(tLocalGlobal, bits, fixedP, 0);
               drawPyramid(pipeline, location, tP, shaded);
               drawFreeDegreeLine(pipeline, location, tP, 3 - fixedP.Count);
            }
            if (fixedM.Count > 0)
            {
               Transform tM = getSupportOrientation(tLocalGlobal, bits, fixedM, 1);
               if (fixedM.Count == 1)
                  drawFork(pipeline, location, tM);
               if (fixedM.Count == 2)
                  drawCylinder(pipeline, location, tM, shaded);
               if (fixedM.Count == 3)
               {
                  drawFork(pipeline, location, tM);
                  drawCylinder(pipeline, location, tM * Rhino.Geometry.Transform.RotationZYX(0, Math.PI / 2, 0), shaded);
               }
            }
         }
      }

      private static void drawCylinder(Rhino.Display.DisplayPipeline pipeline, Point3d location, Transform tf, bool shaded)
      {
         double preScale = 0.1;
         Circle ci = new Circle(new Plane(new Point3d(-1 * DrawUtil.ScaleFactor * preScale, 0, 0), Vector3d.XAxis), 0.318 * preScale * DrawUtil.ScaleFactor);
         ci.Transform(tf);
         ci.Translate(new Vector3d(location));
         Cylinder c = new Cylinder(ci, 2 * preScale * DrawUtil.ScaleFactor);


         if (shaded)
         {
            Rhino.Display.DisplayMaterial disMat = new Rhino.Display.DisplayMaterial();
            disMat.Diffuse = DrawUtil.DrawColStrc;
            disMat.Specular = DrawUtil.DrawColStrc;
            disMat.Emission = DrawUtil.DrawColStrc;
            pipeline.DrawBrepShaded(c.ToBrep(true, true), disMat);
         }
         else
         {
            pipeline.DrawCylinder(c, System.Drawing.Color.Black);
         }
      }

      private static void drawFork(Rhino.Display.DisplayPipeline pipeline, Point3d location, Transform tf)
      {
         double preScale = 0.1;
         Point3d[] pts = new Point3d[4];
         pts[0] = new Point3d(-0.5, -1, 0);
         pts[1] = new Point3d(-0.5, 1, 0);
         pts[2] = new Point3d(0.5, 1, 0);
         pts[3] = new Point3d(0.5, -1, 0);
         for (int i = 0; i < 4; i++)
         {
            pts[i] *= DrawUtil.ScaleFactor * preScale;
            pts[i].Transform(tf);
            pts[i] += location;
         }

         pipeline.DrawLine(new Line(pts[0], pts[1]), System.Drawing.Color.Black);
         pipeline.DrawLine(new Line(pts[2], pts[3]), System.Drawing.Color.Black);

      }

      private static void drawPyramid(Rhino.Display.DisplayPipeline pipeline, Point3d location, Transform tf, bool shaded)
      {
         double preScale = 0.2;
         Point3d[] pyramidPoints = new Point3d[4];
         pyramidPoints[0] = new Point3d(-0.5, -0.5, -1);
         pyramidPoints[1] = new Point3d(-0.5, 0.5, -1);
         pyramidPoints[2] = new Point3d(0.5, 0.5, -1);
         pyramidPoints[3] = new Point3d(0.5, -0.5, -1);
         for (int i = 0; i < 4; i++)
         {
            pyramidPoints[i] *= DrawUtil.ScaleFactor * preScale;
            pyramidPoints[i].Transform(tf);
            pyramidPoints[i] += location;
         }

         if (shaded)
         {
            Mesh mesh = new Mesh();
            mesh.Vertices.Add(location.X, location.Y, location.Z);
            for (int i = 0; i < 4; i++)
               mesh.Vertices.Add(pyramidPoints[i].X, pyramidPoints[i].Y, pyramidPoints[i].Z);
            mesh.Faces.AddFace(1, 4, 0);
            mesh.Faces.AddFace(2, 1, 0);
            mesh.Faces.AddFace(3, 2, 0);
            mesh.Faces.AddFace(4, 3, 0);
            mesh.Faces.AddFace(1, 2, 3, 4);
            mesh.Normals.ComputeNormals();
            mesh.Compact();

            Rhino.Display.DisplayMaterial disMat = new Rhino.Display.DisplayMaterial();
            disMat.Diffuse = DrawUtil.DrawColStrc;
            disMat.Specular = DrawUtil.DrawColStrc;
            disMat.Emission = DrawUtil.DrawColStrc;

            pipeline.DrawMeshShaded(mesh, disMat);
         }
         else
         {
            Line[] pyramidLines = new Line[8];
            for (int i = 0; i < 4; i++) pyramidLines[i] = new Line(location, pyramidPoints[i]);
            for (int i = 0; i < 4; i++) pyramidLines[4 + i] = new Line(pyramidPoints[i], pyramidPoints[(i + 1) % 4]);
            for (int i = 0; i < 8; i++) pipeline.DrawLine(pyramidLines[i], System.Drawing.Color.Black);
         }
      }

      private static void drawFreeDegreeLine(Rhino.Display.DisplayPipeline pipeline, Point3d location, Transform tf, int amount)
      {
         double preScale = 0.2;
         Point3d a = new Point3d(0, 0, -1.2);

         for (int j = 0; j < amount; j++)
         {
            Vector3d dirFree = Vector3d.Zero;
            if (j == 0)
               dirFree = new Vector3d(1, 0, 0);
            else
               dirFree = new Vector3d(0, 1, 0);
            Vector3d cdir = Vector3d.CrossProduct(Vector3d.ZAxis, dirFree);
            cdir.Unitize();

            Point3d b1 = a + cdir * 0.5;
            Point3d b2 = a - cdir * 0.5;

            Point3d[] pl = new Point3d[4];
            pl[0] = b1 + dirFree * 0.4;
            pl[1] = b1 - dirFree * 0.4;
            pl[2] = b2 + dirFree * 0.4;
            pl[3] = b2 - dirFree * 0.4;

            for (int i = 0; i < 4; i++)
            {
               pl[i] *= DrawUtil.ScaleFactor * preScale;
               pl[i].Transform(tf);
               pl[i] += location;
            }

            Line l1 = new Line(pl[0], pl[1]);
            Line l2 = new Line(pl[2], pl[3]);

            pipeline.DrawLine(l1, System.Drawing.Color.Black);
            pipeline.DrawLine(l2, System.Drawing.Color.Black);
         }
      }

      private static void drawCube(Rhino.Display.DisplayPipeline pipeline, Point3d location, Transform tf, bool shaded, bool big)
      {
         double preScale = 0.1;
         Point3d[] cubePoints = new Point3d[8];
         cubePoints[0] = new Point3d(-1, -1, -1);
         cubePoints[1] = new Point3d(1, -1, -1);
         cubePoints[2] = new Point3d(1, 1, -1);
         cubePoints[3] = new Point3d(-1, 1, -1);
         cubePoints[4] = new Point3d(-1, -1, 1);
         cubePoints[5] = new Point3d(1, -1, 1);
         cubePoints[6] = new Point3d(1, 1, 1);
         cubePoints[7] = new Point3d(-1, 1, 1);
         for (int i = 0; i < 8; i++)
         {
            if (big)
               cubePoints[i] += new Vector3d(0, 0, -1);
            else
               cubePoints[i] *= 0.5;
            cubePoints[i] *= DrawUtil.ScaleFactor * preScale;
            cubePoints[i].Transform(tf);
            cubePoints[i] += location;
         }

         if (shaded)
         {
            Mesh mesh = new Mesh();
            for (int i = 0; i < 8; i++)
               mesh.Vertices.Add(cubePoints[i].X, cubePoints[i].Y, cubePoints[i].Z);
            mesh.Faces.AddFace(0, 1, 5, 4);
            mesh.Faces.AddFace(3, 0, 4, 7);
            mesh.Faces.AddFace(1, 2, 6, 5);
            mesh.Faces.AddFace(0, 5, 6, 7);
            mesh.Faces.AddFace(0, 3, 2, 1);
            mesh.Faces.AddFace(7, 6, 2, 3);
            mesh.Normals.ComputeNormals();
            mesh.Compact();

            Rhino.Display.DisplayMaterial disMat = new Rhino.Display.DisplayMaterial();
            disMat.Diffuse = DrawUtil.DrawColStrc;
            disMat.Specular = DrawUtil.DrawColStrc;
            disMat.Emission = DrawUtil.DrawColStrc;

            pipeline.DrawMeshShaded(mesh, disMat);
         }
         else
         {
            Line[] cubeLines = new Line[12];
            for (int i = 0; i < 4; i++) cubeLines[i] = new Line(cubePoints[i], cubePoints[(i + 1) % 4]);
            for (int i = 0; i < 4; i++) cubeLines[4 + i] = new Line(cubePoints[4 + i], cubePoints[4 + ((i + 1) % 4)]);
            for (int i = 0; i < 4; i++) cubeLines[8 + i] = new Line(cubePoints[i], cubePoints[4 + i]);
            for (int i = 0; i < 12; i++) pipeline.DrawLine(cubeLines[i], System.Drawing.Color.Black);
         }
      }
   }
}
