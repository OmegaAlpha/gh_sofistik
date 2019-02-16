﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace gh_sofistik
{
   public class GS_LineLoad : GH_GeometricGoo<Curve>, IGS_Load, IGH_PreviewData
   {
      public int LoadCase { get; set; } = 0;
      public bool UseHostLocal { get; set; } = false;
      public Vector3d Forces { get; set; } = new Vector3d();
      public Vector3d Moments { get; set; } = new Vector3d();
      
      public GS_StructuralLine ReferenceLine { get; set; }

      public override string TypeName
      {
         get { return "GS_LineLoad"; }
      }

      public override string TypeDescription
      {
         get { return "Line Load of load case " + LoadCase.ToString(); }
      }

      public override string ToString()
      {
         return "Line Load, LC = " + LoadCase.ToString();
      }

      public override bool CastTo<Q>(out Q target)
      {
         return Util.CastCurveTo(this.Value, out target);
      }

      public override IGH_GeometricGoo DuplicateGeometry()
      {
         return new GS_LineLoad()
         {
            Value = this.Value.DuplicateCurve(),
            LoadCase = this.LoadCase,
            Forces = this.Forces,
            Moments = this.Moments,
            UseHostLocal = this.UseHostLocal
         };
      }

      public override BoundingBox Boundingbox
      {
         get { return Value.GetBoundingBox(true); }
      }

      public BoundingBox ClippingBox
      {
         get {
            return DrawUtil.GetClippingBox(Value.GetBoundingBox(false), Forces.Length, Moments.Length);
         }
      }

      public override BoundingBox GetBoundingBox(Transform xform)
      {
         return xform.TransformBoundingBox(Value.GetBoundingBox(true));
      }

      public override IGH_GeometricGoo Morph(SpaceMorph xmorph)
      {
         var dup = this.DuplicateGeometry() as GS_PointLoad;
         xmorph.Morph(dup.Value);

         return dup;
      }

      public override IGH_GeometricGoo Transform(Transform xform)
      {
         var dup = this.DuplicateGeometry() as GS_PointLoad;
         dup.Value.Transform(xform);

         return dup;
      }

      public void DrawViewportWires(GH_PreviewWireArgs args)
      {
         //draw clippingbox
         //args.Pipeline.DrawBox(ClippingBox, System.Drawing.Color.Black);
         if (!(Value is null))
         {
            args.Pipeline.DrawCurve(Value, DrawUtil.DrawColForces);            
            if (!(Forces.Equals(Vector3d.Zero) && Moments.Equals(Vector3d.Zero)))
            {
               Vector3d lz = Vector3d.Negate(Vector3d.ZAxis);   //default local z
               if (!(ReferenceLine is null))
               {
                  if (!ReferenceLine.DirectionLocalZ.Equals(Vector3d.Zero))
                  {
                     lz = ReferenceLine.DirectionLocalZ;
                  }
               }
               DrawUtil.DrawCurve(args.Pipeline, Value, Forces, Moments, UseHostLocal, lz, null);
            }
         }
      }

      public void DrawViewportMeshes(GH_PreviewMeshArgs args)
      {
         //no meshes for arrows needed
      }
   }

   public class CreateLineLoad : GH_Component
   {
      public CreateLineLoad()
         : base("Line Load", "Line Load", "Creates SOFiSTiK Line Loads", "SOFiSTiK", "Loads")
      { }

      protected override System.Drawing.Bitmap Icon
      {
         get { return Properties.Resources.structural_line_load_16; } // TODO
      }

      protected override void RegisterInputParams(GH_InputParamManager pManager)
      {
         pManager.AddGeometryParameter("Hosting Curve / Sln", "Crv / Sln", "Hosting Curve / SOFiSTiK Structural Line", GH_ParamAccess.list);
         pManager.AddIntegerParameter("LoadCase", "LoadCase", "Id of Load Case", GH_ParamAccess.list, 1);
         pManager.AddVectorParameter("Force", "Force", "Acting Force", GH_ParamAccess.list, new Vector3d());
         pManager.AddVectorParameter("Moment", "Moment", "Acting Moment", GH_ParamAccess.list, new Vector3d());
         pManager.AddBooleanParameter("HostLocal", "HostLocal", "Use local coordinate system of host", GH_ParamAccess.list, false);
      }

      protected override void RegisterOutputParams(GH_OutputParamManager pManager)
      {
         pManager.AddGeometryParameter("Line Load", "LLd", "SOFiSTiK Line Load", GH_ParamAccess.list);
      }

      protected override void SolveInstance(IGH_DataAccess da)
      {
         var curves = da.GetDataList<IGH_GeometricGoo>(0);
         var loadcases = da.GetDataList<int>(1);
         var forces = da.GetDataList<Vector3d>(2);
         var moments = da.GetDataList<Vector3d>(3);
         var hostlocals = da.GetDataList<bool>(4);

         var gs_line_loads = new List<GS_LineLoad>();

         for (int i = 0; i < curves.Count; ++i)
         {
            var curve = curves.GetItemOrLast(i);

            var ll = new GS_LineLoad()
            {
               LoadCase = loadcases.GetItemOrLast(i),
               Forces = forces.GetItemOrLast(i),
               Moments = moments.GetItemOrLast(i),
               UseHostLocal = hostlocals.GetItemOrLast(i)
            };

            if (curve is GS_StructuralLine)
            {
               var sln = curve as GS_StructuralLine;

               ll.Value = sln.Value;
               ll.ReferenceLine = sln;  // pass reference of structural line
            }
            else if (curve is GH_Curve)
            {
               ll.Value = (curve as GH_Curve).Value;
            }
            else if (curve is GH_Line)
            {
               ll.Value = new LineCurve((curve as GH_Line).Value);
            }
            else if (curve is GH_Arc)
            {
               ll.Value = new ArcCurve((curve as GH_Arc).Value);
            }
            else
            {
               throw new Exception("Unable to Cast input to Curve Geometry");
            }

            gs_line_loads.Add(ll);
         }

         da.SetDataList(0, gs_line_loads);
      }

      public override Guid ComponentGuid
      {
         get { return new Guid("992ADED6-7395-4166-8DA7-7C78AE554615"); }
      }
   }
}
