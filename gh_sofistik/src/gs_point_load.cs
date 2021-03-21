﻿using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using gh_sofistik.Structure;

namespace gh_sofistik.Load
{
   public interface IGS_Load
   {
      int LoadCase { get; }
      string TypeName { get; }
   }

   public class GS_PointLoad : GH_GeometricGoo<Point>, IGS_Load, IGH_PreviewData
   {
      public int LoadCase { get; set; } = 0;
      public bool UseHostLocal { get; set; } = false;
      public Vector3d Forces { get; set; } = new Vector3d();
      public Vector3d Moments { get; set; } = new Vector3d();
      public Vector3d Displacement { get; set; } = new Vector3d();
      public Vector3d DisplacementRotational { get; set; } = new Vector3d();

      private LoadCondition _loadCondition = new LoadCondition();
      private InfoPanel _infoPanel;

      public GS_StructuralPoint ReferencePoint { get; set; }

      public override string TypeName
      {
         get { return "GS_PointLoad"; }
      }

      public override string TypeDescription
      {
         get { return "Point Load of load case " + LoadCase.ToString(); }
      }

      public override string ToString()
      {
         return "Point Load, LC = " + LoadCase.ToString();
      }

      public override bool CastTo<Q>(out Q target)
      {
         if (Value != null)
         {
            if (typeof(Q).IsAssignableFrom(typeof(GH_Point)))
            {
               var gp = new GH_Point(this.Value.Location);
               target = (Q)(object)gp;
               return true;
            }
         }

         target = default(Q);
         return false;
      }

      public override IGH_GeometricGoo DuplicateGeometry()
      {
         return new GS_PointLoad()
         {
            Value = new Point(this.Value.Location),
            LoadCase = this.LoadCase,
            Forces = this.Forces,
            Moments = this.Moments,
            Displacement = this.Displacement,
            DisplacementRotational = this.DisplacementRotational,
            UseHostLocal = this.UseHostLocal
         };
      }

      public override BoundingBox Boundingbox
      {
         get { return Value.GetBoundingBox(true); }
      }

      public BoundingBox ClippingBox
      {
         get
         {
            return DrawUtil.GetClippingBoxLoads(Value.GetBoundingBox(false), Forces.Length, Moments.Length);
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
         //ClippingBox
         //args.Pipeline.DrawBox(ClippingBox, System.Drawing.Color.Black);
         if (!(Value is null))   //if no point or, force AND moment are zero, nothing to draw
         {  
            System.Drawing.Color col = args.Color;
            if (!DrawUtil.CheckSelection(col))
            {
               col = DrawUtil.DrawColorLoads;
            }
            else
            {
               drawInfoPanel(args.Pipeline, args.Viewport);
            }

            args.Pipeline.DrawPoint(Value.Location, Rhino.Display.PointStyle.X, 5, DrawUtil.DrawColorLoads);

            if (!(Forces.IsTiny() && Moments.IsTiny() && Displacement.IsTiny() && DisplacementRotational.IsTiny()) && DrawUtil.ScaleFactorLoads > 0.0001) {
               if (!_loadCondition.isValid)
               {
                  updateLoadTransforms();
               }
               _loadCondition.Draw(args.Pipeline, col);
            }
         }
      }

      private void drawInfoPanel(Rhino.Display.DisplayPipeline pipeline, Rhino.Display.RhinoViewport viewport)
      {
         if (DrawUtil.DrawInfo)
         {
            if (_infoPanel == null)
            {
               _infoPanel = new InfoPanel();
               _infoPanel.Positions.Add(Value.Location);

               _infoPanel.Content.Add("LC: " + LoadCase);
               if (!Forces.IsTiny())
                  _infoPanel.Content.Add("Force: " + Forces.Length);
               if (!Moments.IsTiny())
                  _infoPanel.Content.Add("Moment: " + Moments.Length);
               if (!Displacement.IsTiny())
                  _infoPanel.Content.Add("Displacement: " + Displacement.Length);
               if (!DisplacementRotational.IsTiny())
                  _infoPanel.Content.Add("Rot.Displacement: " + DisplacementRotational.Length);
            }
            _infoPanel.Draw(pipeline, viewport);
         }
      }

      private void updateLoadTransforms()
      {
         _loadCondition = new LoadCondition(Forces, Moments, Displacement, DisplacementRotational);

         Transform t = Rhino.Geometry.Transform.Identity;
         if (UseHostLocal) //transformation needed
         {
            Vector3d lx = Vector3d.XAxis;   //default x,z directions
            Vector3d lz = Vector3d.Negate(Vector3d.ZAxis);
            //set localX and localZ directions if pointload has referencepoint and if directions are not zero
            if (!(ReferencePoint is null))
            {
               if (!ReferencePoint.DirectionLocalX.IsTiny()) lx = ReferencePoint.DirectionLocalX;
               if (!ReferencePoint.DirectionLocalZ.IsTiny()) lz = ReferencePoint.DirectionLocalZ;
            }
            t = TransformUtils.GetGlobalTransformPoint(lx, lz);   //setup transform
         }

         t = Rhino.Geometry.Transform.Translation(new Vector3d(Value.Location)) * t * Rhino.Geometry.Transform.Scale(Point3d.Origin, DrawUtil.ScaleFactorLoads);

         _loadCondition.Transforms.Add(t);
      }
   

      public void DrawViewportMeshes(GH_PreviewMeshArgs args)
      {
         //no meshes for arrow needed
      }

   }

   public class CreatePointLoad : GH_Component
   {
      private System.Drawing.Bitmap _icon;

      public CreatePointLoad()
         : base("Point Load","Point Load","Creates SOFiSTiK Point Loads", "SOFiSTiK", "Loads")
      { }

      protected override System.Drawing.Bitmap Icon
      {
         get
         {
            if (_icon == null)
               _icon = Util.GetBitmap(GetType().Assembly, "structural_point_load_24x24.png");
            return _icon;
         }
      }

      protected override void RegisterInputParams(GH_InputParamManager pManager)
      {
         pManager.AddGeometryParameter("Hosting Point / Spt", "Pt / Spt", "Hosting Point / SOFiSTiK Structural Point", GH_ParamAccess.list);
         pManager.AddIntegerParameter("LoadCase", "LoadCase", "Id of Load Case", GH_ParamAccess.list, 1);
         pManager.AddVectorParameter("Force", "Force", "Acting Force [kN]", GH_ParamAccess.list, new Vector3d());
         pManager.AddVectorParameter("Moment", "Moment", "Acting Moment [kNm]", GH_ParamAccess.list, new Vector3d());
         pManager.AddVectorParameter("Displacement", "Disp", "Displacement [mm]", GH_ParamAccess.list, new Vector3d());
         pManager.AddVectorParameter("Rotational Displacement", "RotDisp", "Rotational Displacement [rad]", GH_ParamAccess.list, new Vector3d());
         pManager.AddBooleanParameter("HostLocal", "HostLocal", "Use local coordinate system of host", GH_ParamAccess.list, false);
      }

      protected override void RegisterOutputParams(GH_OutputParamManager pManager)
      {
         pManager.AddGeometryParameter("Point Load", "PLd", "SOFiSTiK Point Load", GH_ParamAccess.list);
      }

      protected override void SolveInstance(IGH_DataAccess da)
      {
         var points = da.GetDataList<IGH_GeometricGoo>(0);
         var loadcases = da.GetDataList<int>(1);
         var forces = da.GetDataList<Vector3d>(2);
         var moments = da.GetDataList<Vector3d>(3);
         var displacements = da.GetDataList<Vector3d>(4);
         var rotationalDisplacements = da.GetDataList<Vector3d>(5);
         var hostlocals = da.GetDataList<bool>(6);

         var gs_point_loads = new List<GS_PointLoad>();

         for (int i = 0; i < points.Count; ++i)
         {
            var point = points.GetItemOrLast(i);

            if (!(point is null))
            {
               var pl = new GS_PointLoad()
               {
                  LoadCase = loadcases.GetItemOrLast(i),
                  Forces = forces.GetItemOrLast(i),
                  Moments = moments.GetItemOrLast(i),
                  Displacement = displacements.GetItemOrLast(i),
                  DisplacementRotational = rotationalDisplacements.GetItemOrLast(i),
                  UseHostLocal = hostlocals.GetItemOrLast(i)
               };

               bool addPoint = true;
               if (point is GS_StructuralPoint)
               {
                  var spt = point as GS_StructuralPoint;

                  pl.Value = spt.Value;
                  pl.ReferencePoint = spt; // pass reference of structural point
               }
               else if (point is GH_Point)
               {
                  pl.Value = new Point((point as GH_Point).Value);
               }
               else
               {
                  AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Unable to Cast input to Point Geometry");
                  addPoint = false;
               }

               if (addPoint)
                  gs_point_loads.Add(pl);
            }
         }
         da.SetDataList(0, gs_point_loads);
      }

      public override Guid ComponentGuid
      {
         get { return new Guid("CA3198C1-C700-4A67-BA9F-F9F83EE73B93"); }
      }
   }
}
