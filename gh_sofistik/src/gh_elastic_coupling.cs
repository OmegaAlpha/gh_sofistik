﻿using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace gh_sofistik.Structure
{

   public class GH_Elastic_Coupling : GH_GeometricGoo<GH_CouplingStruc>, IGH_PreviewData
   {
      private CouplingCondition _cplCond = new CouplingCondition();
      private InfoPanel _infoPanel;

      public int GroupId { get; set; } = 0;

      public double Axial_stiffness { get; set; } = 0.0;

      public double Rotational_stiffness { get; set; } = 0.0;

      public double Transversal_stiffness { get; set; } = 0.0;

      public Vector3d Direction { get; set; } = new Vector3d();

      public BoundingBox ClippingBox
      {
         get
         {
            BoundingBox bBox;
            if (Value.IsACurve)
               bBox = Value.CurveA.GetBoundingBox(false);
            else
               bBox = Value.PointA.GetBoundingBox(false);
            if (Value.IsBCurve)
               bBox.Union(Value.CurveB.GetBoundingBox(false));
            else
               bBox.Union(Value.PointB.GetBoundingBox(false));
            return bBox;
         }
      }

      public override BoundingBox Boundingbox
      {
         get
         {
            BoundingBox bBox;
            if (Value.IsACurve)
               bBox = Value.CurveA.GetBoundingBox(true);
            else
               bBox = Value.PointA.GetBoundingBox(true);
            if (Value.IsBCurve)
               bBox.Union(Value.CurveB.GetBoundingBox(true));
            else
               bBox.Union(Value.PointB.GetBoundingBox(true));
            return bBox;
         }
      }

      public override string TypeName
      {
         get
         {
            return "GH_Elastic_Coupling";
         }
      }

      public override string TypeDescription
      {
         get
         {
            return "Elastic Coupling between Points and Lines";
         }
      }

      public void DrawViewportMeshes(GH_PreviewMeshArgs args)
      {

      }

      public void DrawViewportWires(GH_PreviewWireArgs args)
      {
         //ClippingBox
         //args.Pipeline.DrawBox(ClippingBox, System.Drawing.Color.Black);
         if (!_cplCond.isValid)
         {
            updateECoupling();
         }

         System.Drawing.Color col = args.Color;
         if (!DrawUtil.CheckSelection(col))
            col = System.Drawing.Color.DarkViolet;
         else
            drawInfoPanel(args.Pipeline, args.Viewport);

         _cplCond.Draw(args.Pipeline, col);
      }

      private void drawInfoPanel(Rhino.Display.DisplayPipeline pipeline, Rhino.Display.RhinoViewport viewport)
      {
         if (DrawUtil.DrawInfo)
         {
            if (_infoPanel == null)
            {
               _infoPanel = new InfoPanel();
               var lines = Value.GetConnectionLines();
               if (lines.Count == 1)
               {
                  _infoPanel.Positions.Add((lines[0].From + lines[0].To) * 0.5);
               }
               else
               {
                  _infoPanel.Positions.Add((lines[0].From + lines[0].To) * 0.5);
                  _infoPanel.Positions.Add((lines[lines.Count - 1].From + lines[lines.Count - 1].To) * 0.5);
                  _infoPanel.Positions.Add((lines[0].From + lines[lines.Count - 1].From) * 0.5);
                  _infoPanel.Positions.Add((lines[0].To + lines[lines.Count - 1].To) * 0.5);
               }
               if (GroupId != 0)
                  _infoPanel.Content.Add("Grp: " + GroupId);
               _infoPanel.Content.Add("Stf: " + Axial_stiffness + " / " + Rotational_stiffness + " / " + Transversal_stiffness);
            }
            _infoPanel.Draw(pipeline, viewport);
         }
      }

      private void updateECoupling()
      {
         _cplCond = new CouplingCondition();
         _cplCond.CreateDottedLineSymbols(Value.GetConnectionLines());
      }

      public override IGH_GeometricGoo DuplicateGeometry()
      {
         GH_Elastic_Coupling nc = new GH_Elastic_Coupling();
         nc.Value = new GH_CouplingStruc();
         if (Value.Reference_A != null)
         {
            if (Value.IsACurve)
               nc.Value.Reference_A = (Value.Reference_A as GS_StructuralLine).DuplicateGeometry() as GS_StructuralLine;
            else
               nc.Value.Reference_A = (Value.Reference_A as GS_StructuralPoint).DuplicateGeometry() as GS_StructuralPoint;
         }
         if (Value.Reference_B != null)
         {
            if (Value.IsBCurve)
               nc.Value.Reference_B = (Value.Reference_B as GS_StructuralLine).DuplicateGeometry() as GS_StructuralLine;
            else
               nc.Value.Reference_B = (Value.Reference_B as GS_StructuralPoint).DuplicateGeometry() as GS_StructuralPoint;
         }
         nc.GroupId = GroupId;
         nc.Axial_stiffness = Axial_stiffness;
         nc.Rotational_stiffness = Rotational_stiffness;
         nc.Transversal_stiffness = Transversal_stiffness;
         nc.Direction = Direction;
         return nc;
      }

      public override BoundingBox GetBoundingBox(Transform xform)
      {
         return xform.TransformBoundingBox(ClippingBox);
      }

      public override IGH_GeometricGoo Transform(Transform xform)
      {
         GH_Elastic_Coupling nc = this.DuplicateGeometry() as GH_Elastic_Coupling;
         if (nc.Value.IsACurve)
            nc.Value.CurveA.Transform(xform);
         else
            nc.Value.PointA.Transform(xform);
         if (nc.Value.IsBCurve)
            nc.Value.CurveB.Transform(xform);
         else
            nc.Value.PointB.Transform(xform);

         var localDir = nc.Direction;
         localDir.Transform(xform);
         nc.Direction = localDir;

         return nc;
      }

      public override IGH_GeometricGoo Morph(SpaceMorph xmorph)
      {
         GH_Elastic_Coupling nc = this.DuplicateGeometry() as GH_Elastic_Coupling;
         if (nc.Value.IsACurve)
            xmorph.Morph(nc.Value.CurveA);
         else
            xmorph.Morph(nc.Value.PointA);
         if (nc.Value.IsBCurve)
            xmorph.Morph(nc.Value.CurveB);
         else
            xmorph.Morph(nc.Value.PointB);
         return nc;
      }

      public override string ToString()
      {
         return "Elastic Coupling" + (GroupId == 0 ? "" : ", Grp Id: " + GroupId) + ", AxStf: " + Axial_stiffness + ", RotStf: " + Rotational_stiffness + ", TransStf: " + Transversal_stiffness + (Direction.IsTiny() ? "" : ", Dir: " + Direction);
      }
   }

   public class CreateElasticCoupling : GH_Component
   {
      private System.Drawing.Bitmap _icon;

      public CreateElasticCoupling()
            : base("Elastic Coupling", "Elastic Coupling", "Creates SOFiSTiK Point/Point, Point/Line or Line/Line Elastic Coupling", "SOFiSTiK", "Structure")
      { }

      public override Guid ComponentGuid
      {
         get
         {
            return new Guid("C6F702F1-E3E2-47AC-880D-7210465E7530");
         }
      }

      protected override System.Drawing.Bitmap Icon
      {
         get
         {
            if (_icon == null)
               _icon = Util.GetBitmap(GetType().Assembly, "structural_elastic_constraint_24x24.png");
            return _icon;
         }
      }

      protected override void RegisterInputParams(GH_InputParamManager pManager)
      {
         pManager.AddGeometryParameter("A: Point/Curve", "A", "Root geometry (Point / Curve) of this Elastic Coupling", GH_ParamAccess.list);
         pManager.AddGeometryParameter("B: Reference Point/Curve", "B", "Reference geometry (Point / Curve) of this Elastic Coupling", GH_ParamAccess.list);
         pManager.AddIntegerParameter("Group", "Group", "Group number of this Elastic Coupling", GH_ParamAccess.list, 0);
         pManager.AddNumberParameter("Axial Stiffness", "Ax. Stf.", "Stiffness of this Elastic Coupling in axial direction [kN/m^3]", GH_ParamAccess.list, 0.0);
         pManager.AddNumberParameter("Rotational Stiffness", "Rot. Stf", "Stiffness of this Elastic Coupling in rotational direction [kNm/rad]", GH_ParamAccess.list, 0.0);
         pManager.AddNumberParameter("Transversal Stiffness", "Tr. Stf", "Stiffness of this Elastic Coupling in transversal direction [kN/m^3]", GH_ParamAccess.list, 0.0);
         pManager.AddVectorParameter("Explicit Direction", "Dir", "Explicit Direction of this Elastic Coupling. If no direction is given, the Coupling is aligned towards its reference point (default)", GH_ParamAccess.list, new Vector3d());
      }

      protected override void RegisterOutputParams(GH_OutputParamManager pManager)
      {
         pManager.AddGeometryParameter("Elastic Coupling", "E-Cpl", "SOFiSTiK Point/Point Point/Line Line/Line Elastic Coupling", GH_ParamAccess.list);
      }

      protected override void SolveInstance(IGH_DataAccess da)
      {

         List<IGH_GeometricGoo> a_list = da.GetDataList<IGH_GeometricGoo>(0);
         List<IGH_GeometricGoo> b_list = da.GetDataList<IGH_GeometricGoo>(1);
         List<int> groups = da.GetDataList<int>(2);
         List<double> axial_stiffness = da.GetDataList<double>(3);
         List<double> rotational_stiffness = da.GetDataList<double>(4);
         List<double> transversal_stiffness = da.GetDataList<double>(5);
         List<Vector3d> direction = da.GetDataList<Vector3d>(6);

         List<GH_Elastic_Coupling> out_list = new List<GH_Elastic_Coupling>();

         int count = Math.Max(a_list.Count, b_list.Count);

         for (int i = 0; i < count; i++)
         {
            IGH_GeometricGoo a_goo = a_list.GetItemOrLast(i);
            IGH_GeometricGoo b_goo = b_list.GetItemOrLast(i);
            
            GH_Elastic_Coupling spr = new GH_Elastic_Coupling();
            spr.Value = new GH_CouplingStruc();
            spr.GroupId = groups.GetItemOrLast(i);
            spr.Axial_stiffness = axial_stiffness.GetItemOrLast(i);
            spr.Rotational_stiffness = rotational_stiffness.GetItemOrLast(i);
            spr.Transversal_stiffness = transversal_stiffness.GetItemOrLast(i);
            spr.Direction = direction.GetItemOrLast(i);
            
            Enum state = spr.Value.SetInputs(a_goo, b_goo);
            if (state.Equals(GH_CouplingStruc.State.OK))
               out_list.Add(spr);
            else if (state.Equals(GH_CouplingStruc.State.InvalidA))
               this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input param A: only (structural)points/lines allowed");
            else if (state.Equals(GH_CouplingStruc.State.InvalidB))
               this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input param B: only (structural)points/lines allowed");
            else if (state.Equals(GH_CouplingStruc.State.Modified))
            {
               this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input parameters swapped, only Elastic Couplings from Lines to Points are supported");
               out_list.Add(spr);
            }
         }

         da.SetDataList(0, out_list);
      }
   }
}
