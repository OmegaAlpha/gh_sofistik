﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace gh_sofistik
{
   public class GS_StructuralArea : GH_GeometricGoo<Brep>, IGH_PreviewData, IGH_BakeAwareData, IGS_StructuralElement
   {
      public int Id { get; set; } = 0;
      public int GroupId { get; set; } = 0;
      public int MaterialId { get; set; } = 0;
      public int ReinforcementId { get; set; } = 0;
      public double Thickness { get; set; } = 0.0;
      public Vector3d DirectionLocalX { get; set; } = new Vector3d();
      public double Bedding { get; set; } = 0.0;
      public string Text { get; set; } = "";

      public override BoundingBox Boundingbox
      {
         get { return Value.GetBoundingBox(true); }
      }

      public override string TypeDescription
      {
         get { return Value.ToString() + " with Structural Properties"; }
      }

      public override string TypeName
      {
         get { return "GS_StructuralArea"; }
      }

      public override string ToString()
      {
         return "Structural Area, Id = " + Id.ToString();
      }

      public override IGH_GeometricGoo DuplicateGeometry()
      {
         return new GS_StructuralArea()
         {
            Value = this.Value.DuplicateBrep(),
            Id = this.Id,
            GroupId = this.GroupId,
            MaterialId = this.MaterialId,
            ReinforcementId = this.ReinforcementId,
            Thickness = this.Thickness,
            DirectionLocalX = this.DirectionLocalX,
            Bedding = this.Bedding,
            Text = this.Text
         
         };
      }

      public override BoundingBox GetBoundingBox(Transform xform)
      {
         return xform.TransformBoundingBox(Value.GetBoundingBox(true));
      }

      public override bool CastTo<Q>(out Q target)
      {
         if(Value != null)
         {
            if(typeof(Q).IsAssignableFrom(typeof(GH_Brep)))
            {
               var gb = new GH_Brep(this.Value);
               target = (Q)(object)gb;
               return true;
            }
            else
            {
               throw new Exception("Unable to cast to type: " + typeof(Q).ToString());
            }
         }

         target = default(Q);
         return false;
      }

      public override IGH_GeometricGoo Morph(SpaceMorph xmorph)
      {
         var dup = this.DuplicateGeometry() as GS_StructuralArea;
         xmorph.Morph(dup.Value);

         return dup;
      }

      public override IGH_GeometricGoo Transform(Transform xform)
      {
         var dup = this.DuplicateGeometry() as GS_StructuralArea;
         dup.Value.Transform(xform);

         return dup;
      }

      public BoundingBox ClippingBox
      {
         get { return Boundingbox; }
      }

      public void DrawViewportWires(GH_PreviewWireArgs args)
      {
         if (Value != null)
         {
            args.Pipeline.DrawBrepWires(Value, System.Drawing.Color.Red);
         }
      }

      public void DrawViewportMeshes(GH_PreviewMeshArgs args)
      {
         //throw new NotImplementedException();
      }

      public bool BakeGeometry(RhinoDoc doc, ObjectAttributes baking_attributes, out Guid obj_guid)
      {
         if (Value != null)
         {
            var att = baking_attributes.Duplicate();

            var id_str = this.Id > 0 ? Id.ToString() : "0"; 
            var grp_str = this.GroupId.ToString();
            var mno_str = this.MaterialId.ToString();
            var mrf_str = this.ReinforcementId.ToString();
            var t_str = this.Thickness.ToString();

            att.SetUserString("SOF_OBJ_TYPE", "SAR");
            att.SetUserString("SOF_ID", id_str);
            att.SetUserString("SOF_T", Thickness.ToString());
            if(GroupId>0)
               att.SetUserString("SOF_GRP", GroupId.ToString());
            if (MaterialId > 0)
               att.SetUserString("SOF_MNO", MaterialId.ToString());
            if (ReinforcementId > 0)
               att.SetUserString("SOF_MRF", ReinforcementId.ToString());

            if(DirectionLocalX.Length > 1.0e-6)
            {
               att.SetUserString("SOF_DRX", DirectionLocalX.X.ToString("F6"));
               att.SetUserString("SOF_DRX", DirectionLocalX.Y.ToString("F6"));
               att.SetUserString("SOF_DRZ", DirectionLocalX.Z.ToString("F6"));
            }

            obj_guid = doc.Objects.AddBrep(Value, att);
         }
         else
         {
            obj_guid = new Guid();
         }
         return true;
      }

   }


   // Structural Area node
   public class CreateStructuralArea : GH_Component
   {
      public CreateStructuralArea()
         : base("Structural Area","Structural Area","Creates SOFiSTiK Structural Areas","SOFiSTiK", "Structure")
      { }

      protected override System.Drawing.Bitmap Icon
      {
         get { return Properties.Resources.structural_area16; }
      }

      protected override void RegisterInputParams(GH_InputParamManager pManager)
      {
         pManager.AddBrepParameter("Brep", "Brep", "Brep / Surface Geometry", GH_ParamAccess.list);
         pManager.AddIntegerParameter("Number", "Number", "Identifier of structural area", GH_ParamAccess.list, 0);
         pManager.AddIntegerParameter("Group", "Group", "Group numbers", GH_ParamAccess.list, 0);
         pManager.AddNumberParameter("Thickness", "Thickness", "Thickness of structural area", GH_ParamAccess.list, 0.0);
         pManager.AddIntegerParameter("Material", "Material", "Material number", GH_ParamAccess.list, 0);
         pManager.AddIntegerParameter("ReinforcementMaterial", "ReinfMat", "Reinforcement material number", GH_ParamAccess.list, 0);
         pManager.AddVectorParameter("Dir x", "Dir x", "Direction of local x-axis", GH_ParamAccess.list, new Vector3d());
         pManager.AddNumberParameter("Bedding", "Bedding", "Bedding modulus for normal subgrade bedding [kN/m^3]", GH_ParamAccess.list, 0);
         pManager.AddTextParameter("Text", "Text", "Extra text to add to structural area definition", GH_ParamAccess.list, "");
      }

      protected override void RegisterOutputParams(GH_OutputParamManager pManager)
      {
         pManager.AddGeometryParameter("Structural Area", "Sar", "SOFiSTiK Structural Area", GH_ParamAccess.list);
      }

      protected override void SolveInstance(IGH_DataAccess da)
      {
         var breps = da.GetDataList<Brep>(0);
         var ids = da.GetDataList<int>(1);
         var groups = da.GetDataList<int>(2);
         var thicknss = da.GetDataList<double>(3);
         var materials = da.GetDataList<int>(4);
         var matreinfs = da.GetDataList<int>(5);
         var xdirs = da.GetDataList<Vector3d>(6);
         var bedding = da.GetDataList<double>(7);
         var text = da.GetDataList<string>(8);

         var gh_structural_areas = new List<GS_StructuralArea>();

         for(int i=0; i<breps.Count; ++i)
         {
            var b = breps[i];

            var ga = new GS_StructuralArea()
            {
               Value = b,
               Id = ids.GetItemOrCountUp(i),
               GroupId = groups.GetItemOrLast(i),
               MaterialId = materials.GetItemOrLast(i),
               ReinforcementId = matreinfs.GetItemOrLast(i),
               Thickness = thicknss.GetItemOrLast(i),
               DirectionLocalX = xdirs.GetItemOrLast(i),
               Bedding = bedding.GetItemOrLast(i),
               Text = text.GetItemOrLast(i)
            };
            gh_structural_areas.Add(ga);
         }

         da.SetDataList(0, gh_structural_areas);
      }

      public override Guid ComponentGuid
      {
         get { return new Guid("FC0973D1-0EC1-435A-ABD2-F6D9C7D3D7F3"); }
      }
   }

}
