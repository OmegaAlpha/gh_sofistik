using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using gh_sofistik;


public class CreateGlobalSettings : GH_Component
{
	public CreateGlobalSettings() : base("Global Settings ", "Settings", "Global settings for scaling load arrows / adjusting colors / etc.", "SOFiSTiK", "General")
   {
	}

   public override Guid ComponentGuid
   {
      get
      {
         return new Guid("32EDDB4F-DD7F-42E5-9F7A-298D5035BE9F");
      }
   }

   protected override void RegisterInputParams(GH_InputParamManager pManager)
   {      
      pManager.AddColourParameter("Color Structural Elements", "Color Structural Elements", "Adjust Color for Structural Points/Lines/Areas", GH_ParamAccess.item, System.Drawing.Color.Red);
      pManager.AddColourParameter("Color Forces", "Color Forces", "Adjust Color for Point/Line/Area Load Forces", GH_ParamAccess.item, System.Drawing.Color.IndianRed);
      pManager.AddNumberParameter("Scale Factor Arrow Size", "Scale Factor", "Global Scale Factor for displayed SOFiSTiK Loads", GH_ParamAccess.item, 1.0);
      pManager.AddNumberParameter("Density Factor Arrows", "Density Factor", "Global Density for Line/Area Loads", GH_ParamAccess.item, 1.0);
   }

   protected override void RegisterOutputParams(GH_OutputParamManager pManager)
   {
      //no output. just for scaling displayed forces
   }

   protected override void SolveInstance(IGH_DataAccess DA)
   {      
      DrawUtil.DrawColStrc = DA.GetData<System.Drawing.Color>(0);
      DrawUtil.DrawColForces = DA.GetData<System.Drawing.Color>(1);
      DrawUtil.ScaleFactor = DA.GetData<double>(2);
      DrawUtil.DensityFactor = DA.GetData<double>(3);
   }
}
