using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Controls;

namespace ZhangCalibration
{
	public class DataCalibration : NotifyPropertyChanged.NotifyPropertyChanged
	{
		public const string SupportedImageFormat = ".tif";

		public DataCalibration(Canvas canvas)
		{
			CanvasEditor = new CanvasEditor.CanvasEditor(canvas);
		}

		public void Load(string imageFilename)
		{
			this.ImageFilename = imageFilename;
			this.Filename = imageFilename.Replace(SupportedImageFormat, ".txt");
			System.IO.TextReader tr = new System.IO.StreamReader(this.Filename);
			string data = tr.ReadToEnd();
			List<EditingQuad> quads = new List<EditingQuad>();
			DataLoader.ParseData(data).ForEach(q => quads.Add(new EditingQuad(q)));
			foreach (EditingQuad editingQuad in quads)
			{
				
			}
			MyQuads = quads;
		}

		public string Filename { get; set; }
		public string ImageFilename { get; set; }

		private CanvasEditor.CanvasEditor CanvasEditor;

		public const string QuadsProperty = "MyQuads";
		private List<EditingQuad> _MyQuads;
		public List<EditingQuad> MyQuads {
			get
			{
				return _MyQuads;
			}
			set
			{
				_MyQuads = value;
				FirePropertyChanged(QuadsProperty);
			}
		}

	}
}
