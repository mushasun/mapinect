using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ZhangCalibration
{
	public class DataCalibration : NotifyPropertyChanged
	{
		public const string SupportedImageFormat = ".tif";

		public DataCalibration()
		{
		}

		public void Load(string imageFilename)
		{
			this.ImageFilename = imageFilename;
			this.Filename = imageFilename.Replace(SupportedImageFormat, ".txt");
			System.IO.TextReader tr = new System.IO.StreamReader(this.Filename);
			string data = tr.ReadToEnd();
			List<EditingQuad> quads = new List<EditingQuad>();
			DataLoader.ParseData(data).ForEach(q => quads.Add(new EditingQuad(q)));
			Quads = quads;
		}

		public string Filename { get; set; }
		public string ImageFilename { get; set; }

		private List<EditingQuad> _Quads;
		public List<EditingQuad> Quads {
			get
			{
				return _Quads;
			}
			set
			{
				_Quads = value;
				FirePropertyChanged("Quads");
			}
		}

	}
}
