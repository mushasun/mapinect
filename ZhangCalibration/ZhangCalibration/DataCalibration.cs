using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ZhangCalibration
{
	public class DataCalibration : NotifyPropertyChanged.NotifyPropertyChanged
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
			foreach (EditingQuad editingQuad in quads)
			{
				editingQuad.PropertyChanged += new System.ComponentModel.PropertyChangedEventHandler(editingQuad_PropertyChanged);
			}
			MyQuads = quads;
		}

		void editingQuad_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
		{
			if (e.PropertyName == EditingQuad.IsEditingProperty)
			{
				EditingQuad editingQuad = (EditingQuad)sender;
				foreach (EditingQuad eq in MyQuads)
				{
					if (eq != editingQuad)
					{
						eq.IsEditing = false;
					}
				}
			}
		}

		public string Filename { get; set; }
		public string ImageFilename { get; set; }

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
