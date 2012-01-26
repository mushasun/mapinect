using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Controls;
using System.Windows.Media;

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

			myCanvasEditor.Clear();

			Image CalibrationImage = new Image();
			CalibrationImage.Source = new System.Windows.Media.Imaging.BitmapImage(
				new Uri(ImageFilename, UriKind.RelativeOrAbsolute));
			CalibrationImage.Width = 640;
			CalibrationImage.Height = 480;
			CalibrationImage.Stretch = Stretch.Fill;
			myCanvasEditor.Canvas.Children.Add(CalibrationImage);

			foreach (EditingQuad editingQuad in quads)
			{
				myCanvasEditor.Canvas.Children.Add(editingQuad.Polygon);
				editingQuad.PropertyChanged += new System.ComponentModel.PropertyChangedEventHandler(editingQuad_PropertyChanged);
			}
			myCanvasEditor.EditableUIElements = myCanvasEditor.Canvas.Children;
			MyQuads = quads;
		}

		public string Filename { get; set; }
		public string ImageFilename { get; set; }

		private CanvasEditor.CanvasEditor myCanvasEditor;
		public Canvas Canvas {
			set
			{
				myCanvasEditor = new CanvasEditor.CanvasEditor(value);
			}
		}

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

		private void editingQuad_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
		{
			if (sender is EditingQuad && e.PropertyName == EditingQuad.SelectedProperty)
			{
				EditingQuad editingQuad = (EditingQuad)sender;
				myCanvasEditor.SelectByUIElement(editingQuad.Polygon);
			}
		}

	}
}
