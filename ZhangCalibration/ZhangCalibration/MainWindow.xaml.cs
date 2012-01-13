using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace ZhangCalibration
{
	/// <summary>
	/// Interaction logic for MainWindow.xaml
	/// </summary>
	public partial class MainWindow : Window
	{
		public MainWindow()
		{
			InitializeComponent();
			myDataCalibration = (DataCalibration)Resources["myDataCalibration"];
			myDataCalibration.PropertyChanged += new System.ComponentModel.PropertyChangedEventHandler(myDataCalibration_PropertyChanged);
		}

		private void ButtonOpen_Click(object sender, RoutedEventArgs e)
		{
			Microsoft.Win32.OpenFileDialog ofd = new Microsoft.Win32.OpenFileDialog();
			ofd.DefaultExt = DataCalibration.SupportedImageFormat;
			ofd.Filter = "Image files (" + DataCalibration.SupportedImageFormat + ")|*" + DataCalibration.SupportedImageFormat;
			bool? result = ofd.ShowDialog();
			if (result == true)
			{
				string filename = ofd.FileName;
				myDataCalibration.Load(filename);
			}
		}

		void myDataCalibration_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
		{
			CanvasCalibration.Children.Clear();

			Image CalibrationImage = new Image();
			CalibrationImage.Source = new System.Windows.Media.Imaging.BitmapImage(
				new Uri(myDataCalibration.ImageFilename, UriKind.RelativeOrAbsolute));
			CalibrationImage.Width = 640;
			CalibrationImage.Height = 480;
			CalibrationImage.Stretch = Stretch.Fill;
			CanvasCalibration.Children.Add(CalibrationImage);

			for (int i = 0; i < myDataCalibration.Quads.Count; i++)
			{
				CanvasCalibration.Children.Add(myDataCalibration.Quads[i].Polygon);
			}
		}

		public DataCalibration myDataCalibration { get; set; }
		public EditingQuad myEditingQuad { get; set; }

		private void ButtonAddQuad_Click(object sender, RoutedEventArgs e)
		{
			Quad quad = new Quad();
			myEditingQuad = new EditingQuad(quad);
			myDataCalibration.Quads.Add(myEditingQuad);
		}

		private void CanvasCalibration_MouseDown(object sender, MouseButtonEventArgs e)
		{
			if (e.LeftButton == MouseButtonState.Pressed)
			{
				Polygon polygon = (Polygon)CanvasCalibration.InputHitTest(e.GetPosition(CanvasCalibration));
				if (polygon != null)
				{
					polygon.Stroke = Brushes.Green;
				}
			}
		}

	}
}
