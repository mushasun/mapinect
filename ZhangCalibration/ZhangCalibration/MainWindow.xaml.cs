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
			myDataCalibration.Canvas = CanvasCalibration;
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

		}

		private void ButtonAddQuad_Click(object sender, RoutedEventArgs e)
		{
			Quad quad = new Quad();
			myEditingQuad = new EditingQuad(quad);
			myDataCalibration.MyQuads.Add(myEditingQuad);
		}

		public DataCalibration myDataCalibration { get; set; }
		public EditingQuad myEditingQuad { get; set; }

		private Point dragStartPosition;
	}
}
