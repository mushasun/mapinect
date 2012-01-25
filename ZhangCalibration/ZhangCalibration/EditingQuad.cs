using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Data;
using System.Windows.Shapes;
using System.Windows.Media;
using System.Windows.Controls;
using System.Collections.ObjectModel;

namespace ZhangCalibration
{
	public class EditingQuad : NotifyPropertyChanged.NotifyPropertyChanged
	{

		private Brush UnselectedStrokeColor = Brushes.Gray;
		private Brush SelectedStrokeColor = Brushes.Blue;
		private Brush PolygonFillColor = new SolidColorBrush(Color.FromArgb(64, 255, 255, 0));
		private Brush HandleColor = new SolidColorBrush(Color.FromArgb(192, 255, 0, 0));

		public EditingQuad(Quad quad)
		{
			this.MyQuad = quad;
			this.MyPolygon = new Polygon();
			MyPolygon.Fill = PolygonFillColor;
			MyPolygon.StrokeThickness = 1;
		}

		~EditingQuad()
		{
			MyQuad = null;
			MyPolygon = null;
		}

		public override string ToString()
		{
			return MyQuad.ToString();
		}

		public const string MyQuadProperty = "MyQuad";
		private Quad _MyQuad;
		public Quad MyQuad
		{
			get
			{
				return _MyQuad;
			}
			set
			{
				_MyQuad = value;
				FirePropertyChanged(MyQuadProperty);
			}
		}

		public Polygon MyPolygon { get; private set; }

	}

}
