using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Data;
using System.Windows.Shapes;
using System.Windows.Media;

namespace ZhangCalibration
{
	public class EditingQuad : NotifyPropertyChanged
	{

		public EditingQuad(Quad quad)
		{
			this.MyQuad = quad;
			this.MyPolygon = new Polygon();
			MyPolygon.Fill = new SolidColorBrush(Color.FromArgb(64, 255, 255, 0));
			MyPolygon.Stroke = Brushes.Red;
			MyPolygon.StrokeThickness = 1;
			Binding binding = new Binding(Quad.PointsProperty);
			binding.Source = MyQuad;
			MyPolygon.SetBinding(Polygon.PointsProperty, binding);
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

		public System.Windows.Shapes.Polygon MyPolygon { get; private set; }

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

		public const string EditingPointIndexProperty = "EditingPointIndex";
		private int _EditingPointIndex;
		public int EditingPointIndex
		{
			get
			{
				return _EditingPointIndex;
			}
			set
			{
				_EditingPointIndex = value;
				FirePropertyChanged(EditingPointIndexProperty);
			}
		}

	}
}
