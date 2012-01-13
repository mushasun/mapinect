using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Data;
using System.Windows.Shapes;

namespace ZhangCalibration
{
	public class EditingQuad : NotifyPropertyChanged
	{

		public EditingQuad(Quad quad)
		{
			this.MyQuad = quad;
			this.MyPolygon = new Polygon();
			MyPolygon.Stroke = System.Windows.Media.Brushes.Red;
			MyPolygon.StrokeThickness = 1;
			Binding binding = new Binding(Quad.PointsProperty);
			binding.Source = MyQuad;
			binding.Mode = BindingMode.TwoWay;
			MyPolygon.SetBinding(Polygon.PointsProperty, binding);
		}

		~EditingQuad()
		{
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
