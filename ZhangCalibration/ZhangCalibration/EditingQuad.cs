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

			Binding binding = new Binding(Quad.PointsProperty);
			binding.Source = MyQuad;
			MyPolygon.SetBinding(Polygon.PointsProperty, binding);

			MyPolygon.MouseLeftButtonDown += new System.Windows.Input.MouseButtonEventHandler(MyPolygon_MouseLeftButtonDown);

			MyVertexCircles = new ObservableCollection<Ellipse>();

			for (int i = 0; i < MyQuad.Points.Count; i++)
			{
				Ellipse circle = new Ellipse();
				circle.Fill = HandleColor;
				circle.SetValue(Canvas.WidthProperty, EditingQuadVertexConverter.HandlesSize);
				circle.SetValue(Canvas.HeightProperty, EditingQuadVertexConverter.HandlesSize);

				Binding xBinding = new Binding("X");
				xBinding.Source = MyQuad.Points[i];
				xBinding.Mode = BindingMode.TwoWay;
				xBinding.Converter = new EditingQuadVertexConverter();
				circle.SetBinding(Canvas.LeftProperty, xBinding);

				Binding yBinding = new Binding("Y");
				yBinding.Source = MyQuad.Points[i];
				yBinding.Mode = BindingMode.TwoWay;
				yBinding.Converter = new EditingQuadVertexConverter();
				circle.SetBinding(Canvas.TopProperty, yBinding);

				circle.MouseLeftButtonDown += new System.Windows.Input.MouseButtonEventHandler(circle_MouseLeftButtonDown);
				circle.MouseLeftButtonUp += new System.Windows.Input.MouseButtonEventHandler(circle_MouseLeftButtonUp);

				MyVertexCircles.Add(circle);
			}

			IsEditing = false;
		}

		~EditingQuad()
		{
			MyQuad = null;
			MyPolygon = null;
		}

		void MyPolygon_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			if (DraggingVertex == null)
			{
				IsEditing = true;
			}
		}

		void circle_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			if (MyVertexCircles.Contains(sender))
			{
				DraggingVertex = (Ellipse)sender;
				e.Handled = true;
			}
		}

		void circle_MouseLeftButtonUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			if (sender == DraggingVertex)
			{
				DraggingVertex = null;
				e.Handled = true;
			}
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
		public ObservableCollection<Ellipse> MyVertexCircles { get; private set; }

		public const string IsEditingProperty = "IsEditing";
		private bool _IsEditing;
		public bool IsEditing {
			get
			{
				return IsEditing;
			}
			set
			{
				_IsEditing = value;
				MyPolygon.Stroke = _IsEditing ? SelectedStrokeColor : UnselectedStrokeColor;
				for (int i = 0; i < MyVertexCircles.Count; i++)
				{
					MyVertexCircles[i].Visibility = _IsEditing ?
						System.Windows.Visibility.Visible : System.Windows.Visibility.Hidden;
				}
				if (_IsEditing)
				{
					FirePropertyChanged(IsEditingProperty);
				}
			}
		}

		public const string DraggingVertexProperty = "DraggingVertex";
		private Ellipse _DraggingVertex;
		public Ellipse DraggingVertex
		{
			get
			{
				return _DraggingVertex;
			}
			set
			{
				_DraggingVertex = value;
				FirePropertyChanged(DraggingVertexProperty);
			}
		}

		internal void OffsetDraggingVertex(System.Windows.Vector dif)
		{
			int index = MyVertexCircles.IndexOf(DraggingVertex);
			System.Windows.Point point = MyQuad.Points[index];
			point.Offset(dif.X, dif.Y);
			MyQuad.Points[index] = point;
			MyPolygon.Points = MyQuad.Points;
			DraggingVertex.SetValue(Canvas.LeftProperty,
				(double)DraggingVertex.GetValue(Canvas.LeftProperty) + dif.X);
			DraggingVertex.SetValue(Canvas.TopProperty,
				(double)DraggingVertex.GetValue(Canvas.TopProperty) + dif.Y);
		}

	}

	[ValueConversion(typeof(double), typeof(double))]
	class EditingQuadVertexConverter : IValueConverter
	{

		public const double HandlesSize = 6.0;

		public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value - HandlesSize * .5;
		}

		public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value + HandlesSize * .5;
		}
	}
}
