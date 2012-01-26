using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Shapes;
using System.Collections.ObjectModel;
using PropertyChangedLibrary;
using System.Windows.Data;
using System.Windows;

namespace CanvasEditor
{
	public class EditablePolygon : EditableShape
	{

		public EditablePolygon(CanvasEditor editor, FrameworkElement element)
			: base(editor, element)
		{
			PointCollection = new PointCollection(Polygon.Points);
		}

		private PointCollection myPointCollection;
		public PointCollection PointCollection {
			get
			{
				return myPointCollection;
			}
			set
			{
				myPointCollection = value;
				if (Polygon != null)
				{
					Binding pointsBinding = new Binding(PointCollection.PointsProperty);
					pointsBinding.Source = myPointCollection;
					pointsBinding.Converter = new PointCollectionConverter();
					Polygon.SetValue(Polygon.PointsProperty, pointsBinding);
				}
			}
		}

		private Polygon Polygon
		{
			get
			{
				return (Polygon)UIElement;
			}
		}

	}

	public class PointCollection : NotifyPropertyChanged.NotifyPropertyChanged
	{

		public PointCollection(System.Windows.Media.PointCollection pointCollection)
		{
			ObservableCollection<PointNotifyPropertyChanged> points = new ObservableCollection<PointNotifyPropertyChanged>();
			foreach (Point point in pointCollection)
			{
				points.Add(new PointNotifyPropertyChanged(point));
			}
			Points = points;
		}

		public const string PointsProperty = "Points";
		private ObservableCollection<PointNotifyPropertyChanged> myPoints;
		public ObservableCollection<PointNotifyPropertyChanged> Points
		{
			get
			{
				return myPoints;
			}
			set
			{
				myPoints = value;
				FirePropertyChanged(PointsProperty);
			}
		}

	}

	[ValueConversion(typeof(PointCollection), typeof(System.Windows.Media.PointCollection))]
	internal class PointCollectionConverter : IValueConverter
	{

		public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			PointCollection original = (PointCollection)value;
			System.Windows.Media.PointCollection target = new System.Windows.Media.PointCollection(
				original.Points.Select(pn => pn.Point).ToList());
			return target;
		}

		public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			throw new NotImplementedException();
		}

	}

}
