using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Shapes;
using System.Collections.ObjectModel;
using PropertyChangedLibrary;
using System.Windows.Data;

namespace CanvasEditor
{
	class EditablePolygon : EditableShape
	{

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

	class PointCollection : NotifyPropertyChanged.NotifyPropertyChanged
	{

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
}
