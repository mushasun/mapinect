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
			PointCollection.CollectionChanged += new System.Collections.Specialized.NotifyCollectionChangedEventHandler(PointCollection_CollectionChanged);
			VertexManager = new EditableVertexManager(this);
		}

		public PointCollection PointCollection { get; private set; }

		private EditableVertexManager VertexManager { get; set; }

		private Polygon Polygon
		{
			get
			{
				return (Polygon)UIElement;
			}
		}

		private void PointCollection_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
		{
			Polygon.Points = PointCollection.ToPointCollection;
		}

	}

	public class PointCollection : ObservableCollection<PointNotifyPropertyChanged>
	{

		public PointCollection(System.Windows.Media.PointCollection pointCollection)
		{
			foreach (Point point in pointCollection)
			{
				Add(new PointNotifyPropertyChanged(point));
			}
		}

		public System.Windows.Media.PointCollection ToPointCollection
		{
			get
			{
				return new System.Windows.Media.PointCollection(this.Select(p => p.Point));
			}
		}

	}

}
