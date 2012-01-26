using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PropertyChangedLibrary;

namespace CanvasEditor
{
	internal class EditableVertexManager : EditableAlteringManager
	{

		internal EditableVertexManager(EditablePolygon polygon)
			: base(polygon)
		{

		}

		protected override List<EditableAlteringObject> GetAlteringObjects()
		{
			List<EditableAlteringObject> result = new List<EditableAlteringObject>();

			return result;
		}

		internal PointNotifyPropertyChanged EditableVertexIndexToPoint(int vertexIndex)
		{
			return Polygon.PointCollection[vertexIndex];
		}

		private EditablePolygon Polygon
		{
			get 
			{
				return (EditablePolygon)Editable;
			}
		}

	}
}
