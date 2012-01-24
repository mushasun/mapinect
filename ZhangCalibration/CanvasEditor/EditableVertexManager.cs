using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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
			foreach (Point point in Polygon)
			{
				
			}
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
