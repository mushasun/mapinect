using System.Collections.Generic;
using System.Windows;

namespace CanvasEditor
{
	internal class EditableHandleManager : EditableAlteringManager
	{

		internal EditableHandleManager(IEditable editable)
			: base(editable)
		{

		}

		protected override List<EditableAlteringObject> GetAlteringObjects()
		{
			List<EditableAlteringObject> result = new List<EditableAlteringObject>();
			for (EditableHandlePosition position = EditableHandlePosition.First; position < EditableHandlePosition.Count; position++)
			{
				EditableHandle handle = new EditableHandle(this, position);
				result.Add(handle);
			}
			return result;
		}

		internal Point EditableHandlePositionToPoint(EditableHandlePosition handlePosition)
		{
			Point point;
			Point position = Editable.Position.Point;
			Size size = Editable.Size.Size;
			switch (handlePosition)
			{
				case EditableHandlePosition.TopLeft:
					point = position;
					break;
				case EditableHandlePosition.TopCenter:
					point = new Point(position.X + size.Width * .5, position.Y);
					break;
				case EditableHandlePosition.TopRight:
					point = new Point(position.X + size.Width, position.Y);
					break;
				case EditableHandlePosition.MidRight:
					point = new Point(position.X + size.Width, position.Y + size.Height * .5);
					break;
				case EditableHandlePosition.BottomRight:
					point = new Point(position.X + size.Width, position.Y + size.Height);
					break;
				case EditableHandlePosition.BottomCenter:
					point = new Point(position.X + size.Width * .5, position.Y + size.Height);
					break;
				case EditableHandlePosition.BottomLeft:
					point = new Point(position.X, position.Y + size.Height);
					break;
				case EditableHandlePosition.MidLeft:
				default:
					point = new Point(position.X, position.Y + size.Height * .5);
					break;
			}

			return point;
		}

	}
}
