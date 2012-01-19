using System.Collections.Generic;
using System.Windows;

namespace CanvasEditor
{
	internal class EditableHandleManager
	{

		internal EditableHandleManager(IEditable editable)
		{
			Editable = editable;
			HandlesVisible = false;
		}

		public IEditable Editable { get; private set; }

		private bool myHandlesVisible;
		public bool HandlesVisible
		{
			get
			{
				return myHandlesVisible;
			}
			set
			{
				myHandlesVisible = value;
				if (myHandlesVisible)
				{
					CreateHandlesIfNecessary();
					UpdateHandlesPosition();
				}
				if (Handles != null)
				{
					Handles.ForEach(h => h.IsVisible = myHandlesVisible);
				}
			}
		}

		private List<EditableHandle> Handles { get; set; }

		private void CreateHandlesIfNecessary()
		{
			if (Handles == null)
			{
				Handles = new List<EditableHandle>();
				for (EditableHandlePosition position = EditableHandlePosition.First; position < EditableHandlePosition.Count; position++)
				{
					EditableHandle handle = new EditableHandle(this, position);
				}
			}
		}

		private void UpdateHandlesPosition()
		{
			foreach (EditableHandle handle in Handles)
			{
				Point position = EditableHandlePositionToPoint(handle.HandlePosition);
				handle.Position.X = position.X;
				handle.Position.Y = position.Y;
			}
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
