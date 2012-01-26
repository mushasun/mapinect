using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Shapes;

namespace CanvasEditor
{

	enum EditableHandlePosition
	{
		First = 0,
		TopLeft = First,
		TopCenter,
		TopRight,
		MidRight,
		BottomRight,
		BottomCenter,
		BottomLeft,
		MidLeft,
		Count
	}

	internal class EditableHandle : EditableAlteringObject
	{

		internal EditableHandle(EditableAlteringManager manager, EditableHandlePosition handlePosition)
			: base(manager)
		{
			HandlePosition = handlePosition;
		}

		public EditableHandlePosition HandlePosition { get; set; }

		internal override void UpdatePosition()
		{
			Point point = ((EditableHandleManager)Manager).EditableHandlePositionToPoint(HandlePosition);
			Position.X = point.X;
			Position.Y = point.Y;
		}

		protected override FrameworkElement NewAlteringFrameworkElement()
		{
			return new Rectangle();
		}
	}
}
