using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Shapes;

namespace CanvasEditor
{

	enum EditableHandlePosition
	{
		First = -1,
		TopLeft,
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

		internal EditableHandle(EditableAlteringManager manager, EditableHandlePosition position)
			: base(manager)
		{
			Position = ((EditableHandleManager)manager).EditableHandlePositionToPoint(position);
		}

	}
}
